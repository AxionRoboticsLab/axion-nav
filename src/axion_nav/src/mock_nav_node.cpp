#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <sstream>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{
rclcpp::QoS bridge_sub_qos()
{
  return rclcpp::QoS(10).best_effort();
}

rclcpp::QoS bridge_pub_qos()
{
  return rclcpp::QoS(10).reliable();
}

double yaw_from_quat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

geometry_msgs::msg::Quaternion quat_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double normalize_angle(double a)
{
  while (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  while (a < -M_PI) {
    a += 2.0 * M_PI;
  }
  return a;
}

/**
 * 与 axion-console 箭头一致：yaw=0 朝 map +Y（屏幕上），
 * 前进方向为 (sin θ, cos θ)，而非 ROS 默认的 (cos θ, sin θ)。
 */
double bearing_to(double from_x, double from_y, double to_x, double to_y)
{
  return std::atan2(to_x - from_x, to_y - from_y);
}

void move_forward(double yaw, double step, double & x, double & y)
{
  x += std::sin(yaw) * step;
  y += std::cos(yaw) * step;
}
}  // namespace

/**
 * Mock localization + go-to-goal for axion-console 通车.
 *
 * 协议（与 console / 未来 Nav2 对齐）:
 *   /initialpose  PoseWithCovarianceStamped  ← 重定位
 *   /goal_pose    PoseStamped                ← 去这里 / 巡检点
 *   /cmd_vel      Twist                      ← 摇杆（有速度时取消导航）
 *   /estop        Bool                       ← 急停（true 触发告警并停导航）
 *   /robot_pose   PoseStamped                → 前端箭头
 *   /plan         Path                       → 直线路径可视化
 *   /nav_state    String                     → idle | navigating | arrived
 *   /robot_status String(JSON)               → 电量/充电/机型等（status_rate_hz）
 *   /alarm_event  String(JSON)               → 告警：定位失败/触边/急停
 *   /charge_pose  PoseStamped                ← 充电点（console 下发，用于判断充电中）
 */
class MockNavNode : public rclcpp::Node
{
public:
  MockNavNode()
  : Node("mock_nav")
  {
    pose_rate_hz_ = declare_parameter<double>("pose_rate_hz", 20.0);
    status_rate_hz_ = declare_parameter<double>("status_rate_hz", 1.0);
    teleop_scale_ = declare_parameter<double>("teleop_scale", 8.0);
    nav_linear_speed_ = declare_parameter<double>("nav_linear_speed", 0.35);
    nav_angular_speed_ = declare_parameter<double>("nav_angular_speed", 1.2);
    goal_xy_tol_ = declare_parameter<double>("goal_xy_tolerance", 0.08);
    goal_yaw_tol_ = declare_parameter<double>("goal_yaw_tolerance", 0.15);
    charge_near_m_ = declare_parameter<double>("charge_near_m", 0.35);
    battery_ = declare_parameter<int>("battery_percent", 100);
    model_ = declare_parameter<std::string>("robot_model", "Demo-v1");
    version_ = declare_parameter<std::string>("robot_version", "v0.1.0");
    sn_ = declare_parameter<std::string>("robot_sn", "AX-DEMO-0001");
    // 告警：定时轮播三种异常；0 关闭自动造异常
    alarm_demo_period_sec_ = declare_parameter<double>("alarm_demo_period_sec", 45.0);
    alarm_cooldown_sec_ = declare_parameter<double>("alarm_cooldown_sec", 25.0);
    edge_bound_m_ = declare_parameter<double>("edge_bound_m", 4.5);

    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto initialpose_topic =
      declare_parameter<std::string>("initialpose_topic", "/initialpose");
    const auto goal_pose_topic = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    const auto robot_pose_topic =
      declare_parameter<std::string>("robot_pose_topic", "/robot_pose");
    const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
    const auto nav_state_topic =
      declare_parameter<std::string>("nav_state_topic", "/nav_state");
    const auto robot_status_topic =
      declare_parameter<std::string>("robot_status_topic", "/robot_status");
    const auto charge_pose_topic =
      declare_parameter<std::string>("charge_pose_topic", "/charge_pose");
    const auto alarm_event_topic =
      declare_parameter<std::string>("alarm_event_topic", "/alarm_event");
    const auto estop_topic = declare_parameter<std::string>("estop_topic", "/estop");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic, bridge_sub_qos());
    plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic, bridge_pub_qos());
    state_pub_ = create_publisher<std_msgs::msg::String>(nav_state_topic, bridge_pub_qos());
    status_pub_ = create_publisher<std_msgs::msg::String>(robot_status_topic, bridge_pub_qos());
    alarm_pub_ = create_publisher<std_msgs::msg::String>(alarm_event_topic, bridge_pub_qos());

    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic, bridge_sub_qos(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        on_initialpose(msg);
      });
    initialpose_sub_rel_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic, bridge_pub_qos(),
      [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        on_initialpose(msg);
      });

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, bridge_sub_qos(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        on_goal(msg);
      });
    goal_sub_rel_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, bridge_pub_qos(),
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        on_goal(msg);
      });

    auto on_cmd = [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      const double speed =
        std::hypot(msg->linear.x, msg->linear.y) + std::abs(msg->angular.z);
      // 摇杆松手会发一次 0 速；不能把 has_cmd 当成“占着导航”
      if (speed <= 1e-3) {
        has_cmd_ = false;
        last_cmd_ = geometry_msgs::msg::Twist();
        return;
      }
      last_cmd_ = *msg;
      last_cmd_time_ = now();
      has_cmd_ = true;
      if (navigating_) {
        navigating_ = false;
        phase_ = NavPhase::Idle;
        pending_idle_ = false;
        set_state_locked("idle");
        clear_plan_locked();
        RCLCPP_INFO(get_logger(), "goal cancelled by teleop");
      }
    };
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, bridge_sub_qos(), on_cmd);
    cmd_vel_sub_rel_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, bridge_pub_qos(), on_cmd);

    auto on_charge = [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      on_charge_pose(msg);
    };
    charge_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      charge_pose_topic, bridge_sub_qos(), on_charge);
    charge_sub_rel_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      charge_pose_topic, bridge_pub_qos(), on_charge);

    auto on_estop = [this](const std_msgs::msg::Bool::SharedPtr msg) {
      on_estop_msg(msg);
    };
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, bridge_sub_qos(), on_estop);
    estop_sub_rel_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic, bridge_pub_qos(), on_estop);

    battery_ = std::clamp(battery_, 0, 100);
    last_battery_tick_ = now();
    last_alarm_demo_ = now();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pose_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { on_timer(); });

    set_state_locked("idle");
    RCLCPP_INFO(
      get_logger(),
      "mock_nav ready. pose=%s status=%s @ %.1fHz alarm=%s demo=%.0fs edge=%.1fm",
      robot_pose_topic.c_str(), robot_status_topic.c_str(), status_rate_hz_,
      alarm_event_topic.c_str(), alarm_demo_period_sec_, edge_bound_m_);
  }

private:
  enum class NavPhase { Idle, AlignBearing, Drive, AlignGoal };

  void set_state_locked(const std::string & s)
  {
    if (nav_state_ == s) {
      return;
    }
    nav_state_ = s;
    std_msgs::msg::String msg;
    msg.data = nav_state_;
    state_pub_->publish(msg);
  }

  void on_initialpose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_yaw_ = yaw_from_quat(msg->pose.pose.orientation);
    navigating_ = false;
    phase_ = NavPhase::Idle;
    pending_idle_ = false;
    loc_ok_ = true;
    edge_hit_ = false;
    set_state_locked("idle");
    clear_plan_locked();
    RCLCPP_INFO(
      get_logger(), "initialpose -> (%.2f, %.2f, yaw=%.2f)",
      pose_x_, pose_y_, pose_yaw_);
  }

  void on_estop_msg(const std_msgs::msg::Bool::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!msg->data) {
      estop_ = false;
      return;
    }
    apply_estop_locked("急停按钮触发，导航已中断");
  }

  void apply_estop_locked(const std::string & detail)
  {
    estop_ = true;
    if (navigating_) {
      navigating_ = false;
      phase_ = NavPhase::Idle;
      pending_idle_ = false;
      set_state_locked("idle");
      clear_plan_locked();
    }
    publish_alarm_locked(
      "emergency_stop", "critical", "急停", detail);
  }

  /** 发布告警事件；同 code 在 cooldown 内去重 */
  void publish_alarm_locked(
    const std::string & code,
    const std::string & level,
    const std::string & event,
    const std::string & detail)
  {
    const double t = now().seconds();
    const auto it = last_alarm_by_code_.find(code);
    if (it != last_alarm_by_code_.end() &&
      (t - it->second) < alarm_cooldown_sec_)
    {
      return;
    }
    last_alarm_by_code_[code] = t;

    if (code == "localization_lost") {
      loc_ok_ = false;
    } else if (code == "edge_collision") {
      edge_hit_ = true;
    } else if (code == "emergency_stop") {
      estop_ = true;
    }

    std::ostringstream oss;
    oss << '{'
        << "\"code\":\"" << code << "\","
        << "\"level\":\"" << level << "\","
        << "\"event\":\"" << event << "\","
        << "\"detail\":\"" << detail << "\","
        << "\"source\":\"mock_nav\","
        << "\"ts\":" << static_cast<std::int64_t>(t)
        << '}';
    std_msgs::msg::String msg;
    msg.data = oss.str();
    alarm_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "alarm_event %s: %s", code.c_str(), event.c_str());
  }

  void maybe_demo_alarm_locked(double dt)
  {
    if (alarm_demo_period_sec_ <= 0.0) {
      return;
    }
    alarm_demo_accum_ += dt;
    if (alarm_demo_accum_ < alarm_demo_period_sec_) {
      return;
    }
    alarm_demo_accum_ = 0.0;
    const int step = alarm_demo_idx_ % 3;
    alarm_demo_idx_ += 1;
    if (step == 0) {
      publish_alarm_locked(
        "localization_lost", "critical", "定位失败",
        "mock: AMCL/定位置信度过低（演示注入）");
    } else if (step == 1) {
      publish_alarm_locked(
        "edge_collision", "warn", "触边",
        "mock: 保险杠/触边传感器触发（演示注入）");
    } else {
      apply_estop_locked("mock: 急停回路断开（演示注入）");
    }
  }

  void maybe_edge_alarm_locked()
  {
    if (std::abs(pose_x_) > edge_bound_m_ || std::abs(pose_y_) > edge_bound_m_) {
      publish_alarm_locked(
        "edge_collision", "warn", "触边",
        "机器人接近虚拟边界，疑似触边");
    }
  }

  void on_charge_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    charge_x_ = msg->pose.position.x;
    charge_y_ = msg->pose.position.y;
    has_charge_ = true;
    RCLCPP_INFO(
      get_logger(), "charge_pose -> (%.2f, %.2f)", charge_x_, charge_y_);
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (estop_) {
      RCLCPP_WARN(get_logger(), "goal ignored: estop active");
      publish_alarm_locked(
        "emergency_stop", "critical", "急停",
        "急停生效中，拒绝导航目标");
      return;
    }
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_yaw_ = yaw_from_quat(msg->pose.orientation);
    // 固定全局路径起点，便于前端画「已走/未走」
    plan_start_x_ = pose_x_;
    plan_start_y_ = pose_y_;
    plan_start_yaw_ = pose_yaw_;
    has_goal_ = true;
    navigating_ = true;
    pending_idle_ = false;
    // 三相：先对目标方位 → 直线前进 → 到位后转目标朝向
    const double already = std::hypot(goal_x_ - pose_x_, goal_y_ - pose_y_);
    phase_ = already <= goal_xy_tol_ ? NavPhase::AlignGoal : NavPhase::AlignBearing;
    set_state_locked("navigating");
    publish_plan_locked();
    RCLCPP_INFO(
      get_logger(), "goal_pose -> (%.2f, %.2f, yaw=%.2f)",
      goal_x_, goal_y_, goal_yaw_);
  }

  void clear_plan_locked()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    plan_pub_->publish(path);
  }

  void publish_plan_locked()
  {
    // 发布固定折线（起点→目标），行驶过程中不再改起点
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";

    constexpr int k_segments = 24;
    path.poses.reserve(static_cast<size_t>(k_segments) + 1);
    for (int i = 0; i <= k_segments; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(k_segments);
      geometry_msgs::msg::PoseStamped p;
      p.header = path.header;
      p.pose.position.x = plan_start_x_ + (goal_x_ - plan_start_x_) * t;
      p.pose.position.y = plan_start_y_ + (goal_y_ - plan_start_y_) * t;
      p.pose.position.z = 0.0;
      const double yaw = (i < k_segments)
        ? bearing_to(plan_start_x_, plan_start_y_, goal_x_, goal_y_)
        : goal_yaw_;
      p.pose.orientation = quat_from_yaw(yaw);
      path.poses.push_back(p);
    }
    plan_pub_->publish(path);
  }

  void publish_pose_locked()
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.x = pose_x_;
    msg.pose.position.y = pose_y_;
    msg.pose.position.z = 0.0;
    msg.pose.orientation = quat_from_yaw(pose_yaw_);
    pose_pub_->publish(msg);
  }

  bool near_charge_locked() const
  {
    if (!has_charge_) {
      return false;
    }
    return std::hypot(pose_x_ - charge_x_, pose_y_ - charge_y_) <= charge_near_m_;
  }

  /** 在充电点：每分钟 +1%；否则每分钟 -1% */
  void update_battery_locked()
  {
    const double elapsed = (now() - last_battery_tick_).seconds();
    if (elapsed < 60.0) {
      return;
    }
    const int mins = static_cast<int>(elapsed / 60.0);
    charging_ = near_charge_locked();
    const int delta = charging_ ? mins : -mins;
    battery_ = std::clamp(battery_ + delta, 0, 100);
    last_battery_tick_ = last_battery_tick_ + rclcpp::Duration::from_seconds(mins * 60.0);
  }

  void publish_status_locked()
  {
    charging_ = near_charge_locked();
    std::string work = "idle";
    if (nav_state_ == "navigating") {
      work = "navigating";
    } else if (nav_state_ == "arrived") {
      work = "idle";
    }

    std::ostringstream oss;
    oss << '{'
        << "\"battery\":" << battery_ << ','
        << "\"charging\":" << (charging_ ? "true" : "false") << ','
        << "\"model\":\"" << model_ << "\","
        << "\"version\":\"" << version_ << "\","
        << "\"sn\":\"" << sn_ << "\","
        << "\"online\":true,"
        << "\"work_state\":\"" << work << "\","
        << "\"nav_state\":\"" << nav_state_ << "\","
        << "\"estop\":" << (estop_ ? "true" : "false") << ','
        << "\"loc_ok\":" << (loc_ok_ ? "true" : "false") << ','
        << "\"edge_hit\":" << (edge_hit_ ? "true" : "false")
        << '}';

    std_msgs::msg::String msg;
    msg.data = oss.str();
    status_pub_->publish(msg);
  }

  void on_timer()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const double dt = 1.0 / std::max(1.0, pose_rate_hz_);

    // 仅非零 cmd_vel 抢占导航；零速/超时不挡 goal
    bool teleop_active = false;
    if (has_cmd_) {
      const double age = (now() - last_cmd_time_).seconds();
      const double speed =
        std::hypot(last_cmd_.linear.x, last_cmd_.linear.y) +
        std::abs(last_cmd_.angular.z);
      if (age < 0.5 && speed > 1e-3) {
        teleop_active = true;
        const double scale = std::max(0.1, teleop_scale_);
        // 摇杆：x=右移，y=前进（与箭头朝向一致）
        const double vx = last_cmd_.linear.x * scale;
        const double vy = last_cmd_.linear.y * scale;
        const double wz = last_cmd_.angular.z * scale;
        const double c = std::cos(pose_yaw_);
        const double s = std::sin(pose_yaw_);
        pose_x_ += (c * vx + s * vy) * dt;
        pose_y_ += (-s * vx + c * vy) * dt;
        pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
      } else if (age >= 0.5) {
        has_cmd_ = false;
      }
    }

    // arrived 保留一拍，再进 idle，方便前端连跑巡检
    if (pending_idle_ && !navigating_) {
      pending_idle_ = false;
      set_state_locked("idle");
    }

    if (!teleop_active && navigating_ && has_goal_ && !estop_) {
      if (phase_ == NavPhase::AlignBearing) {
        const double bearing = bearing_to(pose_x_, pose_y_, goal_x_, goal_y_);
        const double yaw_err = normalize_angle(bearing - pose_yaw_);
        if (std::abs(yaw_err) > bearing_yaw_tol_) {
          const double wz = std::clamp(
            yaw_err * 2.0, -nav_angular_speed_, nav_angular_speed_);
          pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
        } else {
          drive_yaw_ = bearing;
          pose_yaw_ = bearing;
          phase_ = NavPhase::Drive;
        }
      } else if (phase_ == NavPhase::Drive) {
        const double dist = std::hypot(goal_x_ - pose_x_, goal_y_ - pose_y_);
        if (dist > goal_xy_tol_) {
          // 始终朝向目标点，再沿箭头前进（禁止侧移/后移）
          drive_yaw_ = bearing_to(pose_x_, pose_y_, goal_x_, goal_y_);
          pose_yaw_ = drive_yaw_;
          const double step = std::min(nav_linear_speed_ * dt, dist);
          move_forward(drive_yaw_, step, pose_x_, pose_y_);
        } else {
          phase_ = NavPhase::AlignGoal;
        }
      } else if (phase_ == NavPhase::AlignGoal) {
        const double final_err = normalize_angle(goal_yaw_ - pose_yaw_);
        if (std::abs(final_err) > goal_yaw_tol_) {
          const double wz = std::clamp(
            final_err * 2.0, -nav_angular_speed_, nav_angular_speed_);
          pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
        } else {
          pose_yaw_ = goal_yaw_;
          navigating_ = false;
          phase_ = NavPhase::Idle;
          set_state_locked("arrived");
          clear_plan_locked();
          pending_idle_ = true;
          RCLCPP_INFO(get_logger(), "arrived at goal");
        }
      }
    }

    publish_pose_locked();

    maybe_edge_alarm_locked();
    maybe_demo_alarm_locked(dt);

    // 机器人状态：默认 1Hz 推送 JSON（电量 / 充电 / 机型…）
    status_accum_ += dt;
    const double status_period = 1.0 / std::max(0.1, status_rate_hz_);
    if (status_accum_ >= status_period) {
      status_accum_ = 0.0;
      update_battery_locked();
      publish_status_locked();
    }
  }

  double pose_rate_hz_{20.0};
  double status_rate_hz_{1.0};
  double teleop_scale_{8.0};
  double nav_linear_speed_{0.35};
  double nav_angular_speed_{1.2};
  double goal_xy_tol_{0.08};
  double goal_yaw_tol_{0.15};
  double bearing_yaw_tol_{0.08};
  double charge_near_m_{0.35};
  double status_accum_{0.0};
  double alarm_demo_period_sec_{45.0};
  double alarm_cooldown_sec_{25.0};
  double edge_bound_m_{4.5};
  double alarm_demo_accum_{0.0};
  int alarm_demo_idx_{0};

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  double plan_start_x_{0.0};
  double plan_start_y_{0.0};
  double plan_start_yaw_{0.0};
  double drive_yaw_{0.0};
  double charge_x_{0.0};
  double charge_y_{0.0};
  bool has_charge_{false};
  bool charging_{false};
  bool estop_{false};
  bool loc_ok_{true};
  bool edge_hit_{false};
  int battery_{100};
  std::string model_{"Demo-v1"};
  std::string version_{"v0.1.0"};
  std::string sn_{"AX-DEMO-0001"};
  bool has_goal_{false};
  bool navigating_{false};
  bool pending_idle_{false};
  NavPhase phase_{NavPhase::Idle};
  std::string nav_state_{"idle"};
  std::map<std::string, double> last_alarm_by_code_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_battery_tick_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_alarm_demo_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};

  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alarm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_rel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_rel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_rel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr charge_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr charge_sub_rel_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_rel_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockNavNode>());
  rclcpp::shutdown();
  return 0;
}
