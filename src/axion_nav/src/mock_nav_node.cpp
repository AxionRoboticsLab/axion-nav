#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
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
}  // namespace

/**
 * Mock localization + go-to-goal for axion-console 通车.
 *
 * 协议（与 console / 未来 Nav2 对齐）:
 *   /initialpose  PoseWithCovarianceStamped  ← 重定位
 *   /goal_pose    PoseStamped                ← 去这里 / 收藏点
 *   /cmd_vel      Twist                      ← 摇杆（有速度时取消导航）
 *   /robot_pose   PoseStamped                → 前端箭头
 *   /plan         Path                       → 直线路径可视化
 *   /nav_state    String                     → idle | navigating | arrived
 */
class MockNavNode : public rclcpp::Node
{
public:
  MockNavNode()
  : Node("mock_nav")
  {
    pose_rate_hz_ = declare_parameter<double>("pose_rate_hz", 20.0);
    teleop_scale_ = declare_parameter<double>("teleop_scale", 8.0);
    nav_linear_speed_ = declare_parameter<double>("nav_linear_speed", 0.35);
    nav_angular_speed_ = declare_parameter<double>("nav_angular_speed", 1.2);
    goal_xy_tol_ = declare_parameter<double>("goal_xy_tolerance", 0.08);
    goal_yaw_tol_ = declare_parameter<double>("goal_yaw_tolerance", 0.15);

    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    const auto initialpose_topic =
      declare_parameter<std::string>("initialpose_topic", "/initialpose");
    const auto goal_pose_topic = declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    const auto robot_pose_topic =
      declare_parameter<std::string>("robot_pose_topic", "/robot_pose");
    const auto plan_topic = declare_parameter<std::string>("plan_topic", "/plan");
    const auto nav_state_topic =
      declare_parameter<std::string>("nav_state_topic", "/nav_state");

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      robot_pose_topic, bridge_sub_qos());
    plan_pub_ = create_publisher<nav_msgs::msg::Path>(plan_topic, bridge_pub_qos());
    state_pub_ = create_publisher<std_msgs::msg::String>(nav_state_topic, bridge_pub_qos());

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
        set_state_locked("idle");
        clear_plan_locked();
        RCLCPP_INFO(get_logger(), "goal cancelled by teleop");
      }
    };
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, bridge_sub_qos(), on_cmd);
    cmd_vel_sub_rel_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, bridge_pub_qos(), on_cmd);

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, pose_rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      [this]() { on_timer(); });

    set_state_locked("idle");
    RCLCPP_INFO(
      get_logger(),
      "mock_nav ready. initialpose=%s goal=%s pose=%s",
      initialpose_topic.c_str(), goal_pose_topic.c_str(), robot_pose_topic.c_str());
  }

private:
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
    set_state_locked("idle");
    clear_plan_locked();
    RCLCPP_INFO(
      get_logger(), "initialpose -> (%.2f, %.2f, yaw=%.2f)",
      pose_x_, pose_y_, pose_yaw_);
  }

  void on_goal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_yaw_ = yaw_from_quat(msg->pose.orientation);
    has_goal_ = true;
    navigating_ = true;
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
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped a;
    a.header = path.header;
    a.pose.position.x = pose_x_;
    a.pose.position.y = pose_y_;
    a.pose.orientation = quat_from_yaw(pose_yaw_);

    geometry_msgs::msg::PoseStamped b;
    b.header = path.header;
    b.pose.position.x = goal_x_;
    b.pose.position.y = goal_y_;
    b.pose.orientation = quat_from_yaw(goal_yaw_);

    path.poses = {a, b};
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
        const double vx = last_cmd_.linear.x * scale;
        const double vy = last_cmd_.linear.y * scale;
        const double wz = last_cmd_.angular.z * scale;
        const double c = std::cos(pose_yaw_);
        const double s = std::sin(pose_yaw_);
        pose_x_ += (c * vx - s * vy) * dt;
        pose_y_ += (s * vx + c * vy) * dt;
        pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
      } else if (age >= 0.5) {
        has_cmd_ = false;
      }
    }

    if (!teleop_active && navigating_ && has_goal_) {
      const double dx = goal_x_ - pose_x_;
      const double dy = goal_y_ - pose_y_;
      const double dist = std::hypot(dx, dy);
      const double bearing = std::atan2(dy, dx);
      const double yaw_err = normalize_angle(bearing - pose_yaw_);

      if (dist > goal_xy_tol_) {
        // 先转再走（简易差速）
        if (std::abs(yaw_err) > 0.25) {
          const double wz = std::clamp(
            yaw_err * 2.0, -nav_angular_speed_, nav_angular_speed_);
          pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
        } else {
          const double step = std::min(nav_linear_speed_ * dt, dist);
          pose_x_ += std::cos(pose_yaw_) * step;
          pose_y_ += std::sin(pose_yaw_) * step;
          const double wz = std::clamp(
            yaw_err * 1.5, -nav_angular_speed_, nav_angular_speed_);
          pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
        }
        publish_plan_locked();
      } else {
        const double final_err = normalize_angle(goal_yaw_ - pose_yaw_);
        if (std::abs(final_err) > goal_yaw_tol_) {
          const double wz = std::clamp(
            final_err * 2.0, -nav_angular_speed_, nav_angular_speed_);
          pose_yaw_ = normalize_angle(pose_yaw_ + wz * dt);
        } else {
          pose_yaw_ = goal_yaw_;
          navigating_ = false;
          set_state_locked("arrived");
          clear_plan_locked();
          RCLCPP_INFO(get_logger(), "arrived at goal");
          // 短暂停留在 arrived，下一拍回到 idle 方便前端轮询
          set_state_locked("idle");
        }
      }
    }

    publish_pose_locked();
  }

  double pose_rate_hz_{20.0};
  double teleop_scale_{8.0};
  double nav_linear_speed_{0.35};
  double nav_angular_speed_{1.2};
  double goal_xy_tol_{0.08};
  double goal_yaw_tol_{0.15};

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  bool has_goal_{false};
  bool navigating_{false};
  std::string nav_state_{"idle"};

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};

  std::mutex mutex_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initialpose_sub_rel_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_rel_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_rel_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MockNavNode>());
  rclcpp::shutdown();
  return 0;
}
