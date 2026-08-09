# axion-nav

Axion 机器人 **定位 + 导航到目标** 服务。基于 **ROS 2 Humble**。一期提供 **mock 定位 / 直线去目标** 通车路径，供 [axion-console](https://github.com/AxionRoboticsLab/axion-console) 导航页使用；后续接入 Nav2（AMCL + `navigate_to_pose`）。

> 组织：[AxionRoboticsLab](https://github.com/AxionRoboticsLab)  
> 仓库：<https://github.com/AxionRoboticsLab/axion-nav>

---

## 在整体架构中的位置

```text
[axion-console 浏览器]
        │  WebSocket ws://<host>:9090
        ▼
[rosbridge_server]          ← 通常由 axion-slam bringup 拉起
        │
        ├─ axion-slam       ← 建图 / 载图 → `/map`
        │
        └─ axion-nav        ← 本仓库：定位 + 去目标（一期 mock）
```

| 组件 | 职责 |
|------|------|
| **axion-slam** | `/map_command`、地图文件、`/map` |
| **axion-nav** | `/initialpose`、`/goal_pose`、`/robot_pose`、`/plan` |
| **axion-console** | 导航页：重定位 / 去这里 / 巡检点 |

建图时由 slam 发布 `/robot_pose`；**idle 载图后** slam 不再发 pose，由本包 mock 接管，避免双节点抢话题。

---

## 对外协议（一期 mock）

| 接口 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | 入 | 重定位 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 入 | 去这里 / 巡检点（对齐 Nav2 Simple Goal） |
| `/cmd_vel` | `geometry_msgs/Twist` | 入 | 摇杆；有速度时取消当前 goal |
| `/robot_pose` | `geometry_msgs/PoseStamped` | 出 | 前端机器人箭头 |
| `/plan` | `nav_msgs/Path` | 出 | 当前直线路径（可视化） |
| `/nav_state` | `std_msgs/String` | 出 | `idle` / `navigating` / `arrived` |
| `/robot_status` | `std_msgs/String` (JSON) | 出 | 电量/充电/机型等，默认 **1 Hz**（`status_rate_hz`） |
| `/charge_pose` | `geometry_msgs/PoseStamped` | 入 | 充电点；靠近则 `charging=true`，每分钟电量 +1%，否则 -1% |

`/robot_status` JSON 示例：

```json
{
  "battery": 100,
  "charging": false,
  "model": "Demo-v1",
  "version": "v0.1.0",
  "sn": "AX-DEMO-0001",
  "online": true,
  "work_state": "idle",
  "nav_state": "idle"
}
```

> 真实 Nav2 阶段将改为 AMCL + `navigate_to_pose` Action；`/goal_pose` 话题可保留作薄桥接。

---

## 构建与运行

与 `axion-slam` 同一 colcon workspace 即可：

```bash
cd ~/ws   # 含 src/axion_nav、src/axion_slam
colcon build --packages-select axion_nav
source install/setup.bash

# 终端 1：地图 + rosbridge（已有）
ros2 launch axion_slam bringup.launch.py

# 终端 2：mock 定位与导航
ros2 launch axion_nav mock.launch.py
# 或
ros2 launch axion_nav bringup.launch.py
```

控制台：

1. 打开 **导航** 页，加载地图（如 `v1`）
2. **重定位**：点地图设位姿 → 确定 → 发 `/initialpose`
3. **去这里**（或点巡检点）→ 发 `/goal_pose` → 箭头沿直线移动，`/plan` 可画路径

---

## 一期范围

**做：** mock 定位 + 直线去目标 + console 真发 topic  
**不做：** AMCL、完整 Nav2 代价地图/控制器、障碍规避（下一期）

---

## 包结构

```text
src/axion_nav/
  CMakeLists.txt
  package.xml
  config/mock_nav.yaml
  config/fastdds_no_shm.xml
  launch/mock.launch.py
  launch/bringup.launch.py
  src/mock_nav_node.cpp
```
