# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

同济大学SuperPower战队2025赛季RoboMaster自瞄视觉系统。本项目提出"轨迹视角下的自瞄理论"，采用轨迹规划算法替代传统的分段决策逻辑，实现更平滑、更精准的云台控制。

## 编译与运行

```bash
# 编译
cmake -B build
make -C build/ -j`nproc`

# 各兵种主程序（通过配置文件指定参数）
./build/standard              # 步兵（默认配置configs/standard3.yaml）
./build/standard <配置路径>   # 指定配置文件，如./build/standard configs/uav.yaml
./build/standard_mpc          # 步兵（MPC规划器版本）
./build/mt_standard           # 步兵（多线程版本）
./build/uav                   # 无人机
./build/sentry                # 哨兵（需ROS2环境）
./build/sentry_multithread    # 哨兵（多线程版本）
#示例：./build/auto_aim_debug_mpc ./configs/standard3.yaml

# 测试程序
./build/auto_aim_test       # 自瞄离线测试（视频）
./build/auto_buff_test      # 打符离线测试
./build/camera_test         # 相机测试
./build/camera_detect_test  # 工业相机识别测试
./build/camera_thread_test  # 相机线程测试
./build/cboard_test         # C板通信测试
./build/detector_video_test # 识别器视频测试
./build/dm_test             # 达妙IMU测试
./build/fire_test           # 开火测试
./build/gimbal_test         # 云台通信测试
./build/gimbal_response_test # 云台响应测试
./build/handeye_test        # 手眼标定测试
./build/minimum_vision_system # 最小视觉系统测试
./build/multi_usbcamera_test # 多USB相机测试
./build/planner_test        # 规划器实车测试
./build/planner_test_offline # 规划器离线测试
./build/usbcamera_test      # USB相机测试
./build/usbcamera_detect_test # USB相机识别测试
# ROS2相关（需ROS2环境）
./build/publish_test        # ROS发布测试
./build/subscribe_test      # ROS订阅测试
./build/topic_loop_test     # ROS话题循环测试
```

## 代码架构

```
io/          - 硬件抽象层
tasks/       - 应用逻辑模块
tools/       - 工具层
src/         - 各兵种主程序入口
tests/       - 独立模块测试程序
calibration/ - 标定程序
```

### io/ 硬件抽象层

| 文件 | 功能 |
|------|------|
| `camera.hpp` | 相机接口，支持海康/USB相机，读取图像+时间戳 |
| `cboard.hpp` | 下位机通信，获取IMU姿态四元数、档位、弹速，发送瞄准指令 |
| `command.hpp` | 控制指令结构（yaw/pitch角度、射击控制） |
| `gimbal.hpp` | 云台控制 |
| `dm_imu.hpp` | 达妙IMU接口 |
| `socketcan.hpp` | CAN总线通信 |
| `ros2.hpp` | ROS2接口（哨兵导航通信） |

### tasks/auto_aim/ 自瞄模块

| 文件/目录 | 功能 |
|----------|------|
| `detector.hpp` | 识别器基类 |
| `yolo.hpp` | YOLO检测器（统一接口） |
| `yolos/` | YOLOv5/v8/YOLO11具体实现 |
| `armor.hpp` | 装甲板数据结构（灯条、装甲板、颜色、类型、优先级） |
| `target.hpp` | 目标状态（包含EKF滤波器，管理单个目标） |
| `tracker.hpp` | 目标追踪器（多目标管理、关联、更新） |
| `solver.hpp` | PnP位姿解算（装甲板像素坐标→相机坐标系→云台坐标系→世界坐标系） |
| `classifier.hpp` | 装甲板数字分类 |
| `voter.hpp` | 多目标投票决策 |
| `planner/` | 轨迹规划器 |
| `planner/planner.hpp` | 规划器基类 |
| `planner/tinympc/` | TinyMPC求解器实现 |
| `aimer.hpp` | 核心瞄准逻辑（整合追踪、解算、规划） |
| `shooter.hpp` | 开火决策（轨迹偏差+延迟补偿） |

### tasks/auto_buff/ 打符模块

| 文件 | 功能 |
|------|------|
| `buff_detector.hpp` | 能量机关检测器 |
| `buff_predict.hpp` | 能量机关预测（风扇叶片位置预测） |
| `buff_solver.hpp` | 能量机关解算 |
| `buff_aimer.hpp` | 能量机关瞄准 |
| `buff_type.hpp` | 能量机关类型 |

### tools/ 工具层

| 文件 | 功能 |
|------|------|
| `logger.hpp` | 日志（基于spdlog） |
| `plotter.hpp` | 曲线绘制（调试可视化） |
| `recorder.hpp` | 视频录制（时间戳+视频+四元数） |
| `extended_kalman_filter.hpp` | 扩展卡尔曼滤波器 |
| `thread_safe_queue.hpp` | 线程安全队列 |
| `yaml.hpp` | YAML配置解析 |
| `trajectory.hpp` | 弹道解算 |
| `img_tools.hpp` | 图像处理工具 |
| `math_tools.hpp` | 数学工具 |
| `pid.hpp` | PID控制器 |

### 数据流

```
相机 → Detector.detect() → Armor列表
                            ↓
                     Tracker.track() → Target列表（带EKF状态）
                            ↓
                     Solver.set_R_gimbal2world(q) → 坐标系变换
                            ↓
                     Aimer.aim() → 瞄准点 + 开火决策
                            ↓
                     CBoard.send() → 下位机（CAN/串口）
```

### 核心概念

- **Armor（装甲板）**：单个装甲板的几何信息（灯条、中心点、类型、颜色）
- **Target（目标）**：一个敌方单位，包含EKF状态估计、多个装甲板列表
- **Command（指令）**：最终发送给下位机的控制量（yaw/pitch角度、是否射击）

### 运行流程

主循环（[src/standard.cpp](src/standard.cpp)）：

```cpp
while (!exiter.exit()) {
    // 1. 读取图像和时间戳
    camera.read(img, t);

    // 2. 获取云台姿态四元数（时间同步）
    q = gimbal.q(t - 1ms);

    // 3. 更新档位模式
    mode = cboard.mode;

    // 4. 设置坐标系变换矩阵
    solver.set_R_gimbal2world(q);

    // 5. 识别装甲板
    auto armors = detector.detect(img);

    // 6. 追踪目标（关联 + EKF更新）
    auto targets = tracker.track(armors, t);

    // 7. 瞄准决策
    auto command = aimer.aim(targets, t, cboard.bullet_speed);

    // 8. 发送指令到下位机
    cboard.send(command);
}
```

#### Aimer.aim() 内部逻辑

```
输入：Target列表、时间戳、弹速
│
├─ 1. 选择目标（优先序）
│
├─ 2. 目标状态预测
│   └─ 根据延迟时间预测未来位置
│
├─ 3. 弹道解算（迭代求解）
│   ├─ 计算瞄准点到装甲板的距离d
│   ├─ 考虑重力补偿，计算pitch角度
│   └─ 若无解（弹速不足）返回无效
│
├─ 4. 迭代优化（最多10次）
│   ├─ 预测子弹飞行时间后的目标位置
│   ├─ 重新计算弹道
│   └─ 收敛则停止
│
└─ 5. 输出Command {control, shoot, yaw, pitch}
```

#### Tracker.track() 内部逻辑

```
输入：Armor列表、时间戳
│
├─ 1. 状态机更新
│   └─ found / lost / temp_lost
│
├─ 2. 目标关联
│   ├─ 若未锁定目标 → 选择装甲板创建新Target
│   └─ 若已锁定目标 → 匹配最近装甲板更新Target
│
├─ 3. EKF预测（根据dt）
│
├─ 4. EKF更新（观测装甲板位置）
│
├─ 5. 判断是否跳变（装甲板切换）
│
└─ 6. 输出Target列表
```

#### Solver 解算流程

```
输入：Armor（像素坐标）、R_gimbal2world（四元数转旋转矩阵）
│
├─ 1. PnP求解装甲板在相机坐标系下的位置
│
├─ 2. 坐标系变换
│   └─ 相机坐标系 → 云台坐标系 → 世界坐标系
│
├─ 3. Yaw优化（如需要）
│   └─ 通过重投影误差优化yaw角
│
└─ 4. 输出装甲板在世界坐标系下的xyz
```

### 配置文件

`configs/` 目录下各机器人YAML配置：
- `standard*.yaml` - 步兵
- `sentry*.yaml` - 哨兵（依赖ROS2）
- `uav*.yaml` - 无人机

## 手眼标定

### 坐标系定义

| 坐标系 | 说明 |
|--------|------|
| 相机坐标系 | 工业相机光心为原点 |
| 云台坐标系 | 云台旋转中心 |
| IMU机体坐标系 | 惯性测量单元本地坐标系（与云台固连） |
| 世界坐标系 | 场地固定坐标系 |

**标定目标**：求解 `R_camera2gimbal` 和 `t_camera2gimbal`（相机到云台的固定变换）

### 标定流程

#### 1. 准备工作

- 圆点标定板（建议 10×7 或 11×8 圆点）
- 圆点间距 `center_distance_mm: 40`（根据实际标定板设置）
- 创建数据目录：
  ```bash
  mkdir -p assets/img_with_q
  ```

#### 2. 数据采集

运行标定测试程序：
```bash
./build/handeye_test -c configs/standard3.yaml -d
```

- 手动旋转云台到**不同角度**（覆盖 yaw: -45°~+45°, pitch: -30°~+30°）
- 标定板距相机 0.5m~1.5m
- 采集 **20~30组** 数据（每组：云台静止 + 标定板清晰可见）
- 在标定程序中按 `s` 保存当前帧数据到 `assets/img_with_q/{n}.jpg`（需要自行添加保存功能，或截图）

每组数据包含：
| 文件 | 内容 |
|------|------|
| `{n}.jpg` | 标定板图片 |
| `{n}.txt` | 对应时刻云台四元数 `(w x y z)` |

#### 3. 运行标定程序

```bash
./build/calibrate_handeye assets/img_with_q -c configs/calibration.yaml
```

**标定算法**：
1. `cv::findCirclesGrid()` 识别标定板圆点
2. `solvePnP()` 求相机到标定板的变换
3. `cv::calibrateHandEye()` 求解手眼关系

**输出**：
```yaml
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
R_camera2gimbal: [r11, r12, r13, ...]
t_camera2gimbal: [tx, ty, tz]
# 相机同理想情况的偏角: yaw_x.xx pitch_x.xx roll_x.xx degree
```

#### 4. 更新配置文件

将标定结果填入 `configs/standard3.yaml`：

```yaml
R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]
R_camera2gimbal: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
t_camera2gimbal: [tx, ty, tz]  # 单位：米
```

#### 5. 验证标定结果

```bash
./build/handeye_test -c configs/standard3.yaml -d
```

观察：白色圆点为重投影的场地特征点，应与标定板实际位置吻合。

### 坐标变换链

```
装甲板像素 → solvePnP → 相机坐标系 → [R_camera2gimbal] → 云台坐标系 → [R_gimbal2world] → 世界坐标系
                                                              ↑
                                                    IMU四元数（实时更新）
```

### configs/calibration.yaml 完整参数

```yaml
pattern_cols: 11          # 标定板圆点列数
pattern_rows: 8           # 标定板圆点行数
center_distance_mm: 20    # 圆点间距（毫米）

R_gimbal2imubody: [1, 0, 0, 0, 1, 0, 0, 0, 1]  # 云台到IMU的固定旋转

camera_name: "hikrobot"   # 相机类型
exposure_ms: 3
gain: 10.0
vid_pid: "2bdf:0001"

#####-----cboard参数-----#####
quaternion_canid: 0x01
bullet_speed_canid: 0x110
send_canid: 0xff
can_interface: "can0"
```

## 依赖

- OpenCV、fmt、Eigen3、spdlog、yaml-cpp、nlohmann_json
- OpenVINO（Intel GPU推理）
- Ceres（打符求解器）
- 可选：ROS2（哨兵与导航通信）

## 关键源文件

- [src/standard.cpp](src/standard.cpp) - 步兵主程序
- [src/sentry.cpp](src/sentry.cpp) - 哨兵主程序（依赖ROS2）
- [tasks/auto_aim/aimer.hpp](tasks/auto_aim/aimer.hpp) - 核心瞄准逻辑
- [tasks/auto_aim/planner/planner.hpp](tasks/auto_aim/planner/planner.hpp) - 轨迹规划器
- [tasks/auto_aim/tracker.hpp](tasks/auto_aim/tracker.hpp) - EKF目标追踪
- [io/camera.hpp](io/camera.hpp) - 相机接口
- [io/cboard.hpp](io/cboard.hpp) - 下位机通信（IMU、档位、弹速）
- [tools/extended_kalman_filter.hpp](tools/extended_kalman_filter.hpp) - 扩展卡尔曼滤波器
