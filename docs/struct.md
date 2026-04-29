# Open-VINS ROS2 Topic 结构文档

> **ROS版本**: ROS2 (Humble/Foxy)
> **主要实现文件**: `src/ov_msckf/src/ros/ROS2Visualizer.cpp`
> **文档更新时间**: 2026-04-29

---

## 完整Topic列表

| Topic名称 | 消息类型 | 方向 | 数据含义 | 使用场景 |
|-----------|---------|------|----------|---------|
| **输入数据** ||||
| `/imu0` | `sensor_msgs::msg::Imu` | 订阅 | IMU测量数据：角速度(angular_velocity, rad/s) + 线加速度(linear_acceleration, m/s²)，频率~200Hz | 传感器输入，用于状态传播和滤波更新 |
| `/cam0/image_raw` | `sensor_msgs::msg::Image` | 订阅 | 相机0原始图像数据，用于特征提取和跟踪 | 单目或双目左目相机输入 |
| `/cam1/image_raw` | `sensor_msgs::msg::Image` | 订阅 | 相机1原始图像数据（双目模式），与cam0时间同步 | 双目右目相机输入 |
| **位姿估计输出** ||||
| `/ov_msckf/poseimu` | `geometry_msgs::msg::PoseWithCovarianceStamped` | 发布 | IMU在全局坐标系下的位姿估计：位置(x,y,z) + 姿态四元数(x,y,z,w) + 6x6协方差矩阵 | 基础定位输出，用于导航、控制、与其他节点通信 |
| `/ov_msckf/odomimu` | `nav_msgs::msg::Odometry` | 发布 | IMU里程计信息：位姿 + 速度(线速度+角速度) + 协方差，坐标系：global→imu | 高频率位姿估计，需要速度信息的应用 |
| `/ov_msckf/pathimu` | `nav_msgs::msg::Path` | 发布 | IMU位姿历史轨迹序列，包含所有历史位姿估计 | RViz轨迹可视化、路径规划参考 |
| **真值数据** ||||
| `/ov_msckf/posegt` | `geometry_msgs::msg::PoseStamped` | 发布 | 真实位姿（来自仿真或数据集标注） | 算法精度评估、轨迹对比 |
| `/ov_msckf/pathgt` | `nav_msgs::msg::Path` | 发布 | 真值轨迹历史序列 | 轨迹可视化对比、RMSE计算 |
| **特征点云** ||||
| `/ov_msckf/points_msckf` | `sensor_msgs::msg::PointCloud2` | 发布 | MSCKF特征点的3D位置：滑动窗口中被三角化的特征点，用于构建测量约束 | 特征点可视化、地图构建 |
| `/ov_msckf/points_slam` | `sensor_msgs::msg::PointCloud2` | 发布 | SLAM特征点的3D位置：被加入状态向量的长期跟踪特征 | 长期定位、回环检测候选点 |
| `/ov_msckf/points_aruco` | `sensor_msgs::msg::PointCloud2` | 发布 | ARUCO标记特征点的3D位置 | 已知标记定位、视觉标记检测 |
| `/ov_msckf/points_sim` | `sensor_msgs::msg::PointCloud2` | 发布 | 仿真环境中的真实特征点位置（仅仿真模式有效） | 仿真验证、误差分析 |
| **可视化数据** ||||
| `/ov_msckf/trackhist` | `sensor_msgs::msg::Image` | 发布 | 特征跟踪历史图像：当前帧图像上绘制特征点跟踪轨迹 | 调试特征跟踪质量、诊断跟踪问题 |
| **回环检测信息** ||||
| `/ov_msckf/loop_pose` | `nav_msgs::msg::Odometry` | 发布 | 历史关键帧的IMU位姿估计 | 为回环检测提供关键帧位姿候选 |
| `/ov_msckf/loop_feats` | `sensor_msgs::msg::PointCloud` | 发布 | 回环检测特征点：包含3D位置和2D投影信息(uv_norm, uv_raw, feature_id) | 回环检测的特征匹配 |
| `/ov_msckf/loop_extrinsic` | `nav_msgs::msg::Odometry` | 发布 | IMU到相机的外参变换矩阵 | 回环检测中的坐标变换 |
| `/ov_msckf/loop_intrinsics` | `sensor_msgs::msg::CameraInfo` | 发布 | 相机内参：焦距(fx,fy)、主点(cx,cy)、畸变系数 | 回环检测的投影模型 |
| `/ov_msckf/loop_depth` | `sensor_msgs::msg::Image` | 发布 | 稀疏深度图：特征点对应的深度值 | 深度估计可视化 |
| `/ov_msckf/loop_depth_colored` | `sensor_msgs::msg::Image` | 发布 | 彩色深度可视化图像：深度值映射为颜色 | 深度直观可视化 |
| **Debug调试数据** ||||
| `/ov_msckf/debug/features` | `ov_msckf::msg::DebugFeatures` | 发布 | 特征统计：提取数(num_extracted)、跟踪数(num_tracked)、三角化数(num_3d)、活跃数(num_active) | 监控特征跟踪质量、系统健康度 |
| `/ov_msckf/debug/state` | `ov_msckf::msg::DebugState` | 发布 | 完整IMU状态(16维)：位置、速度、陀螺仪偏置、加速度计偏置、姿态四元数 | 详细状态监控、离线数据分析 |
| `/ov_msckf/debug/residuals` | `ov_msckf::msg::DebugResiduals` | 发布 | 优化残差：视觉重投影残差均值(res_visual, 像素)、IMU预积分残差均值(res_imu) | 优化质量监控、异常检测 |
| **TF变换** ||||
| `/tf` | `tf2_msgs::msg::TFMessage` | 发布 | 动态坐标变换：global→imu（IMU位姿）、imu→cam{i}（相机外参） | 多坐标系可视化、传感器融合 |
| `/tf_static` | `tf2_msgs::msg::TFMessage` | 发布 | 静态坐标变换：固定的坐标系关系 | 静态外参发布 |
| **ROS2系统Topic** ||||
| `/rosout` | `rcl_interfaces::msg::Log` | 发布 | ROS2节点日志输出：INFO/WARNING/ERROR等级日志 | 系统日志监控、调试 |
| `/parameter_events` | `rcl_interfaces::msg::ParameterEvent` | 发布 | 参数变化事件：节点参数的设置和更新记录 | 参数监控、动态调参 |
| `/clock` | `rosgraph_msgs::msg::Clock` | 发布/订阅 | 仿真时钟：用于时间同步（仅仿真或use_sim_time=true时有效） | 仿真时间同步、bag播放 |
| **RViz交互Topic** ||||
| `/clicked_point` | `geometry_msgs::msg::PointStamped` | 发布 | RViz中点击的3D点坐标 | 用户交互、标记点选择 |
| `/initialpose` | `geometry_msgs::msg::PoseWithCovarianceStamped` | 发布 | RViz中设置的初始位姿（"2D Pose Estimate"按钮） | 手动设置初始位姿 |
| `/move_base_simple/goal` | `geometry_msgs::msg::PoseStamped` | 发布 | RViz中设置的导航目标点（"2D Nav Goal"按钮） | 简单导航目标设置 |
| **其他Topic** ||||
| `/events/read_split` | `rosbag2_interfaces::msg::ReadSplit` | 发布 | Rosbag播放事件：bag文件读取分片信息 | Bag播放监控 |
| `/vrpn_client/raw_transform` | `geometry_msgs::msg::TransformStamped` | 发布 | VRPN动捕系统原始变换数据 | 外部动捕系统真值输入 |

---

## 自定义消息定义

### DebugFeatures.msg

**文件**: `src/ov_msckf/msg/DebugFeatures.msg`

```
std_msgs/Header header
int32 num_extracted    # 当前帧提取的特征点数量
int32 num_tracked      # 成功跟踪到上一帧的特征点数量
int32 num_3d           # 已三角化的3D特征点数量
int32 num_active       # 当前活跃的SLAM特征数量（在状态向量中）
```

**监控指标**:
- `num_tracked / num_extracted`: 跟踪成功率，应>80%
- `num_3d`: 三角化质量指标
- `num_active`: SLAM特征数量，影响定位精度

### DebugState.msg

**文件**: `src/ov_msckf/msg/DebugState.msg`

```
std_msgs/Header header
float64 pos_x          # 位置x (m)
float64 pos_y          # 位置y (m)
float64 pos_z          # 位置z (m)
float64 vel_x          # 速度x (m/s)
float64 vel_y          # 速度y (m/s)
float64 vel_z          # 速度z (m/s)
float64 bias_gyro_x    # 陀螺仪偏置x (rad/s)
float64 bias_gyro_y    # 陀螺仪偏置y (rad/s)
float64 bias_gyro_z    # 陀螺仪偏置z (rad/s)
float64 bias_accel_x   # 加速度计偏置x (m/s^2)
float64 bias_accel_y   # 加速度计偏置y (m/s^2)
float64 bias_accel_z   # 加速度计偏置z (m/s^2)
float64 quat_x         # 姿态四元数x
float64 quat_y         # 姿态四元数y
float64 quat_z         # 姿态四元数z
float64 quat_w         # 姿态四元数w
```

**监控指标**:
- 位置、速度：轨迹平滑性
- 偏置：IMU偏置收敛情况，应在合理范围内稳定
- 四元数：姿态连续性，应满足 `x²+y²+z²+w²=1`

### DebugResiduals.msg

**文件**: `src/ov_msckf/msg/DebugResiduals.msg`

```
std_msgs/Header header
float64 res_visual     # 视觉重投影残差均值(像素)
float64 res_imu        # IMU预积分残差均值
```

**监控指标**:
- `res_visual`: 应<2像素，过大表示特征匹配错误或外点过多
- `res_imu`: 应接近0，过大表示IMU测量异常或状态估计偏差

---

## 数据录制脚本

### 录制基础数据

```bash
# 录制输入传感器数据
ros2 bag record /imu0 /cam0/image_raw /cam1/image_raw -o sensor_data
```

### 录制位姿输出

```bash
# 录制位姿估计和真值
ros2 bag record /ov_msckf/poseimu /ov_msckf/odomimu /ov_msckf/pathimu /ov_msckf/posegt /ov_msckf/pathgt -o pose_output
```

### 录制调试数据

```bash
# 录制调试监控数据
ros2 bag record /ov_msckf/debug/features /ov_msckf/debug/state /ov_msckf/debug/residuals -o debug_data
```

### 录制完整数据

```bash
# 录制所有open-vins相关topic
ros2 bag record /imu0 /cam0/image_raw /cam1/image_raw /ov_msckf/* /tf /tf_static -o full_data
```

---

## 常用Topic使用场景

| 使用场景 | 订阅Topic | 说明 |
|---------|----------|------|
| **基础定位** | `/ov_msckf/poseimu` 或 `/ov_msckf/odomimu` | 获取当前位姿估计，用于导航、控制 |
| **轨迹可视化** | `/ov_msckf/pathimu` | 在RViz中显示完整轨迹 |
| **精度评估** | `/ov_msckf/poseimu`, `/ov_msckf/posegt` | 对比估计位姿和真值，计算RMSE |
| **特征点分析** | `/ov_msckf/points_msckf`, `/ov_msckf/points_slam` | 分析特征点分布、跟踪质量 |
| **调试诊断** | `/ov_msckf/trackhist`, `/ov_msckf/debug/*` | 查看特征跟踪历史，监控系统健康度 |
| **回环检测** | `/ov_msckf/loop_*` 系列topic | 回环检测、位姿图优化 |
| **坐标系可视化** | `/tf`, `/tf_static` | 在RViz中显示坐标系关系 |

---

## Topic命名空间说明

所有open-vins发布的topic都在 `/ov_msckf/` 命名空间下：

- **无前缀**: 输入传感器数据（`/imu0`, `/cam0/image_raw`）
- **`/ov_msckf/` 前缀**: 系统输出数据
  - 位姿估计: `poseimu`, `odomimu`, `pathimu`
  - 真值: `posegt`, `pathgt`
  - 特征点: `points_*`
  - 回环: `loop_*`
  - 调试: `debug/*`

---

**文档生成时间**: 2026-04-29
**Open-VINS版本**: 基于当前main分支
**ROS版本**: ROS2 (Humble/Foxy)
