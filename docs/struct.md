# Open-VINS 结构文档

> 核心概念：坐标系、Topic、状态定义

---

## 坐标系

### 固定坐标系

| 坐标系 | 符号 | 说明 |
|--------|------|------|
| 全局坐标系 | `{G}` | z轴对齐重力 `g=[0,0,+9.81]`，原点=初始化时IMU位置 |

### 随机身运动坐标系

| 坐标系 | 符号 | 说明 |
|--------|------|------|
| IMU坐标系 | `{I}` | 核心状态估计坐标系，**充当机身坐标系** |
| 相机坐标系 | `{C_k}` | 第k个相机光学坐标系 |
| Clone坐标系 | `{I_i}` | 历史时刻IMU位姿，用于MSCKF滑动窗口 |
| 陀螺仪坐标系 | `{GYRO}` | 物理陀螺仪，默认与`{I}`对齐 |
| 加速度计坐标系 | `{ACC}` | 物理加速度计，默认与`{I}`对齐 |

> **关键**：没有独立机身坐标系，IMU坐标系直接充当。

### 命名规范

定义在 `src/docs/dev-coding-style.dox`：

| 类型 | 格式 | 示例 |
|------|------|------|
| 旋转矩阵 | `R_XtoY` | `R_GtoI` (G→I), `R_ItoC` (I→C) |
| 四元数 | `q_XtoY` | `q_GtoI`, `q_ItoC` |
| 位置向量 | `p_XinY` | `p_IinG` (IMU在G中), `p_IinC` (IMU在C中) |

### 核心状态

IMU状态向量 (15维)，定义在 `ov_core/src/types/IMU.h`：

| 索引 | 变量 | 含义 |
|------|------|------|
| `[0:3]` | `q_GtoI` | 四元数，G→I |
| `[4:6]` | `p_IinG` | IMU在G中位置 |
| `[7:9]` | `v_IinG` | IMU在G中速度 |
| `[10:12]` | `bg` | 陀螺仪偏置 |
| `[13:15]` | `ba` | 加速度计偏置 |

### 变换链

```
全局坐标系 G ──R_GtoI──→ IMU坐标系 I ──R_ItoC──→ 相机坐标系 C
```

特征点投影：`p_FinC = R_ItoC * R_GtoI * (p_FinG - p_IinG) + p_IinC`

### 外参来源

| 外参 | 来源 | 配置key | 在线标定 |
|------|------|---------|---------|
| IMU-CAM `(q_ItoC, p_IinC)` | Kalibr YAML | `T_imu_cam` | `calib_cam_extrinsics` |
| GYRO-IMU | Kalibr IMU YAML | `R_IMUtoGYRO` | `calib_imu_intrinsics` |
| ACC-IMU | Kalibr IMU YAML | `R_IMUtoACC` | `calib_imu_intrinsics` |

**注意**：Kalibr的`T_imu_cam`实际存储`T_CtoI`（相机→IMU），代码内部转为`(q_ItoC, p_IinC)`。

---

## ROS2 Topic

### 输入

| Topic | 类型 | 说明 |
|-------|------|------|
| `/imu0` | `sensor_msgs/Imu` | IMU测量 (~200Hz) |
| `/cam0/image_raw` | `sensor_msgs/Image` | 相机0图像 |
| `/cam1/image_raw` | `sensor_msgs/Image` | 相机1图像 (双目) |

### 输出

| Topic | 类型 | 说明 | 坐标系表示 |
|-------|------|------|-----------|
| `/ov_msckf/poseimu` | `PoseWithCovarianceStamped` | IMU位姿估计 | `(q_GtoI, p_IinG)` 在全局坐标系{G}中 |
| `/ov_msckf/odomimu` | `Odometry` | IMU里程计 (含速度) | `(q_GtoI, p_IinG, v_IinG)` 在全局坐标系{G}中 |
| `/ov_msckf/pathimu` | `Path` | 轨迹历史 | poseimu的历史轨迹，在{G}中 |
| `/ov_msckf/posegt` | `PoseStamped` | 真值位姿 | `(q_GtoI_gt, p_IinG_gt)` 在全局坐标系{G}中 |
| `/ov_msckf/points_msckf` | `PointCloud2` | MSCKF特征点 | `p_FinG` 特征点在全局坐标系{G}中的位置 |
| `/ov_msckf/points_slam` | `PointCloud2` | SLAM特征点 | `p_FinG` 特征点在全局坐标系{G}中的位置 |
| `/ov_msckf/trackhist` | `Image` | 特征跟踪可视化 | 图像坐标系{C}，无3D坐标 |

### Debug

| Topic | 类型 | 说明 |
|-------|------|------|
| `/ov_msckf/debug/features` | `DebugFeatures` | 特征统计 (提取/跟踪/三角化数) |
| `/ov_msckf/debug/state` | `DebugState` | 完整IMU状态 (16维) |
| `/ov_msckf/debug/residuals` | `DebugResiduals` | 优化残差 |

### TF树

```
global ──→ imu ──→ cam0
              ├──→ cam1
              └──→ ...
```

---

## 关键文件

| 功能 | 文件 |
|------|------|
| 状态定义 | `ov_msckf/src/state/State.h` |
| IMU类型 | `ov_core/src/types/IMU.h` |
| 位姿类型 | `ov_core/src/types/PoseJPL.h` |
| 四元数 | `ov_core/src/types/JPLQuat.h` |
| 外参加载 | `ov_msckf/src/core/VioManagerOptions.h` |
| 静态初始化 | `ov_init/src/static/StaticInitializer.cpp` |
| ROS可视化 | `ov_msckf/src/ros/ROS2Visualizer.cpp` |

---

## 四元数约定

JPL四元数（非Hamilton），存储顺序 `[x,y,z,w]`，左乘误差状态：`q_true = dq * q_hat`

参考：Trawny & Roumeliotis (2005)
