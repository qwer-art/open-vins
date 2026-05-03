# Open-VINS 结构文档

> 核心概念：坐标系、Topic、状态定义

---

## 坐标系

### 固定坐标系

| 坐标系 | 符号 | 说明 |
|--------|------|------|
| 全局坐标系 | `{G}` | z轴对齐重力 `g=[0,0,+9.81]`，原点=初始化时IMU位置。x轴=世界`[1,0,0]`投影到z轴垂直平面（Gram-Schmidt），y轴=`z×x`（右手定则） |

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

---

## 重力方向漂移问题

**问题**：全局坐标系{G}的z轴初始化时对齐重力，但随时间漂移。

**原因**：陀螺仪bias估计误差 + 噪声累积 + 视觉约束不足

**OpenVINS策略**：
- 初始化时估计重力方向（StaticInitializer.cpp:122-125）
- 运行时固定为`[0,0,9.81]`（Propagator.cpp:57）
- 不在线优化重力方向

**实际影响**：
- 短时间（<1min）：漂移小
- 长时间（>5min）：漂移明显，需后处理对齐

**解决方案**：评估时使用`posyaw`对齐（AlignTrajectory.h）

**为什么不像其他系统在线优化重力？**
- 精度提升有限（20-30%）
- 实现复杂度显著增加（状态15→18维）
- 数值稳定性问题
- 工程权衡：简洁性 > 边际精度提升

**代码位置**：
- 重力初始化：`ov_init/src/static/StaticInitializer.cpp:122-125`
- 重力传播：`ov_msckf/src/state/Propagator.cpp:57`
- 轨迹对齐：`ov_eval/src/alignment/AlignTrajectory.h`

---

## 室内数据集真值获取方式

**问题**：室内无人机/机器人数据集的真值如何获取？

**答案**：使用**运动捕捉系统（Motion Capture, MoCap）**

### 主流系统

| 系统 | 精度 | 频率 | 应用场景 |
|------|------|------|---------|
| **Vicon** | 亚毫米级 | 100-200Hz | EuRoC MAV, TUM VI |
| **OptiTrack** | 亚毫米级 | 100-240Hz | RPNG AR Table |
| **Qualisys** | 亚毫米级 | 100-200Hz | 学术研究 |

### 工作原理

```
室内空间安装多个红外摄像头 → 捕捉反光标记球 → 三角测量得到6DOF位姿
```

**关键步骤**：
1. 室内安装8-24个红外摄像头（覆盖整个空间）
2. 无人机/机器人贴上反光标记球
3. 摄像头同步捕捉标记球位置
4. 软件实时计算6DOF位姿（位置+姿态）
5. 通过时间戳与IMU/相机数据对齐

### 数据处理流程

```cpp
// 1. MoCap测量的是标记球的位姿，需要转换到IMU坐标系
// 2. 使用vicon2gt工具优化（EuRoC V1_01_easy案例）
// 参考：src/docs/gs-datasets.dox:25-28

// 输入：MoCap位姿 + IMU测量
// 输出：IMU轨迹真值（优化后）
vicon2gt::optimize(mocap_poses, imu_measurements);
```

### 为什么都有真值？

**研究需求**：
- 算法性能评估需要基准
- 误差量化分析
- 算法对比公平性

**成本考量**：
- MoCap设备昂贵（$50k-$500k）
- 室内空间有限（通常<10m×10m）
- 仅限实验室环境

### 局限性

| 局限 | 说明 |
|------|------|
| **空间限制** | 仅限室内小范围（Vicon房间约8m×8m） |
| **遮挡问题** | 标记球被遮挡时丢失真值 |
| **成本高昂** | 设备+安装+维护费用高 |
| **仅室内** | 无法用于室外/大范围场景 |

### 实际案例

**EuRoC MAV**（gs-datasets.dox:11-44）：
- Vicon系统，200Hz
- Vicon Room 1和2（约8m×8m）
- Machine Hall（工业场景，真值质量较差）

**TUM VI**（gs-datasets.dox:49-78）：
- MoCap系统，仅room系列有完整真值
- 室外数据无真值

**RPNG AR Table**（gs-datasets.dox:82-101）：
- OptiTrack，100Hz
- 使用vicon2gt优化

### 代码位置

| 功能 | 文件 |
|------|------|
| 数据集说明 | `src/docs/gs-datasets.dox` |
| 真值加载 | `ov_eval/src/utils/Loader.cpp` |
| 轨迹对齐 | `ov_eval/src/alignment/AlignTrajectory.h` |
| vicon2gt工具 | [GitHub链接](https://github.com/rpng/vicon2gt) |
