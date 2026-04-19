# VSLAM 调试数据可视化

## 方案概述

推荐方案：**ROS Debug Topics + CSV记录 + PlotJuggler 可视化**，覆盖实时监控和离线分析两个场景。

所有命令在 `/home/jerett/OpenProject/LidarSlam/open-vins` 目录下运行，先 `source init_env.sh`。

---

## 一、数据来源

SLAM 系统通过 ROS topics 发布调试数据，包含三组信息：

| Topic | 含义 | 字段说明 |
|-------|------|------|
| `/ov_msckf/debug/features` | 特征状态 | `num_extracted` 提取的特征点数, `num_tracked` 成功跟踪的特征点数, `num_3d` 已三角化的3D特征数, `num_active` 当前活跃特征数 |
| `/ov_msckf/debug/state` | IMU 状态 | `pos_x/y/z` 位置, `vel_x/y/z` 速度, `bias_gyro_x/y/z` 陀螺仪零偏, `bias_accel_x/y/z` 加速度计零偏, `quat_x/y/z/w` 姿态四元数 |
| `/ov_msckf/debug/residuals` | 优化残差 | `res_visual` 视觉重投影残差均值, `res_imu` IMU 预积分残差均值 |

---

## 二、怎么运行

### 实时监控

需要开 3 个终端，每个终端先 `source init_env.sh`：

**终端1：启动 SLAM**
```bash
source init_env.sh
ov_run
```

**终端2：启动 PlotJuggler**
```bash
source init_env.sh
plotjuggler
# 操作:
#   1. Streaming -> Start: ROS2 Topic Subscriber
#   2. 勾选 /ov_msckf/debug/features, /ov_msckf/debug/state, /ov_msckf/debug/residuals
#   3. 从左侧列表拖拽字段到右侧图表
#   4. 保存布局: File -> Save Layout，下次直接加载
```

**终端3：播放 bag**
```bash
source init_env.sh
ov_play
```

### 离线分析

需要开 3 个终端，按顺序启动，每个终端先 `source init_env.sh`：

**终端1：播放原始数据 bag**
```bash
source init_env.sh
ov_play
```

**终端2：运行 SLAM**
```bash
source init_env.sh
ov_run
```

**终端3：记录调试数据**
```bash
source init_env.sh
python3 record_data.py
# 跑完后 Ctrl+C 停止，数据保存为 CSV
```

数据记录完成后，用 PlotJuggler 加载 CSV 离线分析：
```bash
source init_env.sh
plotjuggler  # File -> Load Data -> 选择生成的 CSV 文件
```

---

## 三、关键指标含义

| 指标 | 含义 | 正常范围 | 异常表现 | 可能原因 |
|-----|------|---------|---------|---------|
| `num_tracked` | 跟踪到的特征数 | >100 | 突降至 <50 | 跟踪丢失、运动模糊 |
| `num_3d` | 已三角化的3D特征数 | >80 | 持续 <50 | 三角化失败、基线太小 |
| `res_visual` | 视觉残差均值 | <2.0 | >5.0 | 外点过多、初始化错误 |
| `res_imu` | IMU残差均值 | <1.0 | >3.0 | IMU参数错误、bias未收敛 |
| `bias_gyro/accel` | IMU零偏 | 平缓变化 | 突变 | 初始化失败、外力干扰 |
| `vel_x/y/z` | 速度估计 | 合理范围 | 异常大 | 尺度漂移、状态发散 |

---

## 四、PlotJuggler 安装

```bash
sudo apt install ros-humble-plotjuggler-ros
sudo snap remove plotjuggler  # 如有 snap 版需删除
```
