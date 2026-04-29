# OpenVINS 运行说明

## 0. 环境初始化

```bash
source /home/jerett/OpenProject/LidarSlam/open-vins/init_env.sh
```

该脚本会自动：
- 加载ROS2环境
- 检查编译状态（必要时自动编译）
- 加载OpenVINS工作空间
- 设置常用命令别名

## 1. 运行VIO系统

```bash
# 方式一：使用别名（推荐）
ov_run

# 方式二：完整命令
ros2 launch ov_msckf subscribe.launch.py config:=tum_vi rviz_enable:=true
```

**参数说明：**
- `config:=tum_vi` - 使用TUM VI数据集配置（当前数据集）
- `rviz_enable:=true` - 启用RViz可视化

## 2. 播放Bag

### 2.1 使用别名（推荐）

`ov_play` 别名支持速率参数，可以灵活控制播放速度：

```bash
# 1倍速播放（默认）
ov_play 1

# 5倍速播放（快速测试）
ov_play 5

# 半速播放（慢速分析）
ov_play 0.5

# 2倍速播放
ov_play 2

# 10倍速播放（极速测试）
ov_play 10
```

### 2.2 使用完整命令

```bash
# 基础播放（1倍速）
ros2 bag play /home/jerett/OpenProject/LidarSlam/open-vins/src/Data/dataset-room1_512_16_ros2 --clock

# 5倍速播放
ros2 bag play /home/jerett/OpenProject/LidarSlam/open-vins/src/Data/dataset-room1_512_16_ros2 --clock --rate 5

# 其他常用参数
ros2 bag play /path/to/bag --clock --loop          # 循环播放
ros2 bag play /path/to/bag --clock --start-offset 10  # 从第10秒开始
ros2 bag play /path/to/bag --clock --duration 30   # 只播放30秒
```

### 2.3 播放时键盘控制

播放过程中可以使用键盘实时调整速率：

|按键 | 功能 |
|-----|------|
| **空格** | 暂停/继续 |
| **右箭头** | 播放下一条消息（暂停时） |
| **上箭头** | 增加速率 10% |
| **下箭头** | 减少速率 10% |

**示例操作流程**：
1. `ov_play 1` - 开始1倍速播放
2. 按 **上箭头** 5次 - 逐步增加到1.5倍速
3. 按 **空格** - 暂停查看当前状态
4. 按 **右箭头** - 单步播放下一条消息
5. 按 **空格** - 继续播放

### 2.4 速率选择建议

| 场景 | 推荐速率 | 说明 |
|------|---------|------|
| **首次运行** | 1倍速 | 观察完整过程，检查初始化 |
| **快速测试** | 5倍速 | 验证算法是否正常工作 |
| **调试问题** | 0.5倍速 | 慢速观察特征跟踪、状态变化 |
| **性能测试** | 10倍速 | 测试系统处理能力 |
| **轨迹评估** | 1倍速 | 精确评估，避免时间同步问题 |

**注意**：
- 速率过高可能导致系统处理不过来，出现消息队列堆积
- 使用 `--clock` 参数确保时间同步（VIO系统依赖仿真时钟）
- 键盘控制适合微调速率，不适合大幅调整

## 3. ROS2可视化Pose

### 方式一：RViz（推荐）

启动时使用 `rviz_enable:=true`，RViz会自动显示：
- 轨迹路径 (`/ov_msckf/pathimu`)
- 特征点云 (`/ov_msckf/points_msckf`)
- 当前位姿 (`/ov_msckf/poseimu`)

### 方式二：PlotJuggler

```bash
ros2 run plotjuggler plotjuggler
```

**Streaming** → **Start: ROS2 Topic Subscriber** → 选择话题：

| 话题 | 内容 |
|------|------|
| `/ov_msckf/poseimu` | IMU位姿估计（PoseWithCovarianceStamped） |
| `/ov_msckf/odomimu` | IMU里程计（Odometry） |
| `/ov_msckf/pathimu` | 轨迹路径（Path） |

### 方式三：命令行查看

```bash
# 查看位姿输出
ros2 topic echo /ov_msckf/poseimu

# 查看话题频率
ros2 topic hz /ov_msckf/poseimu

# 查看所有话题
ros2 topic list
```

## 4. 常用话题列表

| 话题 | 类型 | 说明 |
|------|------|------|
| `/ov_msckf/poseimu` | PoseWithCovarianceStamped | 当前位姿估计 |
| `/ov_msckf/odomimu` | Odometry | 里程计 |
| `/ov_msckf/pathimu` | Path | 轨迹路径 |
| `/ov_msckf/points_msckf` | PointCloud2 | MSCKF特征点云 |
| `/ov_msckf/points_slam` | PointCloud2 | SLAM特征点云 |
| `/ov_msckf/trackhist` | Image | 特征跟踪可视化 |

## 5. 配置文件

配置文件路径：`src/config/euroc_mav/estimator_config.yaml`

**关键参数：**

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `init_dyn_use` | true | 动态初始化（移动平台推荐） |
| `use_stereo` | true | 双目跟踪 |
| `max_cameras` | 2 | 相机数量 |
| `num_pts` | 200 | 特征点数量 |

## 6. 保存轨迹

在配置文件中启用：

```yaml
save_total_state: true
filepath_est: "/tmp/ov_estimate.txt"
filepath_std: "/tmp/ov_estimate_std.txt"
filepath_gt: "/tmp/ov_groundtruth.txt"
```

使用 `ov_eval` 工具评估：

```bash
source /home/jerett/OpenProject/LidarSlam/open-vins/src/install/setup.bash
rosrun ov_eval error_singlerun /tmp/ov_groundtruth.txt /tmp/ov_estimate.txt
```

---

## 数据集信息

### TUM Visual-Inertial Dataset

**数据集主页**: https://vision.in.tum.de/data/datasets/visual-inertial-dataset

**当前使用数据集**:
- 名称: `dataset-room1_512_16`
- 下载地址: http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room1_512_16.bag
- 时长: 147秒
- 分辨率: 512×512
- 相机模型: equidistant (鱼眼)

**其他可用数据集**:

| 名称 | 时长(s) | 下载链接 |
|------|---------|----------|
| room1 | 147 | [rosbag](http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room1_512_16.bag) |
| room2 | 142 | [rosbag](http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room2_512_16.bag) |
| room3 | 136 | [rosbag](http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room3_512_16.bag) |
| room4 | 69 | [rosbag](http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room4_512_16.bag) |
| room5 | 132 | [rosbag](http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room5_512_16.bag) |
| room6 | 67 | [rosbag](http://vision.in.tum.de/tumvi/calibrated/512_16/dataset-room6_512_16.bag) |

### 本地数据集路径

| 数据集 | Bag路径 |
|--------|---------|
| TUM VI Room1 (ROS1格式) | `src/Data/dataset-room1_512_16.bag` |
| TUM VI Room1 (ROS2格式) | `src/Data/dataset-room1_512_16_ros2/` |

## 常见问题

### 初始化失败

如果日志显示 `failed static init`，需要启用动态初始化：
```yaml
init_dyn_use: true
```

### Ceres库冲突

如果编译时出现 `cublas` 链接错误，需要移除 `/usr/local` 下的CUDA版Ceres：
```bash
sudo mv /usr/local/lib/libceres.so* /tmp/
sudo mv /usr/local/lib/cmake/Ceres /tmp/
sudo ldconfig
```

### 话题不匹配

检查bag文件中的话题名称：
```bash
ros2 bag info /path/to/bag
```

检查OpenVINS订阅的话题：
```bash
ros2 node info /ov_msckf/run_subscribe_msckf
```

## 调试数据监控

### 实时监控（推荐）

使用PlotJuggler实时查看调试数据：

```bash
# 终端1: 启动VIO系统
ov_run

# 终端2: 启动PlotJuggler
plotjuggler
# Streaming -> Start: ROS2 Topic Subscriber
# 选择 /debug/features, /debug/state, /debug/residuals

# 终端3: 播放bag
ov_play
```

### 离线分析

记录调试数据到CSV，然后用PlotJuggler分析：

```bash
# 终端1: 启动VIO系统
ov_run

# 终端2: 启动调试数据记录器
python3 src/ov_msckf/scripts/vslam_debug_recorder.py

# 终端3: 播放bag
ov_play

# Ctrl+C停止后，用PlotJuggler加载CSV
plotjuggler
# File -> Load Data -> 选择 logs/<timestamp>/vslam_debug.csv
```

### 调试Topics说明

| Topic | 类型 | 内容 |
|-------|------|------|
| `/debug/features` | Float64MultiArray | [提取数, 跟踪数, 3D特征数, 活跃特征数] |
| `/debug/state` | Float64MultiArray | [p(3), v(3), bg(3), ba(3), q(4)] |
| `/debug/residuals` | Float64MultiArray | [视觉残差, IMU残差] |

### 关键指标解读

| 指标 | 正常范围 | 异常表现 | 可能原因 |
|-----|---------|---------|---------|
| `feat_tracked` | >100 | 突降至<50 | 跟踪丢失、运动模糊 |
| `feat_3d` | >80 | 持续<50 | 三角化失败、基线太小 |
| `res_visual` | <2.0 | >5.0 | 外点过多、初始化错误 |
| `res_imu` | <1.0 | >3.0 | IMU参数错误、bias未收敛 |
| `bg/ba` | 平缓变化 | 突变 | 初始化失败、外力干扰 |
| `velocity` | 合理范围 | 异常大 | 尺度漂移、状态发散 |

### CSV文件格式

记录的CSV文件包含以下列：

```
timestamp, feat_extracted, feat_tracked, feat_3d, feat_active,
px, py, pz, vx, vy, vz, bgx, bgy, bgz, bax, bay, baz, qx, qy, qz, qw,
res_visual, res_imu
```

### 使用rosbag记录（最简单）

```bash
# 记录所有debug topics
ros2 bag record /debug/features /debug/state /debug/residuals -o debug.bag

# 播放bag
ros2 bag play dataset.bag

# 然后用PlotJuggler播放debug.bag
plotjuggler
# Streaming -> Start: ROS2 Topic Subscriber from Bag
```
