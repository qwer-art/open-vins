import os
import csv
import json
import numpy as np
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, Image, PointCloud2
from tf2_msgs.msg import TFMessage
import os.path as osp


def timestamp_to_frametime(timestamp):
    """将时间戳转换为frame_time格式

    frame_time格式:
    - 前12位: 秒部分，不足补0
    - 后12位: 微秒部分，不足补0
    - 微秒按20Hz帧率(50ms=50000us)四舍五入

    Args:
        timestamp: float, 如 1777476533.213250

    Returns:
        str: 24位frame_time字符串
    """
    sec = int(timestamp)
    us = round((timestamp - sec) * 1e6)

    # 按50ms(50000us)四舍五入
    frame_us = round(us / 50000) * 50000

    # 处理进位: 如果微秒部分进位到1秒
    if frame_us >= 1000000:
        sec += frame_us // 1000000
        frame_us = frame_us % 1000000

    # 前12位秒，下划线分隔，后12位微秒
    return f"{sec:012d}_{frame_us:012d}"


def parse_all_topics(bag_path, save_path):
    """解析open-vins bag文件中的所有话题，输出JSON文件

    输出目录结构:
    1. imu/        : 每个 frame_time 一个 JSON 文件 (IMU数据)
    2. odomimu/    : 每个 frame_time 一个 JSON 文件 (里程计+IMU状态)
    3. poseimu/    : 每个 frame_time 一个 JSON 文件 (位姿+协方差)
    4. cam0/       : 每帧一个 PNG 图片
    5. cam1/       : 每帧一个 PNG 图片
    6. points_msckf/ : 每个 frame_time 一个 JSON 文件 (MSCKF特征点)
    7. tf/         : 每个 frame_time 一个 JSON 文件
    8. tf_static/  : 每个 frame_time 一个 JSON 文件

    Args:
        bag_path: bag文件目录路径
        save_path: 输出目录路径
    """
    import cv2

    # 创建输出目录
    imu_dir = os.path.join(save_path, "imu")
    odomimu_dir = os.path.join(save_path, "odomimu")
    poseimu_dir = os.path.join(save_path, "poseimu")
    cam0_dir = os.path.join(save_path, "cam0")
    cam1_dir = os.path.join(save_path, "cam1")
    points_dir = os.path.join(save_path, "points_msckf")
    tf_dir = os.path.join(save_path, "tf")
    tf_static_dir = os.path.join(save_path, "tf_static")

    for d in [imu_dir, odomimu_dir, poseimu_dir, cam0_dir, cam1_dir,
              points_dir, tf_dir, tf_static_dir]:
        os.makedirs(d, exist_ok=True)

    # 打开bag
    storage_opts = StorageOptions(uri=bag_path, storage_id="sqlite3")
    conv_opts = ConverterOptions("", "")
    reader = SequentialReader()
    reader.open(storage_opts, conv_opts)

    # 统计
    count_imu = 0
    count_odomimu = 0
    count_poseimu = 0
    count_cam0 = 0
    count_cam1 = 0
    count_points = 0
    count_tf = 0
    count_tf_static = 0

    msg_count = 0

    while reader.has_next():
        topic, data, t = reader.read_next()

        if topic == "/imu0":
            msg = deserialize_message(data, Imu)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            imu_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "linear_acceleration": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z
                },
                "angular_velocity": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z
                },
                "orientation": {
                    "x": msg.orientation.x,
                    "y": msg.orientation.y,
                    "z": msg.orientation.z,
                    "w": msg.orientation.w
                }
            }

            json_file = os.path.join(imu_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(imu_item, f, indent=2)
            count_imu += 1

        elif topic == "/ov_msckf/odomimu":
            msg = deserialize_message(data, Odometry)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            odomimu_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "pose": {
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z
                    },
                    "orientation": {
                        "x": msg.pose.pose.orientation.x,
                        "y": msg.pose.pose.orientation.y,
                        "z": msg.pose.pose.orientation.z,
                        "w": msg.pose.pose.orientation.w
                    },
                    "covariance": list(msg.pose.covariance)
                },
                "twist": {
                    "linear": {
                        "x": msg.twist.twist.linear.x,
                        "y": msg.twist.twist.linear.y,
                        "z": msg.twist.twist.linear.z
                    },
                    "angular": {
                        "x": msg.twist.twist.angular.x,
                        "y": msg.twist.twist.angular.y,
                        "z": msg.twist.twist.angular.z
                    },
                    "covariance": list(msg.twist.covariance)
                }
            }

            json_file = os.path.join(odomimu_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(odomimu_item, f, indent=2)
            count_odomimu += 1

        elif topic == "/ov_msckf/poseimu":
            msg = deserialize_message(data, PoseWithCovarianceStamped)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            poseimu_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "pose": {
                    "position": {
                        "x": msg.pose.pose.position.x,
                        "y": msg.pose.pose.position.y,
                        "z": msg.pose.pose.position.z
                    },
                    "orientation": {
                        "x": msg.pose.pose.orientation.x,
                        "y": msg.pose.pose.orientation.y,
                        "z": msg.pose.pose.orientation.z,
                        "w": msg.pose.pose.orientation.w
                    },
                    "covariance": list(msg.pose.covariance)
                }
            }

            json_file = os.path.join(poseimu_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(poseimu_item, f, indent=2)
            count_poseimu += 1

        elif topic == "/cam0/image_raw":
            msg = deserialize_message(data, Image)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            # 将Image消息转为numpy数组并保存
            img = image_msg_to_cv2(msg)
            if img is not None:
                img_file = os.path.join(cam0_dir, f"{frame_time}.png")
                cv2.imwrite(img_file, img)
            count_cam0 += 1

        elif topic == "/cam1/image_raw":
            msg = deserialize_message(data, Image)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            img = image_msg_to_cv2(msg)
            if img is not None:
                img_file = os.path.join(cam1_dir, f"{frame_time}.png")
                cv2.imwrite(img_file, img)
            count_cam1 += 1

        elif topic == "/ov_msckf/points_msckf":
            msg = deserialize_message(data, PointCloud2)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            points = pointcloud2_to_list(msg)
            points_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "points": points
            }

            json_file = os.path.join(points_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(points_item, f, indent=2)
            count_points += 1

        elif topic == "/tf":
            msg = deserialize_message(data, TFMessage)
            timestamp = t * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            transforms = []
            for transform in msg.transforms:
                transforms.append({
                    "child_frame_id": transform.child_frame_id,
                    "header": {
                        "frame_id": transform.header.frame_id,
                        "timestamp": transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    },
                    "transform": {
                        "translation": {
                            "x": transform.transform.translation.x,
                            "y": transform.transform.translation.y,
                            "z": transform.transform.translation.z
                        },
                        "rotation": {
                            "x": transform.transform.rotation.x,
                            "y": transform.transform.rotation.y,
                            "z": transform.transform.rotation.z,
                            "w": transform.transform.rotation.w
                        }
                    }
                })

            tf_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "transforms": transforms
            }

            json_file = os.path.join(tf_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(tf_item, f, indent=2)
            count_tf += 1

        elif topic == "/tf_static":
            msg = deserialize_message(data, TFMessage)
            timestamp = t * 1e-9
            frame_time = timestamp_to_frametime(timestamp)

            transforms = []
            for transform in msg.transforms:
                transforms.append({
                    "child_frame_id": transform.child_frame_id,
                    "header": {
                        "frame_id": transform.header.frame_id,
                        "timestamp": transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                    },
                    "transform": {
                        "translation": {
                            "x": transform.transform.translation.x,
                            "y": transform.transform.translation.y,
                            "z": transform.transform.translation.z
                        },
                        "rotation": {
                            "x": transform.transform.rotation.x,
                            "y": transform.transform.rotation.y,
                            "z": transform.transform.rotation.z,
                            "w": transform.transform.rotation.w
                        }
                    }
                })

            tf_static_item = {
                "frame_time": frame_time,
                "timestamp": timestamp,
                "transforms": transforms
            }

            json_file = os.path.join(tf_static_dir, f"{frame_time}.json")
            with open(json_file, "w") as f:
                json.dump(tf_static_item, f, indent=2)
            count_tf_static += 1

        msg_count += 1
        if msg_count % 5000 == 0:
            print(f"  已处理 {msg_count} 条消息...")

    print(f"\n完成! 共处理 {msg_count} 条消息")
    print(f"  imu/: {count_imu} 个文件")
    print(f"  odomimu/: {count_odomimu} 个文件")
    print(f"  poseimu/: {count_poseimu} 个文件")
    print(f"  cam0/: {count_cam0} 个文件")
    print(f"  cam1/: {count_cam1} 个文件")
    print(f"  points_msckf/: {count_points} 个文件")
    print(f"  tf/: {count_tf} 个文件")
    print(f"  tf_static/: {count_tf_static} 个文件")


def image_msg_to_cv2(msg):
    """将sensor_msgs/Image转为OpenCV numpy数组

    Args:
        msg: sensor_msgs/msg/Image

    Returns:
        numpy数组或None
    """
    import cv2

    height = msg.height
    width = msg.width
    encoding = msg.encoding
    is_bigendian = msg.is_bigendian
    step = msg.step
    data = bytes(msg.data)

    if encoding == "rgb8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == "bgr8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
    elif encoding == "mono8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
    elif encoding == "mono16":
        if is_bigendian:
            img = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
        else:
            img = np.frombuffer(data, dtype='<u2').reshape(height, width)
        img = (img / 256).astype(np.uint8)  # 16bit转8bit
    else:
        print(f"  警告: 不支持的图像编码格式 {encoding}")
        return None

    return img


def pointcloud2_to_list(msg):
    """将sensor_msgs/PointCloud2转为点列表

    Args:
        msg: sensor_msgs/msg/PointCloud2

    Returns:
        list of dict: [{x, y, z}, ...]
    """
    import struct

    points = []
    # 查找x, y, z字段的偏移量
    field_map = {}
    for field in msg.fields:
        field_map[field.name] = (field.offset, field.datatype)

    if "x" not in field_map or "y" not in field_map or "z" not in field_map:
        return points

    x_offset = field_map["x"][0]
    y_offset = field_map["y"][0]
    z_offset = field_map["z"][0]

    # datatype 7 = FLOAT32
    point_step = msg.point_step
    data = bytes(msg.data)
    n_points = msg.width * msg.height

    for i in range(n_points):
        offset = i * point_step
        x = struct.unpack_from('f', data, offset + x_offset)[0]
        y = struct.unpack_from('f', data, offset + y_offset)[0]
        z = struct.unpack_from('f', data, offset + z_offset)[0]
        if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
            points.append({"x": float(x), "y": float(y), "z": float(z)})

    return points


def export_to_csv(save_path):
    """从JSON文件读取数据，导出为CSV文件，便于PlotJuggler可视化

    输出:
        analyse_data/analyse.csv: 包含所有关键数据
            - 时间戳、frame_time
            - 位置 (p_x, p_y, p_z)
            - 姿态四元数 (q_x, q_y, q_z, q_w) 和欧拉角 (roll, pitch, yaw)
            - 速度 (v_x, v_y, v_z) 和模长 (v_norm)
            - 角速度 (w_x, w_y, w_z) 和模长 (w_norm)
            - 位置协方差对角线 (pos_cov_x, pos_cov_y, pos_cov_z)
            - 姿态协方差对角线 (ori_cov_x, ori_cov_y, ori_cov_z)
            - IMU数据 (acc_x/y/z, gyro_x/y/z)

    Args:
        save_path: JSON文件根目录路径
    """
    odomimu_dir = os.path.join(save_path, "odomimu")
    imu_dir = os.path.join(save_path, "imu")
    output_dir = os.path.join(save_path, "analyse_data")
    os.makedirs(output_dir, exist_ok=True)

    # 读取所有odomimu数据
    print("正在读取odomimu数据...")
    odomimu_data = {}
    for filename in sorted(os.listdir(odomimu_dir)):
        if filename.endswith(".json"):
            filepath = os.path.join(odomimu_dir, filename)
            with open(filepath, "r") as f:
                data = json.load(f)
                frame_time = data["frame_time"]
                odomimu_data[frame_time] = data

    print(f"  读取到 {len(odomimu_data)} 条odomimu数据")

    # 读取所有imu数据
    print("正在读取imu数据...")
    imu_data = {}
    if os.path.exists(imu_dir):
        for filename in sorted(os.listdir(imu_dir)):
            if filename.endswith(".json"):
                filepath = os.path.join(imu_dir, filename)
                with open(filepath, "r") as f:
                    data = json.load(f)
                    frame_time = data["frame_time"]
                    imu_data[frame_time] = data

    print(f"  读取到 {len(imu_data)} 条imu数据")

    # 写入CSV
    output_csv = os.path.join(output_dir, "analyse.csv")

    header = [
        "timestamp", "frame_time",
        "p_x", "p_y", "p_z",
        "q_x", "q_y", "q_z", "q_w",
        "roll", "pitch", "yaw",
        "v_x", "v_y", "v_z", "v_norm",
        "w_x", "w_y", "w_z", "w_norm",
        "pos_cov_x", "pos_cov_y", "pos_cov_z",
        "ori_cov_x", "ori_cov_y", "ori_cov_z",
        "acc_x", "acc_y", "acc_z",
        "gyro_x", "gyro_y", "gyro_z"
    ]

    print("\n正在同步数据并写入CSV...")
    count_sync = 0
    count_odom_only = 0

    def fmt(val):
        return f"{val:.12f}"

    with open(output_csv, "w", newline="\n") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        for frame_time in sorted(odomimu_data.keys()):
            odom = odomimu_data[frame_time]

            timestamp = odom["timestamp"]
            p_x = odom["pose"]["position"]["x"]
            p_y = odom["pose"]["position"]["y"]
            p_z = odom["pose"]["position"]["z"]
            q_x = odom["pose"]["orientation"]["x"]
            q_y = odom["pose"]["orientation"]["y"]
            q_z = odom["pose"]["orientation"]["z"]
            q_w = odom["pose"]["orientation"]["w"]
            v_x = odom["twist"]["linear"]["x"]
            v_y = odom["twist"]["linear"]["y"]
            v_z = odom["twist"]["linear"]["z"]
            w_x = odom["twist"]["angular"]["x"]
            w_y = odom["twist"]["angular"]["y"]
            w_z = odom["twist"]["angular"]["z"]

            # 四元数转欧拉角
            roll = np.arctan2(2.0 * (q_w * q_x + q_y * q_z), 1.0 - 2.0 * (q_x * q_x + q_y * q_y))
            pitch = np.arcsin(np.clip(2.0 * (q_w * q_y - q_z * q_x), -1.0, 1.0))
            yaw = np.arctan2(2.0 * (q_w * q_z + q_x * q_y), 1.0 - 2.0 * (q_y * q_y + q_z * q_z))

            v_norm = np.sqrt(v_x**2 + v_y**2 + v_z**2)
            w_norm = np.sqrt(w_x**2 + w_y**2 + w_z**2)

            # 协方差对角线
            pose_cov = odom["pose"]["covariance"]
            pos_cov_x = pose_cov[0]
            pos_cov_y = pose_cov[7]
            pos_cov_z = pose_cov[14]
            ori_cov_x = pose_cov[21]
            ori_cov_y = pose_cov[28]
            ori_cov_z = pose_cov[35]

            # 尝试从imu数据中获取IMU原始数据
            acc_x, acc_y, acc_z = 0.0, 0.0, 0.0
            gyro_x, gyro_y, gyro_z = 0.0, 0.0, 0.0

            if frame_time in imu_data:
                imu = imu_data[frame_time]
                acc_x = imu["linear_acceleration"]["x"]
                acc_y = imu["linear_acceleration"]["y"]
                acc_z = imu["linear_acceleration"]["z"]
                gyro_x = imu["angular_velocity"]["x"]
                gyro_y = imu["angular_velocity"]["y"]
                gyro_z = imu["angular_velocity"]["z"]
                count_sync += 1
            else:
                count_odom_only += 1

            writer.writerow([
                f"{timestamp:.9f}",
                frame_time,
                fmt(p_x), fmt(p_y), fmt(p_z),
                fmt(q_x), fmt(q_y), fmt(q_z), fmt(q_w),
                fmt(roll), fmt(pitch), fmt(yaw),
                fmt(v_x), fmt(v_y), fmt(v_z), fmt(v_norm),
                fmt(w_x), fmt(w_y), fmt(w_z), fmt(w_norm),
                fmt(pos_cov_x), fmt(pos_cov_y), fmt(pos_cov_z),
                fmt(ori_cov_x), fmt(ori_cov_y), fmt(ori_cov_z),
                fmt(acc_x), fmt(acc_y), fmt(acc_z),
                fmt(gyro_x), fmt(gyro_y), fmt(gyro_z)
            ])

    print(f"  成功同步: {count_sync} 帧")
    print(f"  仅odomimu: {count_odom_only} 帧")
    print(f"\n完成! CSV文件已保存到: {output_csv}")


if __name__ == "__main__":
    bag_path = "/home/jerett/OpenProject/LidarSlam/open-vins/src/scripts/data/open_vins_20260429_232853"
    save_path = osp.join(bag_path, "asset_data")

    # 第一步：解析bag文件
    print("=" * 60)
    print("Step 1: 解析bag文件")
    print("=" * 60)
    parse_all_topics(bag_path, save_path)

    # 第二步：导出CSV
    print("\n" + "=" * 60)
    print("Step 2: 导出CSV")
    print("=" * 60)
    export_to_csv(save_path)
