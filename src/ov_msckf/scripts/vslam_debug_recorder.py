#!/usr/bin/env python3
"""
OpenVINS调试数据记录脚本
订阅调试topics并保存成CSV文件，用于PlotJuggler离线分析

使用方法:
    ros2 run ov_msckf vslam_debug_recorder

输出文件:
    logs/<timestamp>/vslam_debug.csv - 所有数据合并到一个文件

PlotJuggler加载CSV:
    File -> Load Data -> 选择 vslam_debug.csv
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import csv
import os
from datetime import datetime


class VSLAMDebugRecorder(Node):
    def __init__(self):
        super().__init__('vslam_debug_recorder')

        # 创建带时间戳的输出目录
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(os.getcwd(), 'logs', self.timestamp)
        os.makedirs(self.log_dir, exist_ok=True)

        # CSV文件
        self.csv_path = os.path.join(self.log_dir, 'vslam_debug.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)

        # 写入header
        self.csv_writer.writerow([
            'timestamp',
            # 特征统计
            'feat_extracted', 'feat_tracked', 'feat_3d', 'feat_active',
            # 15维状态量
            'px', 'py', 'pz',
            'vx', 'vy', 'vz',
            'bgx', 'bgy', 'bgz',
            'bax', 'bay', 'baz',
            'qx', 'qy', 'qz', 'qw',
            # 残差
            'res_visual', 'res_imu'
        ])

        # 数据缓存
        self.current_data = {}

        # 订阅topics
        self.create_subscription(
            Float64MultiArray, '/debug/features',
            self.cb_features, 10
        )
        self.create_subscription(
            Float64MultiArray, '/debug/state',
            self.cb_state, 10
        )
        self.create_subscription(
            Float64MultiArray, '/debug/residuals',
            self.cb_residuals, 10
        )

        self.get_logger().info(f'VSLAM Debug Recorder started')
        self.get_logger().info(f'Output: {self.csv_path}')

    def cb_features(self, msg):
        """特征统计: [提取数, 跟踪数, 3D特征数, 活跃特征数]"""
        self.current_data['features'] = list(msg.data)

    def cb_state(self, msg):
        """15维状态量: [p(3), v(3), bg(3), ba(3), q(4)]"""
        self.current_data['state'] = list(msg.data)
        self._write_row()

    def cb_residuals(self, msg):
        """残差: [视觉残差, IMU残差]"""
        self.current_data['residuals'] = list(msg.data)

    def _write_row(self):
        """state作为触发器，写入一行数据"""
        # 获取当前时间戳
        now = self.get_clock().now()
        timestamp = now.seconds_nanoseconds()[0] + now.seconds_nanoseconds()[1] * 1e-9

        # 获取数据
        features = self.current_data.get('features', [0, 0, 0, 0])
        state = self.current_data.get('state', [0] * 16)
        residuals = self.current_data.get('residuals', [0, 0])

        # 构建行数据
        row = [f'{timestamp:.6f}']

        # 特征统计
        row.extend([f'{v:.0f}' for v in features[:4]])

        # 状态量
        row.extend([f'{v:.6f}' for v in state[:16]])

        # 残差
        row.extend([f'{v:.6f}' for v in residuals[:2]])

        # 写入并刷新
        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # 清空缓存
        self.current_data = {}

    def close(self):
        """关闭文件"""
        self.csv_file.close()
        print(f'CSV saved: {self.csv_path}')


def main(args=None):
    rclpy.init(args=args)
    node = VSLAMDebugRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
