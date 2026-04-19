#!/usr/bin/env python3
"""
OpenVINS调试数据记录脚本
订阅自定义debug topics并保存成CSV文件，用于PlotJuggler离线分析

使用方法:
    source init_env.sh
    python3 record_data.py

输出文件:
    logs/<timestamp>/vslam_debug.csv

PlotJuggler加载CSV:
    plotjuggler -> File -> Load Data -> 选择 vslam_debug.csv
"""

import rclpy
from rclpy.node import Node
from ov_msckf.msg import DebugFeatures, DebugState, DebugResiduals
import csv
import os
from datetime import datetime


class VSLAMDebugRecorder(Node):
    def __init__(self):
        super().__init__('vslam_debug_recorder')

        # 输出目录
        self.timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_dir = os.path.join(os.getcwd(), 'logs', self.timestamp)
        os.makedirs(self.log_dir, exist_ok=True)

        # CSV
        self.csv_path = os.path.join(self.log_dir, 'vslam_debug.csv')
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp',
            'num_extracted', 'num_tracked', 'num_3d', 'num_active',
            'pos_x', 'pos_y', 'pos_z',
            'vel_x', 'vel_y', 'vel_z',
            'bias_gyro_x', 'bias_gyro_y', 'bias_gyro_z',
            'bias_accel_x', 'bias_accel_y', 'bias_accel_z',
            'quat_x', 'quat_y', 'quat_z', 'quat_w',
            'res_visual', 'res_imu',
        ])

        # 缓存
        self.features = None
        self.state = None
        self.residuals = None

        # 订阅
        self.create_subscription(DebugFeatures, '/ov_msckf/debug/features', self.cb_features, 10)
        self.create_subscription(DebugState, '/ov_msckf/debug/state', self.cb_state, 10)
        self.create_subscription(DebugResiduals, '/ov_msckf/debug/residuals', self.cb_residuals, 10)

        self.get_logger().info(f'Recording to: {self.csv_path}')

    def cb_features(self, msg):
        self.features = msg

    def cb_state(self, msg):
        self.state = msg
        self._write_row()

    def cb_residuals(self, msg):
        self.residuals = msg

    def _write_row(self):
        t = self.get_clock().now()
        timestamp = t.seconds_nanoseconds()[0] + t.seconds_nanoseconds()[1] * 1e-9

        f = self.features
        s = self.state
        r = self.residuals

        row = [f'{timestamp:.6f}']
        row += [f.num_extracted, f.num_tracked, f.num_3d, f.num_active] if f else [0, 0, 0, 0]
        row += [f'{s.pos_x:.6f}', f'{s.pos_y:.6f}', f'{s.pos_z:.6f}',
                f'{s.vel_x:.6f}', f'{s.vel_y:.6f}', f'{s.vel_z:.6f}',
                f'{s.bias_gyro_x:.6f}', f'{s.bias_gyro_y:.6f}', f'{s.bias_gyro_z:.6f}',
                f'{s.bias_accel_x:.6f}', f'{s.bias_accel_y:.6f}', f'{s.bias_accel_z:.6f}',
                f'{s.quat_x:.6f}', f'{s.quat_y:.6f}', f'{s.quat_z:.6f}', f'{s.quat_w:.6f}'] if s else ['0.0'] * 16
        row += [f'{r.res_visual:.6f}', f'{r.res_imu:.6f}'] if r else ['0.0', '0.0']

        self.csv_writer.writerow(row)
        self.csv_file.flush()

        # 清空，等下一轮
        self.features = None
        self.state = None
        self.residuals = None

    def close(self):
        self.csv_file.close()
        print(f'CSV saved: {self.csv_path}')


def main():
    rclpy.init()
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
