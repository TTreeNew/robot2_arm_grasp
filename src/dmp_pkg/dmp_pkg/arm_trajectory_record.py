#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import tf2_ros
import pandas as pd
import numpy as np
from geometry_msgs.msg import TransformStamped

# from ament_index_python.packages import get_package_share_directory
import os

class ArmTrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('arm_trajectory_recorder')

        # 目标坐标系
        self.source_frame = 'base_link'
        self.target_frame = 'camera_link'

        # TF buffer & listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 存储数据
        self.positions = []

        # 定时器：10Hz 记录
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                now
            )

            # 提取位置 (x, y, z)
            pos = trans.transform.translation
            self.positions.append([pos.x, pos.y, pos.z])
            self.get_logger().info(f'Recorded point: {pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}')

        except Exception as e:
            self.get_logger().warn(f'Failed to get transform: {e}')

    def save_csv(self, filename="demo_trajectory_for_discrete_dmp.csv"):
        if len(self.positions) > 0:
            # 找到功能包路径
            # pkg_path = get_package_share_directory("dmp_pkg")
            # file_path = os.path.join(pkg_path, "config", filename)
            file_path = os.path.join(
            "/home/tree/robot2_arm_grasp/src/dmp_pkg/config",
            filename
        )
            # 转成 numpy array 再转置 (shape: 3 × N)
            data = np.array(self.positions).T
            df = pd.DataFrame(data)
            df.to_csv(file_path, header=False, index=False)
            self.get_logger().info(f'Saved trajectory to {file_path}, shape={data.shape}')
        else:
            self.get_logger().warn("No data recorded, not saving.")


def main(args=None):
    rclpy.init(args=args)
    node = ArmTrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received")
    finally:
        # 程序退出时保存 CSV
        node.save_csv("demo_trajectory_for_discrete_dmp.csv")
        node.destroy_node()
        # shutdown 只调用一次
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == '__main__':
    main()
