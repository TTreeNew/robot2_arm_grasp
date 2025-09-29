# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dmp_pkg.dmp_discrete import dmp_discrete   # 这里假设你已经有 dmp_discrete.py

import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import ArmposeToTrajectory  # 自定义接口
from geometry_msgs.msg import Point
# import sys
# import os
# # 添加模块路径
# current_dir = os.path.dirname(os.path.abspath(__file__))
# parent_dir = os.path.dirname(current_dir)
# if parent_dir not in sys.path:
#     sys.path.insert(0, parent_dir)

class TrajectoryGenPub(Node):

    def __init__(self):
        super().__init__('trajectory_gen_pub')
        # self.current_state = self.create_client(/* srv_type */, '/* srv_name */')
        self.weight_file_path = "/home/tree/robot2_arm_grasp/src/dmp_pkg/config/up_to_down_trajectory.npy" #权重文件路径
        self.csv_file = "/home/tree/robot2_arm_grasp/src/dmp_pkg/config/demo_trajectory_for_discrete_dmp.csv"  # 你的csv文件路径
        self.df = pd.read_csv(self.csv_file, header=None)
        self.reference_trajectory = np.array(self.df).copy()  #读取参考轨迹
        self.data_dim = 1
        self.data_len = 1
        self.srv = self.create_service(ArmposeToTrajectory, 'armpose_to_trajectory', self.generate_trajectory_callback)        
        # self.plot_dmp_trajectory(self.reference_trajectory,self.reproduced_trajectory)
   

    def generate_trajectory_callback(self, request, response):
        current_pose = request.current_state
        self.get_logger().info(f"Received current_state: x={current_pose.x}, y={current_pose.y}, z={current_pose.z}")

        self.reproduced_trajectory = self.dmp_trajectory_gen(initial_pose = current_pose)
        
        # 转换 numpy -> Point[]
        response.trajectory = [
            Point(x=float(pt[0]), y=float(pt[1]), z=float(pt[2]))
            for pt in self.reproduced_trajectory
        ]

        self.get_logger().info(f"Sending trajectory with {len(response.trajectory)} points")
        return response
    
    def dmp_trajectory_gen(self,initial_pose = None):
        # ==========================
        # 1. 读取示教轨迹
        # ==========================

        if initial_pose is None:
            initial_pose = self.reference_trajectory[:,0].copy()
        else:
            # 将 Point 转换成 numpy 数组
            initial_pose = np.array([initial_pose.x, initial_pose.y, initial_pose.z])
        goal_pose = self.reference_trajectory[:,-1].copy()

        # reference_trajectory 形状: (维度, 轨迹点数)
        self.data_dim = self.reference_trajectory.shape[0]
        self.data_len = self.reference_trajectory.shape[1]

        print(f"示教轨迹维度 = {self.data_dim}, 轨迹长度 = {self.data_len}")

        # ==========================
        # 2. DMP 学习
        # ==========================
        dmp = dmp_discrete(n_dmps=self.data_dim, n_bfs=1000, dt=1.0/self.data_len)

        dmp.load_weights(self.weight_file_path)
        # ==========================
        # 3. DMP 生成轨迹
        # ==========================

        reproduced_trajectory, _, _ = dmp.reproduce(initial=initial_pose, goal=goal_pose)
        return reproduced_trajectory

    # print(f"DMP生成轨迹形状: {reproduced_trajectory.shape}")

    # ==========================
    # 4. 画图对比
    # ==========================

    # --- 3D 轨迹图 ---
    def plot_dmp_trajectory(self,reference_trajectory,reproduced_trajectory):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(reference_trajectory[0, :], reference_trajectory[1, :], reference_trajectory[2, :],
                'g', label="reference_trajectory")
        ax.plot(reproduced_trajectory[:, 0], reproduced_trajectory[:, 1], reproduced_trajectory[:, 2],
                'r--', label="reproduced_trajectory")
        ax.legend()
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        plt.title("reference_trajectory vs reproduced_trajectory")

        # --- 各维度对比 ---
        fig, axs = plt.subplots(self.data_dim, 1, figsize=(8, 6))
        for i in range(self.data_dim):
            axs[i].plot(reference_trajectory[i, :], 'g', label='demo')
            axs[i].plot(reproduced_trajectory[:, i], 'r--', label='DMP')
            axs[i].legend()
            axs[i].set_ylabel(f"Dim {i+1}")
        plt.xlabel("Time step")
        plt.suptitle("dim_comparison")

        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGenPub()
    # while rclpy.ok() and not node.called:
    #     rclpy.spin_once(node)  # 只处理一次事件循环
    # rclpy.shutdown()
    # self.plot_dmp_trajectory(self.reference_trajectory,self.reproduced_trajectory)
    rclpy.spin_once(node)  # 只处理一次事件循环
    rclpy.shutdown()

if __name__ == '__main__':
    main()
