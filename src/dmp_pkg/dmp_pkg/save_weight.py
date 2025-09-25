# -*- coding: utf-8 -*-
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys
import os
# 添加模块路径
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from dmp_pkg.dmp_discrete import dmp_discrete   # 这里假设你已经有 dmp_discrete.py

# ==========================
# 1. 读取示教轨迹
# ==========================
csv_file_path = "/home/tree/robot2_arm_grasp/src/dmp_pkg/config/demo_trajectory_for_discrete_dmp.csv"  # 你的csv文件路径
df = pd.read_csv(csv_file_path, header=None)
reference_trajectory = np.array(df)

# reference_trajectory 形状: (维度, 轨迹点数)
data_dim = reference_trajectory.shape[0]
data_len = reference_trajectory.shape[1]

print(f"示教轨迹维度 = {data_dim}, 轨迹长度 = {data_len}")

# ==========================
# 2. DMP 学习
# ==========================
dmp = dmp_discrete(n_dmps=data_dim, n_bfs=1000, dt=1.0/data_len)
dmp.learning(y_demo = reference_trajectory)
weight_file_path = "/home/tree/robot2_arm_grasp/src/dmp_pkg/config/up_to_down_trajectory.npy"
dmp.save_weights(weight_file_path)
dmp.load_weights(weight_file_path)
initial_pose = reference_trajectory[:,0].copy()
goal_pose = reference_trajectory[:,-1].copy()
# ==========================
# 3. DMP 生成轨迹
# ==========================
reproduced_trajectory, _, _ = dmp.reproduce(initial=initial_pose, goal=goal_pose)




print(f"DMP生成轨迹形状: {reproduced_trajectory.shape}")

# ==========================
# 4. 画图对比
# ==========================

# --- 3D 轨迹图 ---
# plt.rcParams['font.sans-serif'] = ['SimHei']  # 使用黑体
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
fig, axs = plt.subplots(data_dim, 1, figsize=(8, 6))
for i in range(data_dim):
    axs[i].plot(reference_trajectory[i, :], 'g', label='demo')
    axs[i].plot(reproduced_trajectory[:, i], 'r--', label='DMP')
    axs[i].legend()
    axs[i].set_ylabel(f"Dim {i+1}")
plt.xlabel("Time step")
plt.suptitle("dim_comparison")

plt.show()