#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from visualization_msgs.msg import MarkerArray, Marker
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np

# 清除之前的设置
plt.rcParams.update(plt.rcParamsDefault)

# 强制设置中文字体为首选
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = [
    'WenQuanYi Micro Hei',      # 文泉驿微米黑
    'WenQuanYi Zen Hei',        # 文泉驿正黑
    'Noto Sans CJK JP',         # 思源黑体日本变体（也能显示中文）
    'DejaVu Sans',              # 回退英文字体
    'Bitstream Vera Sans',
    'Computer Modern Sans Serif'
]
plt.rcParams['axes.unicode_minus'] = False




# 存储轨迹数据：实际轨迹(蓝色)、预测轨迹(红色)
real_traj = {'t': [], 'x': [], 'y': [], 'z': []}  # 实际轨迹
pred_traj = {'t': [], 'x': [], 'y': [], 'z': []}  # 预测轨迹
time_step = 0  # 时间步计数器

# --------------------------
# 订阅实际轨迹话题 (例如：/history_traj)
# --------------------------
def real_traj_callback(msg):
    global real_traj, time_step
    # 清空历史数据（避免累积过多）
    if len(real_traj['t']) > 100:
        real_traj = {'t': [], 'x': [], 'y': [], 'z': []}
    # 提取Marker中的轨迹点（假设是LINE_STRIP类型，points字段存储坐标）
    for point in msg.markers[0].points:  # 取第一个障碍物的轨迹
        real_traj['t'].append(time_step)
        real_traj['x'].append(point.x)
        real_traj['y'].append(point.y)
        real_traj['z'].append(point.z)
    time_step += 1

# --------------------------
# 订阅预测轨迹话题 (例如：/pred_traj)
# --------------------------
def pred_traj_callback(msg):
    global pred_traj
    # 清空历史数据
    if len(pred_traj['t']) > 100:
        pred_traj = {'t': [], 'x': [], 'y': [], 'z': []}
    # 提取预测轨迹点
    for point in msg.markers[0].points:  # 取第一个障碍物的预测轨迹
        pred_traj['t'].append(len(pred_traj['t']))  # 预测轨迹时间步从0开始
        pred_traj['x'].append(point.x)
        pred_traj['y'].append(point.y)
        pred_traj['z'].append(point.z)

# --------------------------
# 计算ADE和FDE
# --------------------------
def compute_ade_fde(real, pred):
    if len(real['x']) == 0 or len(pred['x']) == 0:
        return 0.0, 0.0  # 数据不足时返回0
    # 对齐时间步（取较短的长度）
    min_len = min(len(real['x']), len(pred['x']))
    real_x = np.array(real['x'][:min_len])
    real_y = np.array(real['y'][:min_len])
    real_z = np.array(real['z'][:min_len])
    pred_x = np.array(pred['x'][:min_len])
    pred_y = np.array(pred['y'][:min_len])
    pred_z = np.array(pred['z'][:min_len])
    # 计算每个时间步的欧氏距离
    distances = np.sqrt((real_x - pred_x)**2 + (real_y - pred_y)** 2 + (real_z - pred_z)**2)
    ade = np.mean(distances)  # 平均位移误差
    fde = distances[-1] if min_len > 0 else 0.0  # 最终位移误差
    return ade, fde

# --------------------------
# 实时更新轨迹图像
# --------------------------
def update_plot(frame):
    plt.clf()  # 清空当前图像
    # 计算ADE和FDE
    ade, fde = compute_ade_fde(real_traj, pred_traj)
    # 绘制2D轨迹（x-y平面）
    plt.subplot(2, 1, 1)
    plt.plot(real_traj['x'], real_traj['y'], 'b-', label='实际轨迹', linewidth=2)
    plt.plot(pred_traj['x'], pred_traj['y'], 'r--', label='预测轨迹', linewidth=2)
    plt.xlabel('X坐标 (m)')
    plt.ylabel('Y坐标 (m)')
    plt.title(f'轨迹对比 (ADE: {ade:.2f}m, FDE: {fde:.2f}m)')
    plt.legend()
    plt.grid(True)
    # 绘制误差曲线
    plt.subplot(2, 1, 2)
    if len(real_traj['x']) > 0 and len(pred_traj['x']) > 0:
        min_len = min(len(real_traj['x']), len(pred_traj['x']))
        distances = np.sqrt(
            (np.array(real_traj['x'][:min_len]) - np.array(pred_traj['x'][:min_len]))**2 +
            (np.array(real_traj['y'][:min_len]) - np.array(pred_traj['y'][:min_len]))** 2 +
            (np.array(real_traj['z'][:min_len]) - np.array(pred_traj['z'][:min_len]))**2
        )
        plt.plot(range(min_len), distances, 'g-', label='位移误差')
        plt.axhline(y=ade, color='orange', linestyle='--', label=f'ADE: {ade:.2f}')
        plt.xlabel('时间步')
        plt.ylabel('误差 (m)')
        plt.legend()
        plt.grid(True)
    plt.tight_layout()  # 调整布局

# --------------------------
# 主函数：初始化ROS节点并订阅话题
# --------------------------
if __name__ == '__main__':
    try:
        # 初始化ROS节点（节点名：traj_visualizer）
        rospy.init_node('traj_visualizer', anonymous=True)
        # 订阅实际轨迹话题（请替换为你的实际轨迹话题名，如/history_traj）
        rospy.Subscriber('/dynamic_predictor/history_trajectories', MarkerArray, real_traj_callback)
        # 订阅预测轨迹话题（请替换为你的预测轨迹话题名，如/pred_traj）
        rospy.Subscriber('/dynamic_predictor/predict_trajectories', MarkerArray, pred_traj_callback)
        # 创建实时更新的图像
        fig = plt.figure(figsize=(10, 8))
        ani = FuncAnimation(fig, update_plot, interval=500)  # 每500ms更新一次
        plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass