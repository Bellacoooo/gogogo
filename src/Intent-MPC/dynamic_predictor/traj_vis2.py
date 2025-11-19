#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from visualization_msgs.msg import MarkerArray, Marker

# 清除之前的设置，修复中文显示
plt.rcParams.update(plt.rcParamsDefault)
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = [
    'WenQuanYi Micro Hei', 'WenQuanYi Zen Hei',
    'Noto Sans CJK JP', 'DejaVu Sans'
]
plt.rcParams['axes.unicode_minus'] = False

# --------------------------
# 数据存储：按「障碍物ID」分类（关键修改）
# --------------------------
# 格式：{obs_id: {'t': [], 'x': [], 'y': [], 'z': []}}
real_trajs = {}  # 所有障碍物的真实轨迹
pred_trajs = {}  # 所有障碍物的预测轨迹
target_obs_id = 0  # 要可视化的目标障碍物ID（可根据实际调整）
max_data_len = 50  # 最多保留50个时间步（避免画面拥挤）

# --------------------------
# 订阅真实轨迹话题（修复解析逻辑）
# --------------------------
def real_traj_callback(msg):
    global real_trajs
    for marker in msg.markers:
        obs_id = marker.id  # 用marker.id唯一标识障碍物（关键）
        # 若该障碍物是首次出现，初始化数据结构
        if obs_id not in real_trajs:
            real_trajs[obs_id] = {'t': [], 'x': [], 'y': [], 'z': []}
        # 提取当前marker的轨迹点（每个点对应一个时间步）
        traj = real_trajs[obs_id]
        for idx, point in enumerate(marker.points):
            # 时间步 = 轨迹点在列表中的索引（确保和预测轨迹索引对齐）
            traj['t'].append(idx)
            traj['x'].append(point.x)
            traj['y'].append(point.y)
            traj['z'].append(point.z)
        # 截断过长数据（只保留最近max_data_len个点）
        if len(traj['t']) > max_data_len:
            for key in ['t', 'x', 'y', 'z']:
                traj[key] = traj[key][-max_data_len:]

# --------------------------
# 订阅预测轨迹话题（和真实轨迹解析逻辑一致）
# --------------------------
def pred_traj_callback(msg):
    global pred_trajs
    for marker in msg.markers:
        obs_id = marker.id  # 必须和真实轨迹的obs_id一致（关键）
        if obs_id not in pred_trajs:
            pred_trajs[obs_id] = {'t': [], 'x': [], 'y': [], 'z': []}
        traj = pred_trajs[obs_id]
        for idx, point in enumerate(marker.points):
            traj['t'].append(idx)  # 时间步=轨迹点索引（和真实轨迹对齐）
            traj['x'].append(point.x)
            traj['y'].append(point.y)
            traj['z'].append(point.z)
        # 截断过长数据
        if len(traj['t']) > max_data_len:
            for key in ['t', 'x', 'y', 'z']:
                traj[key] = traj[key][-max_data_len:]

# --------------------------
# 计算ADE和FDE（修复对齐逻辑）
# --------------------------
def compute_ade_fde(real_traj, pred_traj):
    # 先判断目标障碍物是否同时存在于两个轨迹中
    if target_obs_id not in real_trajs or target_obs_id not in pred_trajs:
        return 0.0, 0.0
    real = real_trajs[target_obs_id]
    pred = pred_trajs[target_obs_id]
    # 数据为空则返回0
    if len(real['x']) == 0 or len(pred['x']) == 0:
        return 0.0, 0.0
    # 按时间步索引对齐（取较短长度，避免索引越界）
    min_len = min(len(real['x']), len(pred['x']))
    # 截取对齐后的轨迹（确保每个索引对应同一时间步）
    real_x = np.array(real['x'][:min_len])
    real_y = np.array(real['y'][:min_len])
    real_z = np.array(real['z'][:min_len])
    pred_x = np.array(pred['x'][:min_len])
    pred_y = np.array(pred['y'][:min_len])
    pred_z = np.array(pred['z'][:min_len])
    # 计算欧氏距离
    distances = np.sqrt((real_x - pred_x)**2 + (real_y - pred_y)**2 + (real_z - pred_z)**2)
    ade = np.mean(distances) if min_len > 0 else 0.0
    fde = distances[-1] if min_len > 0 else 0.0
    return ade, fde

# --------------------------
# 实时更新轨迹图像（修复可视化逻辑）
# --------------------------
def update_plot(frame):
    plt.clf()
    # 计算ADE和FDE
    ade, fde = compute_ade_fde(real_trajs, pred_trajs)
    # 只可视化目标障碍物的轨迹（避免多障碍物混乱）
    real = real_trajs.get(target_obs_id, {'x': [], 'y': [], 'z': []})
    pred = pred_trajs.get(target_obs_id, {'x': [], 'y': [], 'z': []})
    
    # 绘制2D轨迹（x-y平面）
    plt.subplot(2, 1, 1)
    if len(real['x']) > 0:
        plt.plot(real['x'], real['y'], 'b-', label='实际轨迹', linewidth=2, markersize=4)
        plt.scatter(real['x'][-1], real['y'][-1], color='blue', s=50, label='实际位置（最新）')
    if len(pred['x']) > 0:
        plt.plot(pred['x'], pred['y'], 'r--', label='预测轨迹', linewidth=2, markersize=4)
        plt.scatter(pred['x'][-1], pred['y'][-1], color='red', s=50, label='预测位置（最新）')
    plt.xlabel('X坐标 (m)')
    plt.ylabel('Y坐标 (m)')
    plt.title(f'障碍物ID={target_obs_id} 轨迹对比（ADE: {ade:.2f}m, FDE: {fde:.2f}m）')
    plt.legend()
    plt.grid(True)
    # 保持坐标轴范围稳定（避免画面跳动）
    if len(real['x']) > 0 or len(pred['x']) > 0:
        all_x = real['x'] + pred['x']
        all_y = real['y'] + pred['y']
        plt.xlim(min(all_x)-0.5, max(all_x)+0.5)
        plt.ylim(min(all_y)-0.5, max(all_y)+0.5)
    
    # 绘制误差曲线
    plt.subplot(2, 1, 2)
    if len(real['x']) > 0 and len(pred['x']) > 0:
        min_len = min(len(real['x']), len(pred['x']))
        distances = np.sqrt(
            (np.array(real['x'][:min_len]) - np.array(pred['x'][:min_len]))**2 +
            (np.array(real['y'][:min_len]) - np.array(pred['y'][:min_len]))**2 +
            (np.array(real['z'][:min_len]) - np.array(pred['z'][:min_len]))**2
        )
        plt.plot(range(min_len), distances, 'g-', label='单步位移误差', linewidth=1.5)
        plt.axhline(y=ade, color='orange', linestyle='--', linewidth=2, label=f'ADE: {ade:.2f}m')
        plt.xlabel('时间步（索引）')
        plt.ylabel('误差 (m)')
        plt.legend()
        plt.grid(True)
        plt.ylim(0, max(distances)*1.2 if len(distances) > 0 else 1.0)  # 误差轴从0开始
    
    plt.tight_layout()

# --------------------------
# 主函数（修复订阅话题和初始化）
# --------------------------
if __name__ == '__main__':
    try:
        rospy.init_node('traj_visualizer', anonymous=True)
        
        # 订阅正确的话题（真实轨迹用onboard_detector的，预测轨迹用dynamic_predictor的）
        rospy.Subscriber('/onboard_detector/history_trajectories', MarkerArray, real_traj_callback)
        rospy.Subscriber('/dynamic_predictor/predict_trajectories', MarkerArray, pred_traj_callback)
        
        # 打印提示（帮助确认障碍物ID）
        rospy.loginfo("请通过 rostopic echo 确认目标障碍物ID，修改代码中的 target_obs_id")
        
        # 初始化可视化窗口
        fig = plt.figure(figsize=(10, 8))
        ani = FuncAnimation(fig, update_plot, interval=300)  # 300ms更新一次（更流畅）
        plt.show()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass