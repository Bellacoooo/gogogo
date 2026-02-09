#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TTC^{-1} (Inverse Time to Collision) 实时计算与绘图节点
实时从ROS topics获取无人机与障碍物状态，计算并可视化逆TTC指标
"""

import rospy
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 使用非GUI后端，不显示窗口
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from dynamic_predictor.msg import PredictedObstacles
from onboard_detector.srv import GetDynamicObstacles
from geometry_msgs.msg import Point
import threading
from collections import defaultdict


class TTCInversePlotter:
    def __init__(self):
        rospy.init_node('ttc_inverse_plotter', anonymous=True)
        
        # ==================== 参数配置 ====================
        # 无人机位姿来源
        self.drone_odom_topic = rospy.get_param('~drone_odom_topic', '/CERLAB/quadcopter/odom')
        
        # 障碍物信息来源
        self.obstacle_topic = rospy.get_param('~obstacle_topic', '/dynamic_predictor/predicted_obstacles')
        self.use_service_for_size = rospy.get_param('~use_service_for_size', True)
        self.obstacle_service = rospy.get_param('~obstacle_service', '/fake_detector/get_dynamic_obstacles')
        
        # 障碍物ID列表（只跟踪这些ID）
        self.target_obstacle_ids = rospy.get_param('~obstacle_ids', [0, 1, 2])
        
        # TTC^{-1} 计算参数
        self.eps = float(rospy.get_param('~eps', 1e-3))  # 避免除零的小量
        self.r_safe_default = float(rospy.get_param('~r_safe_default', 0.5))  # 默认安全半径(m)
        
        # 绘图参数
        self.window_sec = float(rospy.get_param('~window_sec', 20.0))  # 显示最近20秒
        self.y_min = float(rospy.get_param('~y_min', -1.5))
        self.y_max = float(rospy.get_param('~y_max', 1.5))
        self.update_rate_hz = int(rospy.get_param('~update_rate_hz', 20))
        
        # 自动保存参数
        self.auto_save = bool(rospy.get_param('~auto_save', True))  # 是否自动保存
        self.save_path = str(rospy.get_param('~save_path', '/home/ff/Desktop/ttc_inverse_plot.png'))  # 保存路径
        self.save_interval_sec = float(rospy.get_param('~save_interval_sec', 2.0))  # 每N秒保存一次
        self.last_save_time = None
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("TTC^{-1} Plotter 启动配置:")
        rospy.loginfo("  无人机 Odom Topic: %s", self.drone_odom_topic)
        rospy.loginfo("  障碍物 Topic: %s", self.obstacle_topic)
        rospy.loginfo("  使用服务获取尺寸: %s", self.use_service_for_size)
        if self.use_service_for_size:
            rospy.loginfo("  障碍物服务: %s", self.obstacle_service)
        rospy.loginfo("  目标障碍物IDs: %s", self.target_obstacle_ids)
        rospy.loginfo("  ε (避免除零): %.2e", self.eps)
        rospy.loginfo("  默认安全半径: %.2f m", self.r_safe_default)
        rospy.loginfo("  时间窗口: %.1f s", self.window_sec)
        rospy.loginfo("  Y轴范围: [%.2f, %.2f]", self.y_min, self.y_max)
        rospy.loginfo("  自动保存: %s", "启用" if self.auto_save else "禁用")
        if self.auto_save:
            rospy.loginfo("  保存路径: %s", self.save_path)
            rospy.loginfo("  保存间隔: %.1f s", self.save_interval_sec)
        rospy.loginfo("=" * 60)
        
        # ==================== 数据存储 ====================
        # 无人机状态
        self.drone_pos = None
        self.drone_pos_lock = threading.Lock()
        
        # 障碍物状态：字典[id] -> {'pos': [x,y,z], 'vel': [vx,vy,vz], 'size': [sx,sy,sz]}
        self.obstacles = {}
        self.obstacles_lock = threading.Lock()
        
        # TTC^{-1} 历史数据：字典[id] -> {'time': list, 'ttc_inv': list, 'last_d': float, 'last_t': float}
        self.ttc_data = defaultdict(lambda: {
            'time': [],
            'ttc_inv': [],
            'last_d': None,
            'last_t': None
        })
        self.ttc_data_lock = threading.Lock()
        
        # 起始时间
        self.start_time = None
        
        # ==================== ROS 订阅与服务 ====================
        self.drone_sub = rospy.Subscriber(self.drone_odom_topic, Odometry, self.drone_callback, queue_size=10)
        self.obstacle_sub = rospy.Subscriber(self.obstacle_topic, PredictedObstacles, self.obstacle_callback, queue_size=10)
        
        if self.use_service_for_size:
            rospy.loginfo("等待障碍物服务: %s", self.obstacle_service)
            try:
                rospy.wait_for_service(self.obstacle_service, timeout=5.0)
                self.obstacle_srv = rospy.ServiceProxy(self.obstacle_service, GetDynamicObstacles)
                rospy.loginfo("障碍物服务已连接")
            except rospy.ROSException:
                rospy.logwarn("障碍物服务超时，将使用默认安全半径")
                self.use_service_for_size = False
        
        # ==================== Matplotlib 初始化 ====================
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.lines = {}
        self.colors = plt.cm.tab10(np.linspace(0, 1, 10))
        
        for i, obs_id in enumerate(self.target_obstacle_ids):
            line, = self.ax.plot([], [], label=f'Obstacle {obs_id+1}', 
                               color=self.colors[i % 10], linewidth=2)
            self.lines[obs_id] = line
        
        self.ax.set_xlabel('Time (s)', fontsize=14)
        self.ax.set_ylabel('Inverse Time to Collision (s$^{-1}$)', fontsize=14)
        self.ax.set_title('Real-time TTC$^{-1}$ for Dynamic Obstacles', fontsize=16, fontweight='bold')
        self.ax.legend(loc='upper right', fontsize=12)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(self.y_min, self.y_max)
        
        rospy.loginfo("Matplotlib 初始化完成")
        
    def drone_callback(self, msg):
        """接收无人机位姿"""
        with self.drone_pos_lock:
            self.drone_pos = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
        
        # 初始化起始时间
        if self.start_time is None:
            self.start_time = msg.header.stamp.to_sec()
    
    def obstacle_callback(self, msg):
        """接收障碍物信息（位置+速度）"""
        current_time = msg.header.stamp.to_sec()
        if self.start_time is None:
            self.start_time = current_time
        
        with self.obstacles_lock:
            for obs in msg.obstacles:
                if obs.id in self.target_obstacle_ids:
                    self.obstacles[obs.id] = {
                        'pos': np.array([
                            obs.current_position.x,
                            obs.current_position.y,
                            obs.current_position.z
                        ]),
                        'vel': np.array([
                            obs.current_velocity.x,
                            obs.current_velocity.y,
                            obs.current_velocity.z
                        ]),
                        'size': None,  # 稍后填充
                        'time': current_time
                    }
        
        # 如果启用服务获取尺寸
        if self.use_service_for_size:
            self.query_obstacle_sizes()
        
        # 计算 TTC^{-1}
        self.compute_ttc_inverse(current_time)
    
    def query_obstacle_sizes(self):
        """通过服务查询障碍物尺寸"""
        if not hasattr(self, 'obstacle_srv'):
            return
        
        try:
            # 使用无人机当前位置查询
            with self.drone_pos_lock:
                if self.drone_pos is None:
                    return
                req_pos = Point()
                req_pos.x, req_pos.y, req_pos.z = self.drone_pos
            
            resp = self.obstacle_srv(req_pos, 50.0)  # 查询50m范围内
            
            with self.obstacles_lock:
                for i, pos in enumerate(resp.position):
                    # 匹配障碍物（通过位置相似度）
                    obs_pos = np.array([pos.x, pos.y, pos.z])
                    for obs_id, obs_data in self.obstacles.items():
                        if obs_data['size'] is None:
                            dist = np.linalg.norm(obs_data['pos'] - obs_pos)
                            if dist < 0.5:  # 0.5m阈值认为是同一个障碍物
                                obs_data['size'] = np.array([
                                    resp.size[i].x,
                                    resp.size[i].y,
                                    resp.size[i].z
                                ])
                                break
        except Exception as e:
            rospy.logwarn_throttle(5.0, "查询障碍物尺寸失败: %s", str(e))
    
    def compute_ttc_inverse(self, current_time):
        """计算 TTC^{-1}"""
        with self.drone_pos_lock:
            if self.drone_pos is None:
                return
            drone_pos = self.drone_pos.copy()
        
        with self.obstacles_lock:
            obstacles_copy = {k: v.copy() for k, v in self.obstacles.items()}
        
        with self.ttc_data_lock:
            for obs_id, obs_data in obstacles_copy.items():
                if obs_id not in self.target_obstacle_ids:
                    continue
                
                # 计算距离 d
                obs_pos = obs_data['pos']
                
                # 计算安全半径
                if obs_data['size'] is not None:
                    # 使用障碍物的最大半径
                    r_safe = np.max(obs_data['size']) / 2.0
                else:
                    r_safe = self.r_safe_default
                
                # d = ||p_u - p_o|| - r_safe
                d_current = np.linalg.norm(drone_pos - obs_pos) - r_safe
                
                # 获取上一帧数据
                last_d = self.ttc_data[obs_id]['last_d']
                last_t = self.ttc_data[obs_id]['last_t']
                
                # 首帧跳过（无法计算导数）
                if last_d is None or last_t is None:
                    self.ttc_data[obs_id]['last_d'] = d_current
                    self.ttc_data[obs_id]['last_t'] = current_time
                    continue
                
                # 计算时间差
                dt = current_time - last_t
                if dt < 1e-6:  # 时间间隔太小，跳过
                    continue
                
                # 计算 dot_d = (d_k - d_{k-1}) / dt
                dot_d = (d_current - last_d) / dt
                
                # 计算 TTC^{-1} = dot_d / (d + eps)
                ttc_inv = dot_d / (d_current + self.eps)
                
                # 存储数据
                relative_time = current_time - self.start_time
                self.ttc_data[obs_id]['time'].append(relative_time)
                self.ttc_data[obs_id]['ttc_inv'].append(ttc_inv)
                self.ttc_data[obs_id]['last_d'] = d_current
                self.ttc_data[obs_id]['last_t'] = current_time
                
                # 滚动窗口：只保留最近 window_sec 的数据
                while (len(self.ttc_data[obs_id]['time']) > 0 and 
                       relative_time - self.ttc_data[obs_id]['time'][0] > self.window_sec):
                    self.ttc_data[obs_id]['time'].pop(0)
                    self.ttc_data[obs_id]['ttc_inv'].pop(0)
    
    def update_plot(self, frame):
        """更新绘图"""
        with self.ttc_data_lock:
            for obs_id, line in self.lines.items():
                if obs_id in self.ttc_data:
                    time_data = self.ttc_data[obs_id]['time']
                    ttc_inv_data = self.ttc_data[obs_id]['ttc_inv']
                    line.set_data(time_data, ttc_inv_data)
        
        # 动态调整X轴
        if self.start_time is not None:
            current_relative_time = rospy.Time.now().to_sec() - self.start_time
            if current_relative_time > self.window_sec:
                self.ax.set_xlim(current_relative_time - self.window_sec, current_relative_time)
            else:
                self.ax.set_xlim(0, max(self.window_sec, current_relative_time + 1))
        
        # 自动保存
        if self.auto_save:
            current_time = rospy.Time.now().to_sec()
            # 首次保存或达到保存间隔
            should_save = (self.last_save_time is None) or (current_time - self.last_save_time) >= self.save_interval_sec
            
            if should_save:
                try:
                    # 确保目录存在
                    import os
                    save_dir = os.path.dirname(self.save_path)
                    if save_dir and not os.path.exists(save_dir):
                        os.makedirs(save_dir)
                    
                    self.fig.savefig(self.save_path, dpi=300, bbox_inches='tight')
                    
                    if self.last_save_time is None:
                        rospy.loginfo("首次保存图像到: %s", self.save_path)
                        rospy.loginfo("图像大小: %.2f KB", os.path.getsize(self.save_path) / 1024.0)
                    
                    self.last_save_time = current_time
                except Exception as e:
                    rospy.logwarn_throttle(5.0, "保存图像失败: %s", str(e))
                    import traceback
                    rospy.logwarn_throttle(5.0, traceback.format_exc())
        
        return list(self.lines.values())
    
    def run(self):
        """启动绘图"""
        rospy.loginfo("开始实时绘图...")
        rospy.loginfo("提示: TTC^{-1} < 0 表示障碍物在接近（风险增加），越负风险越高")
        rospy.loginfo("      TTC^{-1} ≈ 0 表示障碍物距离稳定（相对安全）")
        rospy.loginfo("      TTC^{-1} > 0 表示障碍物在远离（风险降低）")
        
        if self.auto_save:
            rospy.loginfo("自动保存已启用，图像将每 %.1f 秒更新到: %s", self.save_interval_sec, self.save_path)
        
        plt.tight_layout()
        plt.ion()  # 开启交互模式
        
        # 立即保存初始图像（即使没有数据）
        if self.auto_save:
            try:
                import os
                save_dir = os.path.dirname(self.save_path)
                if save_dir and not os.path.exists(save_dir):
                    os.makedirs(save_dir)
                
                self.fig.savefig(self.save_path, dpi=300, bbox_inches='tight')
                rospy.loginfo("初始图像已保存到: %s", self.save_path)
                rospy.loginfo("图像大小: %.2f KB", os.path.getsize(self.save_path) / 1024.0)
                self.last_save_time = rospy.Time.now().to_sec()
            except Exception as e:
                rospy.logerr("初始保存失败: %s", str(e))
                import traceback
                rospy.logerr(traceback.format_exc())
        
        # 尝试显示窗口（如果没有图形界面，会自动跳过）
        try:
            plt.show(block=False)
            rospy.loginfo("Matplotlib 窗口已打开（如果有图形界面）")
        except Exception as e:
            rospy.loginfo("无图形界面，仅保存文件模式: %s", str(e))
        
        # 手动更新循环（不依赖FuncAnimation）
        rate = rospy.Rate(self.update_rate_hz)
        frame_count = 0
        while not rospy.is_shutdown():
            try:
                # 手动调用更新函数
                self.update_plot(frame_count)
                frame_count += 1
                
                # 刷新图形
                try:
                    plt.pause(0.001)
                except:
                    pass  # 如果没有图形界面，忽略错误
                
                rate.sleep()
            except Exception as e:
                rospy.logerr_throttle(5.0, "更新绘图时出错: %s", str(e))


if __name__ == '__main__':
    try:
        plotter = TTCInversePlotter()
        plotter.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("TTC^{-1} Plotter 错误: %s", str(e))
        import traceback
        traceback.print_exc()
