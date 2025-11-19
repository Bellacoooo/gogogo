#!/usr/bin/env python3
import rospy
import math
from visualization_msgs.msg import MarkerArray, Marker
from collections import defaultdict
import time

class SimpleEvaluator:
    def __init__(self):
        # 存储数据 {障碍物ID: {意图类型: 轨迹点}}
        self.pred_trajectories = defaultdict(dict)
        self.history_trajectories = defaultdict(list)
        
        # 存储时间戳用于对齐
        self.last_pred_time = 0
        self.last_history_time = 0
        
        # 订阅话题
        self.pred_sub = rospy.Subscriber('/dynamic_predictor/predict_trajectories', 
                                        MarkerArray, self.pred_callback)
        self.history_sub = rospy.Subscriber('/dynamic_predictor/history_trajectories', 
                                           MarkerArray, self.history_callback)
        
        # 定时评估
        self.eval_timer = rospy.Timer(rospy.Duration(1.0), self.evaluate)
        
        print("简单轨迹评估器已启动，等待数据...")

    def pred_callback(self, msg):
        """预测轨迹回调"""
        self.last_pred_time = time.time()
        
        for marker in msg.markers:
            # 提取障碍物ID和意图类型
            obstacle_id = marker.id // 10  # 假设每10个ID是一个障碍物的不同意图
            intent_type = marker.id % 10
            
            if marker.type == Marker.LINE_STRIP:
                points = []
                for p in marker.points:
                    points.append([p.x, p.y, p.z])
                self.pred_trajectories[obstacle_id][intent_type] = points

    def history_callback(self, msg):
        """历史轨迹回调"""
        self.last_history_time = time.time()
        
        for marker in msg.markers:
            obstacle_id = marker.id
            
            if marker.type == Marker.LINE_STRIP:
                points = []
                for p in marker.points:
                    points.append([p.x, p.y, p.z])
                self.history_trajectories[obstacle_id] = points

    def calculate_distance(self, point1, point2):
        """计算两点间距离"""
        dx = point1[0] - point2[0]
        dy = point1[1] - point2[1]
        dz = point1[2] - point2[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def calculate_ade_fde(self, pred_points, actual_points):
        """计算ADE和FDE"""
        if len(pred_points) == 0 or len(actual_points) == 0:
            return float('inf'), float('inf')
        
        # 取相同长度的轨迹
        min_length = min(len(pred_points), len(actual_points))
        
        # 计算ADE
        total_error = 0
        for i in range(min_length):
            distance = self.calculate_distance(pred_points[i], actual_points[i])
            total_error += distance
        ade = total_error / min_length
        
        # 计算FDE
        fde = self.calculate_distance(pred_points[min_length-1], actual_points[min_length-1])
        
        return ade, fde

    def evaluate(self, event=None):
        """评估函数"""
        # 检查数据是否新鲜（2秒内的数据）
        current_time = time.time()
        if current_time - self.last_pred_time > 2.0 or current_time - self.last_history_time > 2.0:
            return
        
        print("\n" + "="*60)
        print("轨迹预测评估结果 (minADE/minFDE - 取最佳意图):")
        print("="*60)
        
        total_min_ade = 0
        total_min_fde = 0
        count = 0
        
        for obstacle_id in self.pred_trajectories:
            if obstacle_id in self.history_trajectories:
                actual_points = self.history_trajectories[obstacle_id]
                
                # 对每个意图计算ADE/FDE，取最好的
                min_ade = float('inf')
                min_fde = float('inf')
                best_intent = -1
                
                for intent_type, pred_points in self.pred_trajectories[obstacle_id].items():
                    ade, fde = self.calculate_ade_fde(pred_points, actual_points)
                    
                    if ade < min_ade:
                        min_ade = ade
                        min_fde = fde
                        best_intent = intent_type
                
                if min_ade < float('inf'):
                    print(f"障碍物 {obstacle_id} (最佳意图:{best_intent}):")
                    print(f"  minADE: {min_ade:.3f}米")
                    print(f"  minFDE: {min_fde:.3f}米")
                    print(f"  轨迹长度: {len(actual_points)}个点")
                    print("-" * 40)
                    
                    total_min_ade += min_ade
                    total_min_fde += min_fde
                    count += 1
        
        if count > 0:
            avg_ade = total_min_ade / count
            avg_fde = total_min_fde / count
            print(f"\n总体平均 (基于{count}个障碍物):")
            print(f"  平均minADE: {avg_ade:.3f}米")
            print(f"  平均minFDE: {avg_fde:.3f}米")
        else:
            print("没有找到匹配的轨迹数据进行评估")
        
        print("="*60)

def main():
    rospy.init_node('simple_trajectory_evaluator')
    evaluator = SimpleEvaluator()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("\n评估器已停止")

if __name__ == '__main__':
    main()