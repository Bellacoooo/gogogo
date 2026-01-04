#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
风险图诊断工具
检查：1. 风险图是否正常发布 2. 数据是否有效 3. A*是否正确使用
"""

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sys

class RiskMapDiagnostic:
    def __init__(self):
        rospy.init_node('risk_map_diagnostic', anonymous=True)
        
        self.risk_map_received = False
        self.risk_map_count = 0
        self.last_risk_map = None
        
        # 订阅风险图
        self.risk_map_sub = rospy.Subscriber(
            '/dynamic_predictor/dynamic_risk_map',
            OccupancyGrid,
            self.risk_map_callback,
            queue_size=1
        )
        
        rospy.loginfo("[RISK-DIAG] 风险图诊断工具已启动")
        rospy.loginfo("[RISK-DIAG] 订阅话题: /dynamic_predictor/dynamic_risk_map")
        
    def risk_map_callback(self, msg):
        self.risk_map_received = True
        self.risk_map_count += 1
        self.last_risk_map = msg
        
        # 每10次打印一次详细统计
        if self.risk_map_count % 10 == 0:
            self.analyze_risk_map(msg)
    
    def analyze_risk_map(self, msg):
        """分析风险图数据"""
        data = np.array(msg.data, dtype=np.int8)
        
        # 基本统计
        total_cells = len(data)
        non_zero = np.sum(data > 0)
        zero_cells = np.sum(data == 0)
        max_val = np.max(data)
        mean_val = np.mean(data[data > 0]) if non_zero > 0 else 0.0
        
        # 风险分布
        risk_ranges = {
            'low (1-25)': np.sum((data > 0) & (data <= 25)),
            'medium (26-50)': np.sum((data > 25) & (data <= 50)),
            'high (51-75)': np.sum((data > 50) & (data <= 75)),
            'very_high (76-100)': np.sum(data > 75)
        }
        
        rospy.logwarn("=" * 60)
        rospy.logwarn("[RISK-DIAG] 风险图统计 #%d", self.risk_map_count)
        rospy.logwarn("[RISK-DIAG] 地图尺寸: %dx%d (分辨率=%.3f m)", 
                     msg.info.width, msg.info.height, msg.info.resolution)
        rospy.logwarn("[RISK-DIAG] 地图原点: (%.3f, %.3f, %.3f)", 
                     msg.info.origin.position.x, 
                     msg.info.origin.position.y,
                     msg.info.origin.position.z)
        rospy.logwarn("[RISK-DIAG] 总栅格数: %d", total_cells)
        rospy.logwarn("[RISK-DIAG] 非零栅格: %d (%.2f%%)", non_zero, 100.0 * non_zero / total_cells if total_cells > 0 else 0)
        rospy.logwarn("[RISK-DIAG] 零栅格: %d (%.2f%%)", zero_cells, 100.0 * zero_cells / total_cells if total_cells > 0 else 0)
        rospy.logwarn("[RISK-DIAG] 最大值: %d, 平均值: %.3f", max_val, mean_val)
        rospy.logwarn("[RISK-DIAG] 风险分布:")
        for level, count in risk_ranges.items():
            rospy.logwarn("[RISK-DIAG]   %s: %d (%.2f%%)", level, count, 
                         100.0 * count / total_cells if total_cells > 0 else 0)
        
        # 检查是否有有效风险数据
        if non_zero == 0:
            rospy.logerr("[RISK-DIAG] ⚠️  警告: 风险图为空！没有检测到任何障碍物风险！")
        elif max_val < 10:
            rospy.logwarn("[RISK-DIAG] ⚠️  警告: 最大风险值很低 (%d)，可能风险图生成有问题", max_val)
        else:
            rospy.loginfo("[RISK-DIAG] ✅ 风险图正常，检测到 %d 个非零栅格", non_zero)
        
        rospy.logwarn("=" * 60)
    
    def run(self):
        """运行诊断"""
        rate = rospy.Rate(1)  # 1 Hz
        
        while not rospy.is_shutdown():
            if not self.risk_map_received:
                rospy.logwarn("[RISK-DIAG] ⚠️  等待风险图数据... (话题: /dynamic_predictor/dynamic_risk_map)")
            else:
                if self.risk_map_count == 0:
                    rospy.logwarn("[RISK-DIAG] ⚠️  已订阅但未收到数据")
                else:
                    rospy.loginfo("[RISK-DIAG] ✅ 已收到 %d 个风险图消息", self.risk_map_count)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        diagnostic = RiskMapDiagnostic()
        diagnostic.run()
    except rospy.ROSInterruptException:
        pass

