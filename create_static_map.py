#!/usr/bin/env python3
"""
创建静态地图 PCD 文件的工具脚本
用法: python3 create_static_map.py output.pcd
"""

import sys
import numpy as np

def create_static_map_pcd(output_file, obstacles=None):
    """
    创建静态地图 PCD 文件
    
    Args:
        output_file: 输出的 PCD 文件路径
        obstacles: 障碍物列表，每个障碍物是一个字典，包含：
            - 'type': 'box' 或 'cylinder' 或 'wall'
            - 'center': [x, y, z] 中心点
            - 'size': [width, length, height] 尺寸（对于 box）
            - 'radius': 半径（对于 cylinder）
            - 'height': 高度（对于 cylinder）
            - 'points': 点列表（对于 wall，直接指定点）
    """
    if obstacles is None:
        # 默认示例：创建一个简单的房间布局
        obstacles = [
            # 示例：四面墙
            {'type': 'wall', 'points': [
                # 前墙 (y = 5)
                [[x, 5.0, z] for x in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
                # 后墙 (y = -5)
                [[x, -5.0, z] for x in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
                # 左墙 (x = -5)
                [[-5.0, y, z] for y in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
                # 右墙 (x = 5)
                [[5.0, y, z] for y in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
            ]},
            # 示例：几个障碍物盒子
            {'type': 'box', 'center': [2.0, 2.0, 0.0], 'size': [1.0, 1.0, 1.5]},
            {'type': 'box', 'center': [-2.0, -2.0, 0.0], 'size': [1.5, 1.5, 2.0]},
            {'type': 'box', 'center': [0.0, 0.0, 0.0], 'size': [0.8, 0.8, 1.2]},
        ]
    
    all_points = []
    
    for obs in obstacles:
        if obs['type'] == 'box':
            center = np.array(obs['center'])
            size = np.array(obs['size'])
            # 生成盒子的点云（表面点）
            half_size = size / 2.0
            resolution = 0.1  # 点云密度
            
            # 6个面的点
            for axis in range(3):
                for sign in [-1, 1]:
                    # 创建该面的点
                    if axis == 0:  # x 面
                        x_coords = [center[0] + sign * half_size[0]]
                        y_coords = np.arange(center[1] - half_size[1], center[1] + half_size[1] + resolution, resolution)
                        z_coords = np.arange(center[2] - half_size[2], center[2] + half_size[2] + resolution, resolution)
                    elif axis == 1:  # y 面
                        x_coords = np.arange(center[0] - half_size[0], center[0] + half_size[0] + resolution, resolution)
                        y_coords = [center[1] + sign * half_size[1]]
                        z_coords = np.arange(center[2] - half_size[2], center[2] + half_size[2] + resolution, resolution)
                    else:  # z 面
                        x_coords = np.arange(center[0] - half_size[0], center[0] + half_size[0] + resolution, resolution)
                        y_coords = np.arange(center[1] - half_size[1], center[1] + half_size[1] + resolution, resolution)
                        z_coords = [center[2] + sign * half_size[2]]
                    
                    for x in x_coords:
                        for y in y_coords:
                            for z in z_coords:
                                all_points.append([x, y, z])
        
        elif obs['type'] == 'cylinder':
            center = np.array(obs['center'])
            radius = obs['radius']
            height = obs['height']
            resolution = 0.1
            
            # 圆柱体的点（侧面和顶面、底面）
            for z in np.arange(center[2], center[2] + height + resolution, resolution):
                for angle in np.arange(0, 2 * np.pi, resolution / radius):
                    x = center[0] + radius * np.cos(angle)
                    y = center[1] + radius * np.sin(angle)
                    all_points.append([x, y, z])
        
        elif obs['type'] == 'wall':
            # 直接使用提供的点列表
            for point_list in obs['points']:
                if isinstance(point_list[0], list):
                    # 嵌套列表
                    for point in point_list:
                        all_points.append(point)
                else:
                    # 单个点
                    all_points.append(point_list)
    
    # 去重（可选，但会慢一些）
    # all_points = list(set(tuple(p) for p in all_points))
    
    # 写入 PCD 文件
    with open(output_file, 'w') as f:
        # PCD 文件头
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write(f"WIDTH {len(all_points)}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {len(all_points)}\n")
        f.write("DATA ascii\n")
        
        # 写入点数据
        for point in all_points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")
    
    print(f"✓ 成功创建静态地图: {output_file}")
    print(f"  - 总点数: {len(all_points)}")
    print(f"  - 障碍物数量: {len(obstacles)}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 create_static_map.py <output.pcd>")
        print("\n示例:")
        print("  python3 create_static_map.py my_map.pcd")
        sys.exit(1)
    
    output_file = sys.argv[1]
    
    # 你可以在这里自定义障碍物
    custom_obstacles = [
        # 示例：创建一个 10x10 米的房间，高度 2.5 米
        {'type': 'wall', 'points': [
            # 前墙 (y = 5)
            [[x, 5.0, z] for x in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
            # 后墙 (y = -5)
            [[x, -5.0, z] for x in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
            # 左墙 (x = -5)
            [[-5.0, y, z] for y in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
            # 右墙 (x = 5)
            [[5.0, y, z] for y in np.arange(-5, 5, 0.1) for z in np.arange(0, 2.5, 0.1)],
        ]},
        # 添加一些障碍物
        {'type': 'box', 'center': [2.0, 2.0, 0.0], 'size': [1.0, 1.0, 1.5]},
        {'type': 'box', 'center': [-2.0, -2.0, 0.0], 'size': [1.5, 1.5, 2.0]},
        {'type': 'box', 'center': [0.0, 0.0, 0.0], 'size': [0.8, 0.8, 1.2]},
    ]
    
    create_static_map_pcd(output_file, custom_obstacles)

