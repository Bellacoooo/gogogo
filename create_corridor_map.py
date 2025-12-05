#!/usr/bin/env python3
"""
创建走廊场景的预建地图（PCD文件）
走廊位置：pose = 8.73735 -0.275929 0 0 -0 0
走廊尺寸：
- Wall_0: y = 2.5, size = 41.15 x 0.15 x 2.5
- Wall_2: y = -2.5, size = 41.15 x 0.15 x 2.5
- Wall_4: x = -20.5, size = 5.15 x 0.15 x 2.5 (旋转 -1.5708)
- Wall_6: x = 20.5, size = 5.15 x 0.15 x 2.5 (旋转 -1.5708)
"""

import numpy as np
import struct

def create_wall_points(center_x, center_y, center_z, size_x, size_y, size_z, resolution=0.1):
    """创建墙的点云"""
    points = []
    half_x = size_x / 2.0
    half_y = size_y / 2.0
    
    # 在墙的表面生成点
    x_range = np.arange(center_x - half_x, center_x + half_x, resolution)
    z_range = np.arange(center_z, center_z + size_z, resolution)
    
    # 前表面和后表面
    for x in x_range:
        for z in z_range:
            # 前表面 (y = center_y + half_y)
            points.append([x, center_y + half_y, z])
            # 后表面 (y = center_y - half_y)
            points.append([x, center_y - half_y, z])
    
    # 左表面和右表面
    y_range = np.arange(center_y - half_y, center_y + half_y, resolution)
    for y in y_range:
        for z in z_range:
            # 左表面 (x = center_x - half_x)
            points.append([center_x - half_x, y, z])
            # 右表面 (x = center_x + half_x)
            points.append([center_x + half_x, y, z])
    
    return np.array(points)

def write_pcd_file(filename, points):
    """写入PCD文件"""
    with open(filename, 'wb') as f:
        # PCD header
        header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
        f.write(header.encode('ascii'))
        
        # Write points
        for point in points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n".encode('ascii'))

def main():
    # 走廊模型的位置
    corridor_pose_x = 8.73735
    corridor_pose_y = -0.275929
    corridor_pose_z = 0.0
    
    all_points = []
    
    # Wall_0: y = 2.5 (相对于走廊模型)
    # 实际位置：y = corridor_pose_y + 2.5 = -0.275929 + 2.5 = 2.224071
    wall0_points = create_wall_points(
        corridor_pose_x, corridor_pose_y + 2.5, 1.25,
        41.15, 0.15, 2.5, resolution=0.1
    )
    all_points.append(wall0_points)
    
    # Wall_2: y = -2.5 (相对于走廊模型)
    # 实际位置：y = corridor_pose_y - 2.5 = -0.275929 - 2.5 = -2.775929
    wall2_points = create_wall_points(
        corridor_pose_x, corridor_pose_y - 2.5, 1.25,
        41.15, 0.15, 2.5, resolution=0.1
    )
    all_points.append(wall2_points)
    
    # Wall_4: x = -20.5 (相对于走廊模型，旋转-1.5708)
    # 实际位置：x = corridor_pose_x - 20.5 = 8.73735 - 20.5 = -11.76265
    # 旋转后：实际上是 y 方向的墙
    wall4_points = create_wall_points(
        corridor_pose_x - 20.5, corridor_pose_y, 1.25,
        0.15, 5.15, 2.5, resolution=0.1
    )
    all_points.append(wall4_points)
    
    # Wall_6: x = 20.5 (相对于走廊模型，旋转-1.5708)
    # 实际位置：x = corridor_pose_x + 20.5 = 8.73735 + 20.5 = 29.23735
    wall6_points = create_wall_points(
        corridor_pose_x + 20.5, corridor_pose_y, 1.25,
        0.15, 5.15, 2.5, resolution=0.1
    )
    all_points.append(wall6_points)
    
    # 合并所有点
    all_points = np.vstack(all_points)
    
    # 写入PCD文件
    output_file = "src/Intent-MPC/autonomous_flight/cfg/saved_map/corridor_exp111_map.pcd"
    write_pcd_file(output_file, all_points)
    print(f"Created PCD file: {output_file}")
    print(f"Total points: {len(all_points)}")

if __name__ == "__main__":
    main()

