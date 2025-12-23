#!/usr/bin/env python3
"""
简单的静态地图创建工具
用法: python3 create_simple_map.py output.pcd
"""

import numpy as np

def create_simple_map(output_file):
    """
    生成一个仅包含两个圆柱障碍物的简单地图
    - 无围墙
    - 两个圆柱在无人机前方（默认朝 +x 方向看）
    """
    all_points = []
    # 注意：这里分辨率最好比 mapping_param.yaml 里的 map_resolution 更细一些，
    # 这样每个体素里至少会落到一个点，不会出现“漏格子”的情况。
    resolution = 0.05  # 点云采样间距（米），建议 <= map_resolution/2

    # ========== 自定义障碍物：两个“实心”圆柱 ==========
    cylinders = [
        {'center': [2.0, 0.5, 0.0], 'radius': 0.5, 'height': 2.0},  # 前方偏左
        {'center': [4.0, -0.5, 0.0], 'radius': 0.5, 'height': 2.0}, # 更远偏右
    ]

    for cyl in cylinders:
        cx, cy, cz = cyl['center']
        r = cyl['radius']
        h = cyl['height']

        # 体素方式填充整个圆柱体体积（不是只画外壳）
        # 为了尽量对齐到栅格中心，我们在每个方向上偏移 half_res
        half_res = resolution * 0.5
        z_min = cz
        z_max = cz + h
        x_min = cx - r
        x_max = cx + r
        y_min = cy - r
        y_max = cy + r

        z_vals = np.arange(z_min + half_res, z_max + half_res, resolution)
        x_vals = np.arange(x_min + half_res, x_max + half_res, resolution)
        y_vals = np.arange(y_min + half_res, y_max + half_res, resolution)

        for z in z_vals:
            for x in x_vals:
                for y in y_vals:
                    if (x - cx) ** 2 + (y - cy) ** 2 <= r ** 2:
                        all_points.append([x, y, z])

    # 写入 PCD 文件
    # 规范化路径，方便后续在 mapping_param.yaml 中直接使用绝对路径
    import os
    abs_output = os.path.abspath(output_file)

    with open(abs_output, 'w') as f:
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

        for point in all_points:
            f.write(f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f}\n")

    print(f"✓ 成功创建静态地图: {abs_output}")
    print(f"  - 总点数: {len(all_points)}")
    print(f"  - 圆柱数量: {len(cylinders)}，半径约 {cylinders[0]['radius']} m，高度约 {cylinders[0]['height']} m")
    print("\n下一步把下面这行填到 mapping_param.yaml 的 prebuilt_map_directory:")
    print(f'prebuilt_map_directory: "{abs_output}"')

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("用法: python3 create_simple_map.py <output.pcd>")
        print("\n示例:")
        print("  python3 create_simple_map.py my_map.pcd")
        sys.exit(1)
    
    create_simple_map(sys.argv[1])

