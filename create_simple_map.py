#!/usr/bin/env python3
"""
简单的静态地图创建工具
用法: python3 create_simple_map.py output.pcd
"""

import numpy as np

def create_simple_map(output_file):
    """
    生成一个仅包含五个长方体障碍物的简单地图
    - 无围墙
    - 五个长方体在无人机前方（默认朝 +x 方向看）
    """
    all_points = []
    # 注意：这里分辨率最好比 mapping_param.yaml 里的 map_resolution 更细一些，
    # 这样每个体素里至少会落到一个点，不会出现“漏格子”的情况。
    resolution = 0.05  # 点云采样间距（米），建议 <= map_resolution/2

    # ========== 自定义障碍物：五个"实心"长方体 ==========
# five_square的静态地图参数
    boxes = [
        {'center': [-1.0, 2.0, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},  # 前方偏左
        {'center': [-1.0, -2.5, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0}, # 更远偏右
        {'center': [-7.0, 2.5, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
        {'center': [-7.0, -2.2, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
        {'center': [-4, -0.25, 0.0], 'width': 0.5, 'depth': 0.8, 'height': 2.0},
    ]

# tongxiang的静态地图参数
    # boxes = [
    #     {'center': [-1.0, 2.0, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},  # 前方偏左
    #     {'center': [-1.0, -2.5, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0}, # 更远偏右
    #     {'center': [-5.0, 2.5, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
    #     {'center': [-7.0, -2.2, 0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
    #     {'center': [1.45, -1.25, 0.0], 'width': 0.5, 'depth': 0.7, 'height': 2.0},
    # ]

    for box in boxes:
        cx, cy, cz = box['center']
        w = box['width']   # x方向宽度
        d = box['depth']   # y方向深度
        h = box['height']  # z方向高度

        # 体素方式填充整个长方体体积（不是只画外壳）
        # 为了尽量对齐到栅格中心，我们在每个方向上偏移 half_res
        half_res = resolution * 0.5
        z_min = cz
        z_max = cz + h
        x_min = cx - w / 2.0
        x_max = cx + w / 2.0
        y_min = cy - d / 2.0
        y_max = cy + d / 2.0

        z_vals = np.arange(z_min + half_res, z_max + half_res, resolution)
        x_vals = np.arange(x_min + half_res, x_max + half_res, resolution)
        y_vals = np.arange(y_min + half_res, y_max + half_res, resolution)

        for z in z_vals:
            for x in x_vals:
                for y in y_vals:
                    # 长方体不需要判断，直接添加所有点
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
    print(f"  - 长方体数量: {len(boxes)}，宽度 {boxes[0]['width']} m，深度 {boxes[0]['depth']} m，高度 {boxes[0]['height']} m")
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

