#!/usr/bin/env python3
"""
简单的静态地图创建工具
用法: python3 create_simple_map.py <output.pcd> [config_name]

config_name 可选 (默认 big6_8):
  big6_8    -> 对应 worlds/test/big6_8.world  (6静态方块 + 8动态圆柱)
  big8_10   -> 对应 worlds/test/big8_10.world (8静态方块 + 10动态圆柱)
  tongxiang -> 对应 worlds/test/test_tongxiang.world

示例:
  python3 create_simple_map.py big6_8.pcd  big6_8
  python3 create_simple_map.py big8_10.pcd big8_10
"""

import numpy as np
import os
import sys

# ===================================================================
# 各场景静态障碍物配置
# 说明: PCD 地图加载后会被栅格地图膨胀，因此这里的 width/depth 比
#       world 文件中实际尺寸缩小约 0.3m，以补偿膨胀带来的偏差。
# center: [x, y, z_bottom]，z_bottom 通常为 0.0 (从地面开始)
# ===================================================================

# ------------------------------------------------------------------
# big6_8 地图配置
# 对应 worlds/test/big6_8.world，地图范围 X:[-5,5]  Y:[-10,10]
# world 实际尺寸(W x D)  ->  PCD 缩减尺寸 (各减约 0.3m 应对膨胀)
# ------------------------------------------------------------------
boxes_big6_8 = [
    {'center': [-3.0, -7.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [ 2.0, -5.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [-1.0, -1.0, 0.0], 'width': 0.9, 'depth': 0.7, 'height': 2.0},  # world: 1.2x1.0
    {'center': [ 3.0,  2.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [-3.0,  5.0, 0.0], 'width': 0.7, 'depth': 0.9, 'height': 2.0},  # world: 1.0x1.2
    {'center': [ 1.0,  8.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
]

# ------------------------------------------------------------------
# big8_10 地图配置
# 对应 worlds/test/big8_10.world，地图范围 X:[-5,5]  Y:[-10,10]
# world 实际尺寸(W x D)  ->  PCD 缩减尺寸 (各减约 0.3m 应对膨胀)
# ------------------------------------------------------------------
boxes_big8_10 = [
    {'center': [-3.0, -8.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [ 3.0, -7.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [-2.0, -4.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [ 3.0, -3.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [-3.0,  0.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [ 1.0,  2.0, 0.0], 'width': 0.7, 'depth': 1.2, 'height': 2.0},  # world: 1.0x1.5
    {'center': [-3.0,  5.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
    {'center': [ 3.0,  7.0, 0.0], 'width': 0.7, 'depth': 0.7, 'height': 2.0},  # world: 1.0x1.0
]

# ------------------------------------------------------------------
# tongxiang 地图配置（保留供参考，已注释）
# 对应 worlds/test/test_tongxiang.world
# ------------------------------------------------------------------

# # tongxiang 旧版参数（较小尺寸）
# boxes_tongxiang_v1 = [
#     {'center': [-1.0,   2.0,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [-1.0,  -2.5,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [-5.0,   2.5,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [-7.0,  -2.2,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [ 1.45, -1.25, 0.0], 'width': 0.5, 'depth': 0.7, 'height': 2.0},
# ]

# # tongxiang 当前版本参数
# boxes_tongxiang = [
#     {'center': [-1.0,   2.0,  0.0], 'width': 1.0, 'depth': 1.0, 'height': 2.0},
#     {'center': [-1.0,  -2.5,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [-5.0,   2.5,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [-7.0,  -2.2,  0.0], 'width': 0.5, 'depth': 0.5, 'height': 2.0},
#     {'center': [ 1.45, -1.25, 0.0], 'width': 1.0, 'depth': 1.5, 'height': 2.0},
# ]

# ------------------------------------------------------------------
# 配置索引（新增场景在此注册）
# ------------------------------------------------------------------
MAP_CONFIGS = {
    'big6_8' : boxes_big6_8,
    'big8_10': boxes_big8_10,
    # 'tongxiang': boxes_tongxiang,
}


def create_simple_map(output_file, config_name='big6_8'):
    """
    根据指定配置生成静态地图 PCD 文件。
    分辨率建议 <= mapping_param.yaml 中 map_resolution 的一半，
    以确保每个体素里至少落到一个点，不会出现漏格子。
    """
    if config_name not in MAP_CONFIGS:
        print("未知配置名: '%s'，可用配置: %s" % (config_name, list(MAP_CONFIGS.keys())))
        sys.exit(1)

    boxes = MAP_CONFIGS[config_name]
    all_points = []
    resolution = 0.05  # 点云采样间距（米），建议 <= map_resolution / 2

    for box in boxes:
        cx, cy, cz = box['center']
        w = box['width']   # x 方向宽度
        d = box['depth']   # y 方向深度
        h = box['height']  # z 方向高度

        # 体素方式填充整个长方体体积（不是只画外壳）
        # 为了对齐到栅格中心，每个方向上偏移 half_res
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
                    all_points.append([x, y, z])

    # 写入 PCD 文件
    abs_output = os.path.abspath(output_file)
    with open(abs_output, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z\n")
        f.write("SIZE 4 4 4\n")
        f.write("TYPE F F F\n")
        f.write("COUNT 1 1 1\n")
        f.write("WIDTH %d\n" % len(all_points))
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write("POINTS %d\n" % len(all_points))
        f.write("DATA ascii\n")
        for point in all_points:
            f.write("%.6f %.6f %.6f\n" % (point[0], point[1], point[2]))

    print("成功创建静态地图 [%s]: %s" % (config_name, abs_output))
    print("  - 障碍物数量: %d" % len(boxes))
    print("  - 总点数:     %d" % len(all_points))
    print("\n下一步把下面这行填到 mapping_param.yaml 的 prebuilt_map_directory:")
    print('  prebuilt_map_directory: "%s"' % abs_output)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python3 create_simple_map.py <output.pcd> [config_name]")
        print("\n可用配置: %s" % list(MAP_CONFIGS.keys()))
        print("\n示例:")
        print("  python3 create_simple_map.py big6_8.pcd  big6_8")
        print("  python3 create_simple_map.py big8_10.pcd big8_10")
        sys.exit(1)

    output = sys.argv[1]
    config = sys.argv[2] if len(sys.argv) >= 3 else 'big6_8'
    create_simple_map(output, config)
