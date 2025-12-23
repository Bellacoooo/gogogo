# 静态地图创建指南

## 1. 静态地图加载机制

RViz 中的静态地图是通过以下方式加载的：

- **文件格式**: PCD (Point Cloud Data) 格式
- **参数文件**: `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/mapping_param.yaml`
- **参数名**: `prebuilt_map_directory`
- **当前配置**: `/cfg/saved_map/demo_map.pcd` (相对路径)

## 2. 创建自定义静态地图

### 方法 1: 使用提供的 Python 脚本（推荐）

#### 步骤 1: 运行脚本生成 PCD 文件

```bash
cd /home/ff/intent-mpc
python3 create_simple_map.py my_custom_map.pcd
```

#### 步骤 2: 修改脚本自定义障碍物

编辑 `create_simple_map.py`，在 `create_simple_map()` 函数中修改障碍物配置：

```python
# 示例：添加一个障碍物盒子
obstacles = [
    {'center': [2.0, 2.0, 0.0], 'size': [1.0, 1.0, 1.5]},  # 位置 (2,2)，尺寸 1x1x1.5 米
    {'center': [-3.0, 1.0, 0.0], 'size': [2.0, 0.5, 2.0]}, # 位置 (-3,1)，尺寸 2x0.5x2 米
    # 添加更多障碍物...
]
```

#### 步骤 3: 将 PCD 文件放到正确的位置

根据 `mapping_param.yaml` 中的配置，PCD 文件应该放在：

```bash
# 如果参数是相对路径（如 "/cfg/saved_map/demo_map.pcd"）
# 文件应该放在包的 cfg 目录下
mkdir -p src/Intent-MPC/autonomous_flight/cfg/saved_map
cp my_custom_map.pcd src/Intent-MPC/autonomous_flight/cfg/saved_map/demo_map.pcd
```

或者修改 `mapping_param.yaml` 使用绝对路径：

```yaml
prebuilt_map_directory: "/home/ff/intent-mpc/my_custom_map.pcd"
```

#### 步骤 4: 更新参数文件

编辑 `src/Intent-MPC/autonomous_flight/cfg/mpc_navigation/mapping_param.yaml`:

```yaml
prebuilt_map_directory: "/cfg/saved_map/my_custom_map.pcd"  # 相对路径
# 或
prebuilt_map_directory: "/home/ff/intent-mpc/my_custom_map.pcd"  # 绝对路径
```

### 方法 2: 使用 PCL 工具

如果你有现成的点云数据，可以使用 PCL 工具转换：

```bash
# 安装 PCL 工具（如果还没有）
sudo apt-get install pcl-tools

# 转换点云格式
pcl_convert_pcd_ascii_binary input.pcd output.pcd 0  # 0 = ASCII
```

### 方法 3: 从 RViz 录制点云

1. 在 RViz 中订阅点云话题（如 `/camera/depth/points`）
2. 使用 `pcl_ros` 工具录制：
   ```bash
   rosrun pcl_ros pointcloud_to_pcd input:=/camera/depth/points _prefix:=my_map
   ```
3. 合并多个 PCD 文件（如果需要）

## 3. PCD 文件格式说明

PCD 文件是 ASCII 格式，结构如下：

```
# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH <点数>
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS <点数>
DATA ascii
<x1> <y1> <z1>
<x2> <y2> <z2>
...
```

## 4. 注意事项

1. **坐标系**: 地图使用世界坐标系（通常是 `map` frame）
2. **单位**: 坐标单位是米（m）
3. **分辨率**: 建议点云密度为 0.1 米（与 `map_resolution: 0.1` 一致）
4. **高度**: 确保障碍物的 z 坐标在合理范围内（通常 0 到 3 米）
5. **膨胀**: 系统会自动根据 `robot_size` 对障碍物进行膨胀，所以不需要手动膨胀

## 5. 验证地图

1. 重新编译（如果需要）：
   ```bash
   cd /home/ff/intent-mpc
   catkin_make
   ```

2. 启动系统：
   ```bash
   roslaunch autonomous_flight intent_mpc_demo.launch
   ```

3. 在 RViz 中查看：
   - 添加 `Map` display
   - 订阅话题: `/dynamic_map/2D_occupancy_map` 或 `/occupancy_map/2D_occupancy_map`
   - 应该能看到你创建的静态地图

## 6. 示例：创建一个走廊地图

```python
# 在 create_simple_map.py 中修改
def create_corridor_map():
    all_points = []
    resolution = 0.1
    
    # 走廊：长 20 米，宽 3 米，高 2.5 米
    length = 20.0
    width = 3.0
    height = 2.5
    
    # 左右两面墙
    for y in np.arange(-width/2, width/2, resolution):
        for z in np.arange(0, height, resolution):
            # 左墙
            all_points.append([-length/2, y, z])
            # 右墙
            all_points.append([length/2, y, z])
    
    # 添加一些障碍物
    obstacles = [
        {'center': [5.0, 0.0, 0.0], 'size': [1.0, 1.0, 1.5]},
        {'center': [-5.0, 0.0, 0.0], 'size': [1.0, 1.0, 1.5]},
    ]
    
    # ... 生成障碍物点云 ...
```

## 7. 故障排除

- **地图不显示**: 检查 `prebuilt_map_directory` 路径是否正确
- **地图位置不对**: 检查坐标系和原点设置
- **障碍物太大/太小**: 调整 `robot_size` 和 `map_resolution` 参数

