# 无人机下落问题修复

## 问题现象

- ✅ 无人机成功加载到 Gazebo（spawn 成功）
- ✅ ROS 节点和 topic 都正常
- ❌ **但在 Gazebo 中看不到无人机**

## 问题诊断

### 检查无人机位置
```bash
gz model -m quadcopter -p
# 输出: 0 0 -28383.2 0 0 -3.14
```

### 检查里程计
```bash
rostopic echo /CERLAB/quadcopter/odom -n 1
# 输出: z: -38433.01830681685
```

**发现**: 无人机 Z 坐标是负数，而且在疯狂下降！

## 根本原因

`test_head_on.world` 文件**缺少地面平面 (ground_plane)**

- 没有地面，无人机一直在自由落体
- 掉到很深的位置（-28000 米以下）
- 在 Gazebo 视角中看不到

## 解决方案

已添加地面平面到 `test_head_on.world`：

```xml
<model name='ground_plane'>
  <static>true</static>
  <link name='link'>
    <collision name='collision'>
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>100</mu>
            <mu2>50</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
    <visual name='visual'>
      <geometry>
        <plane>
          <normal>0 0 1</normal>
          <size>100 100</size>
        </plane>
      </geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Grey</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

## 重新启动

```bash
# 停止当前 Gazebo (Ctrl+C)

# 重新启动
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

## 验证修复

启动后，检查无人机位置：

```bash
# 应该在地面上 (z ≈ 0.1 或 1.0)
gz model -m quadcopter -p

# 里程计也应该正常
rostopic echo /CERLAB/quadcopter/odom -n 1
```

## 为什么之前没有这个问题？

- `test_keep_some.world` 等其他 world 文件都有 ground_plane
- 创建 `test_head_on.world` 时可能是从模板复制的，但漏掉了地面
- 这是第二次出现这个问题，之前修复过一次但文件可能被覆盖了

## 经验教训

**创建新的 world 文件时，必须包含：**
1. ✅ 光源 (light)
2. ✅ **地面平面 (ground_plane)** - 非常重要！
3. ✅ 物理引擎设置
4. ✅ 障碍物模型
