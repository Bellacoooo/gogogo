# SIPP 清理总结

## ✅ 已完成的清理

### 1. .bashrc 配置清理

已删除以下行：
```bash
# 第160行：source ~/sipp/devel/setup.bash
# 第161行：source /home/ff/sipp/devel/setup.bash
```

**备份位置**：`~/.bashrc.backup_YYYYMMDD_HHMMSS`

### 2. 验证清理结果

```bash
grep "sipp" ~/.bashrc
# 应该无输出，表示已清理干净
```

---

## ⚠️ SIPP 目录处理

### 当前状态

SIPP 项目目录仍然存在：
```
/home/ff/sipp/
```

### 选项

#### 选项1：保留目录（推荐）

如果以后可能还需要，可以保留目录。
- 不占用太多空间
- 不影响 intent-mpc 项目
- 已从 .bashrc 中移除，不会干扰

#### 选项2：删除目录

如果确定不再需要：

```bash
# 查看目录大小
du -sh /home/ff/sipp

# 备份（可选）
tar -czf ~/sipp_backup_$(date +%Y%m%d).tar.gz /home/ff/sipp

# 删除
rm -rf /home/ff/sipp
```

**警告**：删除前请确认不需要其中的代码或数据！

---

## 🔧 使新配置生效

### 方法1：重新加载 .bashrc

```bash
source ~/.bashrc
```

### 方法2：重启终端

关闭所有终端，重新打开。

### 方法3：完全重置会话

```bash
# 注销并重新登录
logout
```

---

## 🧪 验证修复

### 1. 检查环境变量

**新开终端**，运行：

```bash
echo $ROS_PACKAGE_PATH
```

**不应该**包含 `/home/ff/sipp`。

### 2. 测试 intent-mpc

```bash
cd /home/ff/intent-mpc
source devel/setup.bash
echo $ROS_PACKAGE_PATH | head -c 60
# 应该显示：/home/ff/intent-mpc/src:...

roslaunch uav_simulator start.launch
```

**应该能看到无人机！**

---

## 📋 已清理的内容

✅ ~/.bashrc 中的 sipp source 语句（2行）  
✅ 环境变量不再指向 sipp  
⏸️ /home/ff/sipp/ 目录（保留，用户可选择删除）

---

## 🎯 下一步

### 立即测试

1. **新开终端**
2. 运行：
```bash
cd /home/ff/intent-mpc
source devel/setup.bash
roslaunch uav_simulator start.launch
```

3. **应该能看到无人机了！**

---

## 🔄 如果需要恢复

备份文件在：
```bash
ls -lh ~/.bashrc.backup_*
```

恢复命令：
```bash
cp ~/.bashrc.backup_YYYYMMDD_HHMMSS ~/.bashrc
source ~/.bashrc
```

---

## 💡 为什么这样做

### 问题原因

.bashrc 中同时 source 多个工作空间会导致：
- ROS_PACKAGE_PATH 冲突
- 环境变量混乱
- 模型路径错误

### 正确做法

1. .bashrc 中只 source ROS 基础环境：
```bash
source /opt/ros/noetic/setup.bash
```

2. 每个项目在需要时单独 source：
```bash
cd /home/ff/intent-mpc
source devel/setup.bash
```

3. 不同项目使用不同终端，互不干扰。

---

## 🆘 如果还有问题

### 检查是否有其他冲突

```bash
# 检查所有 ROS 相关的环境变量
env | grep ROS | sort

# 检查 .bashrc 中所有 source 语句
grep "source.*setup.bash" ~/.bashrc
```

### 完全重置环境

```bash
# 在新终端中
unset ROS_PACKAGE_PATH
unset ROSLISP_PACKAGE_DIRECTORIES
unset CMAKE_PREFIX_PATH

source /opt/ros/noetic/setup.bash
cd /home/ff/intent-mpc
source devel/setup.bash
```

---

## ✅ 预期结果

清理完成后：
- ✅ 无人机能在 Gazebo 中正常显示
- ✅ 环境变量指向正确的工作空间
- ✅ 不再有 sipp 相关的冲突
- ✅ intent-mpc 项目正常工作


