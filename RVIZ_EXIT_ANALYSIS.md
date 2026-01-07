# RViz 退出分析

## 现象
终端显示: "REQUIRED process [rviz1-5] has died! process has finished cleanly"

## 分析

### ✅ 不是崩溃
- 日志明确显示: **"process has finished cleanly"** (进程正常退出)
- RViz 日志中没有 ERROR、FATAL、exception 等错误信息
- 只有正常的启动信息和目标点设置信息

### 可能的原因

1. **手动关闭** (最可能)
   - 用户点击了 RViz 窗口的关闭按钮
   - 或按了 Ctrl+C 停止

2. **launch 文件配置**
   - RViz 在 launch 文件中标记为 "REQUIRED"
   - 当 RViz 退出时，整个 launch 组会关闭（这是正常行为）

3. **RViz 自动退出**
   - 某些配置可能导致 RViz 在完成特定任务后自动退出
   - 但这种情况较少见

## launch 文件中的 REQUIRED 标记

在 ROS launch 文件中，如果一个节点标记为 `required="true"`，那么：
- 当该节点退出时（无论正常或异常）
- 整个 launch 组都会被关闭
- 这是一种保护机制，确保关键组件失败时整个系统停止

## 系统状态

从日志看：
- ✅ MPC navigation 运行正常（最后时间戳 32.368s）
- ✅ 风险地图更新正常
- ✅ 发布了新的 topic: `/mpcNavigation/infeasible`
- ✅ 无人机接收了目标点设置

## 建议

### 如果想让 RViz 退出不影响其他节点：
修改 `intent_mpc_demo.launch` 中的 RViz 配置：
```xml
<!-- 从 required="true" 改为 required="false" -->
<node pkg="rviz" type="rviz" name="rviz1" required="false" .../>
```

### 如果想继续实验：
RViz 退出不影响数据记录，CSV 文件应该已经保存。
只需检查：
```bash
find ~/intent-mpc/src/Intent-MPC/flight_data_recorder -name "*.csv" -mtime -1
```

## 结论

这**不是错误或崩溃**，只是 RViz 正常退出导致 launch 组关闭。
数据应该已经记录到 CSV 文件中。
