# 轨迹采样优化方案

**问题**：当前方法每次采样都要逐步预测并检测碰撞，很多采样会白费（预测到一半发现碰撞）

**目标**：在采样前或采样过程中，更早地判断轨迹是否会碰撞，避免无效计算

**重要修正**：转弯轨迹是圆弧，不能用直线段检测！必须使用折线段检测（polyline collision checking）

---

## 方案1：终点预检测 + 折线段检测（推荐）⭐

### 核心思想
1. **先检测终点**：如果终点就碰撞，整条轨迹肯定无效
2. **直行轨迹**：使用直线段检测（`isInflatedOccupiedLine`）
3. **转弯轨迹**：使用折线段检测（polyline collision checking）- **关键修正！**

### ⚠️ 关键问题：圆弧轨迹 ≠ 直线段

**原错误做法**：
- 转弯轨迹用 `isInflatedOccupiedLine(currPos, endPos)` 检测弦线
- **问题1（漏检）**：圆弧撞墙但弦线没穿过墙 → 误判为安全（严重！）
- **问题2（误杀）**：弦线穿过障碍但圆弧绕开 → 丢掉可行轨迹（保守但减少样本）

**正确做法**：
- 直行：可以用直线段检测（确实是直线）
- 转弯：必须用折线段检测（polyline），每隔几个点检测一次线段

### 实现

#### 直行轨迹（modelForward）

```cpp
void predictor::modelForward(...) {
    // ... 采样循环 ...
    for (double i=minAngle; i<maxAngle; i+=0.1){
        for (double j=minVel; j<maxVel; j+=0.1){
            // ✅ 优化1：快速计算终点位置（直线）
            double maxDist = j * this->numPred_ * this->dt_;
            Eigen::Vector3d endPos = currPos + Eigen::Vector3d(
                maxDist * cos(i),
                maxDist * sin(i),
                currPos(2)
            );
            
            // ✅ 优化2：终点预检测
            if (this->map_->isInflatedOccupied(endPos)){
                continue;  // 终点碰撞，跳过
            }
            
            // ✅ 优化3：直线段检测（直行轨迹是直线，可以用）
            if (this->map_->isInflatedOccupiedLine(currPos, endPos)){
                continue;  // 路径碰撞，跳过
            }
            
            // ✅ 优化4：通过预检测的才详细预测
            // ... 正常预测 ...
        }
    }
}
```

#### 转弯轨迹（modelTurning）- **关键修正**

```cpp
void predictor::modelTurning(...) {
    const int segmentStride = 3;  // 折线段检测步长
    
    for (double i = minVel; i<maxVel; i+=0.2){
        for (double j = minAngVel; j<maxAngVel; j+=0.2){
            for (double endAngle = endMin; endAngle<endMax; endAngle+=0.2){
                // ✅ 优化1：终点预检测（快速拒绝）
                Eigen::Vector3d endPos = /* 计算圆弧终点 */;
                if (this->map_->isInflatedOccupied(endPos)){
                    continue;
                }
                
                // ✅ 优化2：生成轨迹离散点
                std::vector<Eigen::Vector3d> predPointTemp;
                predPointTemp.push_back(currPos);
                
                // ✅ 优化3：折线段检测（polyline collision checking）
                for (int k=0; k<this->numPred_; k++){
                    // 生成下一个点
                    Eigen::Vector3d p = /* 预测点 */;
                    predPointTemp.push_back(p);
                    
                    // 点检测（双重保险）
                    if (this->map_->isInflatedOccupied(p)){
                        isValid = false;
                        break;
                    }
                    
                    // 折线段检测：每隔stride个点检测一次线段
                    if (predPointTemp.size() > segmentStride && 
                        (predPointTemp.size() - 1) % segmentStride == 0) {
                        size_t segStartIdx = predPointTemp.size() - segmentStride - 1;
                        size_t segEndIdx = predPointTemp.size() - 1;
                        if (this->map_->isInflatedOccupiedLine(
                                predPointTemp[segStartIdx], 
                                predPointTemp[segEndIdx])) {
                            isValid = false;
                            break;  // 线段碰撞，early rejection
                        }
                    }
                }
            }
        }
    }
}
```

### 优势
- ✅ **减少40-70%无效计算**：终点碰撞的轨迹直接跳过
- ✅ **使用已有API**：`isInflatedOccupiedLine` 已实现
- ✅ **对圆弧轨迹正确**：折线段检测避免漏检和误杀
- ✅ **论文可写**："对曲线轨迹采用折线近似，将连续曲线表示为若干线段，并在膨胀栅格地图上进行线段占据检测，以实现高效的可行性过滤"

---

## 方案2：方向预筛选（更激进）

### 核心思想
在采样角度前，先快速检测各个方向是否可行

### 实现

```cpp
void predictor::modelForwardOptimized(...) {
    // ✅ 预筛选：快速检测各个方向是否可行
    std::vector<double> validAngles;
    for (double angle = minAngle; angle < maxAngle; angle += 0.2) {  // 粗采样
        double testDist = maxVel * this->numPred_ * this->dt_;
        Eigen::Vector3d testEnd = currPos + Eigen::Vector3d(
            testDist * cos(angle),
            testDist * sin(angle),
            0
        );
        
        if (!this->map_->isInflatedOccupied(testEnd)) {
            validAngles.push_back(angle);  // 记录可行方向
        }
    }
    
    // ✅ 只在可行方向内精细采样
    for (double i : validAngles) {
        for (double j=minVel; j<maxVel; j+=0.1){
            // ... 正常采样（此时方向已确认可行）...
        }
    }
}
```

### 优势
- ✅ **大幅减少采样空间**：只采样可行方向
- ✅ **适合大范围采样**：当角度范围很大时效果明显

---

## 方案3：分层采样（平衡方案）

### 核心思想
先用粗采样找到可行区域，再在可行区域内精细采样

### 实现

```cpp
void predictor::modelForwardOptimized(...) {
    // 第一层：粗采样（快速筛选）
    std::vector<std::pair<double, double>> validSamples;  // (angle, vel)
    
    for (double i=minAngle; i<maxAngle; i+=0.3){  // 粗角度步长
        for (double j=minVel; j<maxVel; j+=0.3){   // 粗速度步长
            double maxDist = j * this->numPred_ * this->dt_;
            Eigen::Vector3d endPos = currPos + Eigen::Vector3d(
                maxDist * cos(i),
                maxDist * sin(i),
                0
            );
            
            if (!this->map_->isInflatedOccupiedLine(currPos, endPos)) {
                validSamples.push_back({i, j});  // 记录可行采样
            }
        }
    }
    
    // 第二层：在可行区域精细采样
    for (auto& sample : validSamples) {
        double centerAngle = sample.first;
        double centerVel = sample.second;
        
        // 在可行采样周围精细采样
        for (double i = centerAngle - 0.1; i <= centerAngle + 0.1; i += 0.05) {
            for (double j = centerVel - 0.1; j <= centerVel + 0.1; j += 0.05) {
                // ... 正常预测 ...
            }
        }
    }
}
```

### 优势
- ✅ **平衡效率和精度**：粗采样筛选 + 精细采样
- ✅ **自适应采样密度**：可行区域采样更密

---

## 方案4：基于可达性地图（最优化，但复杂）

### 核心思想
预先计算从当前位置可达的区域，只在这些区域采样

### 实现思路

```cpp
// 在predictor类中添加
std::vector<Eigen::Vector3d> reachableRegions_;  // 可达区域列表

void predictor::updateReachableRegions(const Eigen::Vector3d& currPos) {
    reachableRegions_.clear();
    
    // 使用raycast快速计算可达区域
    for (double angle = 0; angle < 2*M_PI; angle += 0.1) {
        Eigen::Vector3d direction(cos(angle), sin(angle), 0);
        Eigen::Vector3d endPos;
        
        if (this->map_->castRay(currPos, direction, endPos, maxDist)) {
            // 射线可达，记录终点
            reachableRegions_.push_back(endPos);
        }
    }
}

void predictor::modelForwardOptimized(...) {
    // 只在可达区域内采样
    for (const auto& target : reachableRegions_) {
        // 计算对应的速度和角度
        // ... 采样 ...
    }
}
```

### 优势
- ✅ **最优效率**：只采样可达区域
- ❌ **实现复杂**：需要维护可达性地图

---

## 推荐实现：方案1（最简单有效）

### 修改代码

**文件**：`dynamicPredictor.cpp`

**修改位置1**：`modelForward` 函数（第962行）

```cpp
void predictor::modelForward(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
    predPoints.clear();
    predSize.clear();
    double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2));
    double angleInit = atan2(currVel(1), currVel(0));
    double minVel, maxVel;
    double minAngle, maxAngle;
    minVel = vel-vel;
    maxVel = vel+vel;
    minAngle = angleInit - this->frontAngle_;
    maxAngle = angleInit + this->frontAngle_;

    // ✅ 优化：预计算最大预测距离
    double maxPredDist = maxVel * this->numPred_ * this->dt_;

    for (double i=minAngle; i<maxAngle; i+=0.1){
        for (double j=minVel; j<maxVel; j+=0.1){
            // ✅ 优化1：快速计算终点
            Eigen::Vector3d endPos = currPos + Eigen::Vector3d(
                j * this->numPred_ * this->dt_ * cos(i),
                j * this->numPred_ * this->dt_ * sin(i),
                currPos(2)
            );
            
            // ✅ 优化2：终点预检测（最快判断）
            if (this->map_->isInflatedOccupied(endPos)){
                continue;  // 终点碰撞，跳过
            }
            
            // ✅ 优化3：线段检测（比逐步检测快）
            if (this->map_->isInflatedOccupiedLine(currPos, endPos)){
                continue;  // 路径碰撞，跳过
            }
            
            // ✅ 优化4：通过预检测的才详细预测
            std::vector<Eigen::Vector3d> predPointTemp;
            Eigen::VectorXd currState(4);
            currState<<currPos(0), currPos(1), j*cos(i), j*sin(i);
            predPointTemp.clear();
            predPointTemp.push_back(currPos);
            
            bool isValid = true;
            for (int k=0; k<this->numPred_;k++){
                Eigen::MatrixXd model;
                model = MatrixXd::Identity(4,4);
                model.block(0,2,2,2) = Eigen::MatrixXd::Identity(2,2)*this->dt_;
                Eigen::VectorXd nextState = model*currState;
                Eigen::Vector3d p;
                p << nextState(0), nextState(1), currPos(2);
                
                // 双重保险：虽然预检测过了，但还是要检查（防止数值误差）
                if (this->map_->isInflatedOccupied(p)){
                    isValid = false;
                    break;
                }
                else{
                    predPointTemp.push_back(p);
                }
                currState = nextState;
            }
            
            if (isValid){
                predPoints.push_back(predPointTemp);
            }
        }
    }

    for (int i=0;i<this->numPred_+1;i++){
        predSize.push_back(currSize);
    }
}
```

**修改位置2**：`modelTurning` 函数（第1015行）

同样的优化逻辑应用到转弯模型。

---

## 性能提升估算

### 原方法
```
采样数：角度20个 × 速度20个 = 400个采样
每个采样：预测30步，每步检测1次 = 30次检测
总检测次数：400 × 30 = 12,000次
假设50%碰撞：6,000次无效检测
```

### 优化后（方案1）
```
采样数：400个
预检测：400次终点检测 + 400次线段检测 = 800次
详细预测：200个（50%通过预检测）× 30步 = 6,000次
总检测次数：800 + 6,000 = 6,800次
节省：12,000 - 6,800 = 5,200次（43%）
```

### 如果碰撞率更高（80%碰撞）
```
原方法：12,000次检测
优化后：800 + 2,400 = 3,200次
节省：8,800次（73%）
```

---

## 注意事项

1. **折线段检测的步长**：`segmentStride=3` 表示每3个点检测一次线段
   - 步长越小：检测更精确，但开销更大
   - 步长越大：检测更快，但可能漏检小障碍物
   - 建议：3-5个点，平衡精度和速度

2. **点检测保留**：即使有折线段检测，仍然保留点检测作为双重保险
   - 防止小障碍物漏检
   - 防止折线段检测的数值误差

3. **终点计算**：
   - 直行：恒定速度模型，终点准确
   - 转弯：圆弧近似，用于快速预筛选

4. **性能权衡**：预检测也有开销，但远小于详细预测
   - 建议：先用方案1测试，如果还不够快再用方案2

---

## 实现优先级

1. **立即实现**：方案1（终点预检测 + 线段检测）
2. **如果还不够快**：方案2（方向预筛选）
3. **长期优化**：方案4（可达性地图）

