#include <global_planner/risk_map_2d.h>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

namespace globalPlanner
{

RiskMap2D::RiskMap2D()
  : valid_(false)
  , resolution_(0.0)
  , origin_(Eigen::Vector2d::Zero())
  , size_(Eigen::Vector2i::Zero())
{
}

void RiskMap2D::updateFromMsg(const nav_msgs::OccupancyGrid& msg)
{
  // ROS_WARN("[RISK-MAP-UPDATE] updateFromMsg called: msg.width=%u, msg.height=%u, msg.resolution=%.6f, msg.data.size()=%zu",
  //          msg.info.width, msg.info.height, msg.info.resolution, msg.data.size());
  
  std::lock_guard<std::mutex> lock(mutex_);
  // ROS_WARN("[RISK-MAP-UPDATE] Lock acquired");
  
  if (msg.info.width == 0 || msg.info.height == 0 || msg.info.resolution <= 0.0)
  {
    // ROS_WARN("[RISK-MAP-UPDATE] Invalid message parameters, setting valid_=false");
    valid_ = false;
    return;
  }

  resolution_ = msg.info.resolution;
  origin_(0) = msg.info.origin.position.x;
  origin_(1) = msg.info.origin.position.y;
  size_(0) = static_cast<int>(msg.info.width);
  size_(1) = static_cast<int>(msg.info.height);
  
  // ROS_WARN("[RISK-MAP-UPDATE] Parameters set: resolution=%.6f, origin=(%.3f,%.3f), size=(%d,%d)",
  //          resolution_, origin_(0), origin_(1), size_(0), size_(1));

  // 将 OccupancyGrid 的 [0, 100] 值转换为 [0.0, 1.0] 的风险值
  std::size_t expected_size = static_cast<std::size_t>(size_(0)) * static_cast<std::size_t>(size_(1));
  // ROS_WARN("[RISK-MAP-UPDATE] Resizing data_ to %zu elements", expected_size);
  
  try {
    data_.resize(expected_size, 0.0);  // 初始化为0.0，防止未初始化数据
    // ROS_WARN("[RISK-MAP-UPDATE] data_.resize() completed, data_.size()=%zu", data_.size());
  } catch (const std::exception& e) {
    ROS_ERROR("[RISK-MAP-UPDATE] Exception in data_.resize(): %s", e.what());
    valid_ = false;
    return;
  } catch (...) {
    ROS_ERROR("[RISK-MAP-UPDATE] Unknown exception in data_.resize()");
    valid_ = false;
    return;
  }
  
  std::size_t copy_size = std::min(msg.data.size(), expected_size);
  // ROS_WARN("[RISK-MAP-UPDATE] Copying %zu elements from msg.data", copy_size);
  
  std::size_t non_zero_count = 0;
  for (std::size_t i = 0; i < copy_size; ++i)
  {
    int8_t val = msg.data[i];
    // OccupancyGrid: -1=未知, 0=自由, 1-100=占用概率
    // 风险地图: 0-100 表示风险概率
    if (val < 0)
    {
      data_[i] = 0.0;  // 未知区域视为无风险
    }
    else
    {
      data_[i] = static_cast<double>(val) / 100.0;  // 归一化到 [0.0, 1.0]
      if (data_[i] > 1e-6) non_zero_count++;
    }
  }
  
  // ROS_WARN("[RISK-MAP-UPDATE] Copy completed: non_zero_count=%zu, expected_size=%zu, copy_size=%zu",
  //          non_zero_count, expected_size, copy_size);
  
  // 如果 msg.data 小于预期大小，剩余部分保持为 0.0（已在 resize 时初始化）
  valid_ = true;
  // ROS_WARN("[RISK-MAP-UPDATE] updateFromMsg completed: valid_=true, data_.size()=%zu", data_.size());
}

bool RiskMap2D::worldToGrid(double wx, double wy, int& ix, int& iy) const
{
  if (!valid_) return false;

  ix = static_cast<int>(std::floor((wx - origin_(0)) / resolution_));
  iy = static_cast<int>(std::floor((wy - origin_(1)) / resolution_));

  return (ix >= 0 && ix < size_(0) && iy >= 0 && iy < size_(1));
}

double RiskMap2D::getValue(int ix, int iy) const
{
  static int call_count = 0;
  call_count++;
  bool should_log = (call_count <= 50);
  
  if (should_log) {
    ROS_WARN("[RISK-MAP-GET] getValue call #%d: ix=%d, iy=%d, attempting to acquire lock", call_count, ix, iy);
  }
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (should_log) {
    ROS_WARN("[RISK-MAP-GET] getValue call #%d: lock acquired, calling getValueUnlocked", call_count);
  }
  
  return getValueUnlocked(ix, iy);
}

double RiskMap2D::getValueUnlocked(int ix, int iy) const
{
  static int call_count = 0;
  call_count++;
  bool should_log = (call_count <= 50);  // 前50次调用记录日志
  
  if (should_log) {
    ROS_WARN("[RISK-MAP-GET] getValueUnlocked call #%d: ix=%d, iy=%d, valid_=%s, size=(%d,%d), data_.size()=%zu",
             call_count, ix, iy, valid_ ? "true" : "false", size_(0), size_(1), data_.size());
  }
  
  if (!valid_) {
    if (should_log) ROS_WARN("[RISK-MAP-GET] valid_=false, returning 0.0");
    return 0.0;
  }
  if (size_(0) <= 0 || size_(1) <= 0) {
    if (should_log) ROS_WARN("[RISK-MAP-GET] Invalid size: (%d,%d), returning 0.0", size_(0), size_(1));
    return 0.0;  // 防止空地图
  }
  if (data_.empty()) {
    if (should_log) ROS_WARN("[RISK-MAP-GET] data_.empty()=true, returning 0.0");
    return 0.0;  // 防止数据为空
  }
  if (ix < 0 || ix >= size_(0) || iy < 0 || iy >= size_(1)) {
    if (should_log) ROS_WARN("[RISK-MAP-GET] Out of bounds: ix=%d (range: 0-%d), iy=%d (range: 0-%d), returning 0.0",
                             ix, size_(0)-1, iy, size_(1)-1);
    return 0.0;
  }

  std::size_t idx = static_cast<std::size_t>(iy) * static_cast<std::size_t>(size_(0)) + static_cast<std::size_t>(ix);
  if (should_log) {
    ROS_WARN("[RISK-MAP-GET] Index calculation: iy=%d * size_(0)=%d + ix=%d = idx=%zu, data_.size()=%zu",
             iy, size_(0), ix, idx, data_.size());
  }
  
  if (idx >= data_.size()) {
    ROS_ERROR("[RISK-MAP-GET] Index out of bounds: idx=%zu >= data_.size()=%zu, returning 0.0", idx, data_.size());
    return 0.0;
  }

  double result = data_[idx];
  if (should_log) {
    ROS_WARN("[RISK-MAP-GET] Accessing data_[%zu] = %.6f", idx, result);
  }
  return result;
}

double RiskMap2D::queryBilinear(double wx, double wy) const
{
  static int call_count = 0;
  call_count++;
  bool should_debug = (call_count <= 100);  // 前100次调用输出详细日志
  
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] queryBilinear call #%d: world(%.6f,%.6f) - attempting to acquire lock",
             call_count, wx, wy);
  }
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] queryBilinear call #%d: lock acquired", call_count);
  }
  
  if (should_debug)
  {
    ROS_WARN("[RISK-MAP-QUERY] queryBilinear call #%d: world(%.3f,%.3f)", call_count, wx, wy);
  }
  
  if (!valid_)
  {
    if (should_debug)
    {
      ROS_WARN("[RISK-MAP-QUERY] queryBilinear: valid_=false, returning 0.0");
    }
    return 0.0;
  }
  if (resolution_ <= 0.0)
  {
    if (should_debug)
    {
      ROS_WARN("[RISK-MAP-QUERY] queryBilinear: resolution_=%.6f <= 0, returning 0.0", resolution_);
    }
    return 0.0;  // 防止除零
  }
  if (size_(0) <= 0 || size_(1) <= 0)
  {
    if (should_debug)
    {
      ROS_WARN("[RISK-MAP-QUERY] queryBilinear: size_=(%d,%d) invalid, returning 0.0", size_(0), size_(1));
    }
    return 0.0;  // 防止空地图
  }
  if (data_.empty())
  {
    if (should_debug)
    {
      ROS_WARN("[RISK-MAP-QUERY] queryBilinear: data_.empty()=true, returning 0.0");
    }
    return 0.0;  // 防止数据为空
  }

  // 转换为栅格坐标（浮点数）
  double gx = (wx - origin_(0)) / resolution_;
  double gy = (wy - origin_(1)) / resolution_;

  if (should_debug)
  {
    ROS_WARN("[RISK-MAP-QUERY] queryBilinear: world(%.3f,%.3f) -> grid(%.3f,%.3f), "
             "origin=(%.3f,%.3f), resolution=%.3f",
             wx, wy, gx, gy, origin_(0), origin_(1), resolution_);
  }

  // 找到四个相邻栅格
  int x0 = static_cast<int>(std::floor(gx));
  int y0 = static_cast<int>(std::floor(gy));
  int x1 = x0 + 1;
  int y1 = y0 + 1;

  // 检查是否在地图范围内
  if (x0 < 0 || x1 >= size_(0) || y0 < 0 || y1 >= size_(1))
  {
    // 如果完全在地图外，返回0
    if (x1 < 0 || x0 >= size_(0) || y1 < 0 || y0 >= size_(1))
    {
      if (should_debug)
      {
        ROS_WARN("[RISK-MAP-QUERY] queryBilinear: completely out of map, corners: (%d,%d),(%d,%d), size=(%d,%d), returning 0.0",
                 x0, y0, x1, y1, size_(0), size_(1));
      }
      return 0.0;
    }
    // 部分在地图内，使用最近邻插值
    x0 = std::max(0, std::min(x0, size_(0) - 1));
    y0 = std::max(0, std::min(y0, size_(1) - 1));
    double result = getValueUnlocked(x0, y0);
    if (should_debug)
    {
      ROS_WARN("[RISK-MAP-QUERY] queryBilinear: partially out of map, using nearest neighbor (%d,%d), result=%.6f",
               x0, y0, result);
    }
    return result;
  }

  // 双线性插值
  // 再次检查边界（防止在检查后、调用前 size_ 被修改，虽然不太可能）
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] Before bilinear: x0=%d, y0=%d, x1=%d, y1=%d, size=(%d,%d), data_.size()=%zu",
             x0, y0, x1, y1, size_(0), size_(1), data_.size());
  }
  
  if (x1 >= size_(0) || y1 >= size_(1))
  {
    // 如果 x1 或 y1 超出边界，使用最近邻插值
    if (should_debug) {
      ROS_WARN("[RISK-MAP-QUERY] Boundary check failed: x1=%d >= size_(0)=%d OR y1=%d >= size_(1)=%d",
               x1, size_(0), y1, size_(1));
    }
    x0 = std::max(0, std::min(x0, size_(0) - 1));
    y0 = std::max(0, std::min(y0, size_(1) - 1));
    if (should_debug) {
      ROS_WARN("[RISK-MAP-QUERY] Using nearest neighbor: clamped to (%d,%d)", x0, y0);
    }
    double result = getValueUnlocked(x0, y0);
    if (should_debug)
    {
      ROS_WARN("[RISK-MAP-QUERY] queryBilinear: boundary check failed after initial check, using nearest neighbor (%d,%d), result=%.6f",
               x0, y0, result);
    }
    return result;
  }
  
  double fx = gx - static_cast<double>(x0);
  double fy = gy - static_cast<double>(y0);
  
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] Starting bilinear interpolation: fx=%.6f, fy=%.6f", fx, fy);
  }

  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] Calling getValueUnlocked for corner (x0=%d, y0=%d)", x0, y0);
  }
  double v00 = getValueUnlocked(x0, y0);
  
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] Calling getValueUnlocked for corner (x1=%d, y0=%d)", x1, y0);
  }
  double v10 = getValueUnlocked(x1, y0);
  
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] Calling getValueUnlocked for corner (x0=%d, y1=%d)", x0, y1);
  }
  double v01 = getValueUnlocked(x0, y1);
  
  if (should_debug) {
    ROS_WARN("[RISK-MAP-QUERY] Calling getValueUnlocked for corner (x1=%d, y1=%d)", x1, y1);
  }
  double v11 = getValueUnlocked(x1, y1);

  double v0 = v00 * (1.0 - fx) + v10 * fx;
  double v1 = v01 * (1.0 - fx) + v11 * fx;
  double v = v0 * (1.0 - fy) + v1 * fy;

  if (should_debug)
  {
    ROS_WARN("[RISK-MAP-QUERY] queryBilinear: corners: (%d,%d)=%.6f, (%d,%d)=%.6f, (%d,%d)=%.6f, (%d,%d)=%.6f, "
             "fx=%.6f, fy=%.6f, result=%.6f",
             x0, y0, v00, x1, y0, v10, x0, y1, v01, x1, y1, v11,
             fx, fy, v);
  }
  
  return v;
}

bool RiskMap2D::isValid() const
{
  static int check_count = 0;
  check_count++;
  bool should_log = (check_count <= 50);  // 前50次检查记录日志
  
  if (should_log) {
    ROS_WARN("[RISK-MAP-VALID] isValid() call #%d: attempting to acquire lock", check_count);
  }
  
  std::lock_guard<std::mutex> lock(mutex_);
  
  bool result = valid_;
  if (should_log) {
    ROS_WARN("[RISK-MAP-VALID] isValid() call #%d: lock acquired, valid_=%s, size=(%d,%d), data_.size()=%zu",
             check_count, result ? "true" : "false", size_(0), size_(1), data_.size());
  }
  
  return result;
}

} // namespace globalPlanner

