#include <global_planner/a_star_occ.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <limits>
#include <mutex>

namespace globalPlanner
{

AStarOccMap::AStarOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  nh_.param("astar/use_26_dir", use26dir_, false);  // 默认使用6邻域，更稳定
  
  int max_nodes_tmp = 300000;
  nh_.param("astar/max_expanded_nodes", max_nodes_tmp, max_nodes_tmp);
  max_expanded_nodes_ = static_cast<std::size_t>(std::max(1000, max_nodes_tmp));

  nh_.param("astar/grid_resolution", grid_res_param_, 0.5);
  
  // 风险地图参数
  nh_.param("astar/w_risk", w_risk_, 0.0);
  nh_.param("astar/k_risk", k_risk_, 1.0);
  nh_.param("astar/z_gate", z_gate_, 2.0);
  
  // 初始化A*专用路径发布器
  pathPub_ = nh_.advertise<nav_msgs::Path>("astar/planned_path", 10);
  
  ROS_INFO("[A*] Simple 3D A* initialized (grid_res=%.2f m, use26dir=%s).", 
           grid_res_param_, use26dir_ ? "true" : "false");
  ROS_INFO("[A*] Risk map params: w_risk=%.1f, k_risk=%.1f, z_gate=%.2f", 
           w_risk_, k_risk_, z_gate_);
  ROS_INFO("[A*] Raw path will be published to /astar/planned_path");
}

void AStarOccMap::setMap(const std::shared_ptr<mapManager::occMap> &map)
{
  map_ = map;
  if (map_)
  {
    res_ = map_->getRes();
    if (grid_res_param_ <= 1e-6)
    {
      grid_res_ = res_;
    }
    else
    {
      grid_res_ = grid_res_param_;
      if (grid_res_ < res_)
      {
        ROS_WARN("[A*] astar/grid_resolution=%.3f is finer than map resolution %.3f, clamp to map resolution.",
                 grid_res_, res_);
        grid_res_ = res_;
      }
    }
    
    // 获取地图原点偏移（必须与occupancyMap的posToIndex/indexToPos一致）
    Eigen::Vector3d mapSizeMin, mapSizeMax;
    map_->getMapRange(mapSizeMin, mapSizeMax);
    map_origin_ = mapSizeMin;
    
    // 关键修复：A*必须使用和occupancyMap相同的分辨率mapRes_，而不是grid_res_param_
    // 否则栅格索引会不匹配，导致地图查询错误
    grid_res_ = res_;  // 强制使用地图分辨率，忽略grid_res_param_
    
    ROS_INFO("[A*] Using grid resolution %.3f m (map res=%.3f m, param=%.3f m).", 
             grid_res_, res_, grid_res_param_);
    ROS_INFO("[A*] Map origin offset: (%.3f, %.3f, %.3f)", 
             map_origin_.x(), map_origin_.y(), map_origin_.z());
  }
}

void AStarOccMap::updateStart(const geometry_msgs::Pose &start)
{
  start_ = start;
}

void AStarOccMap::updateGoal(const geometry_msgs::Pose &goal)
{
  goal_ = goal;
}

void AStarOccMap::setRiskMap(const std::shared_ptr<RiskMap2D>& risk_map)
{
  risk_map_ = risk_map;
  use_risk_map_25d_ = false;  // 🔧 使用旧版2D风险地图
  // ROS_WARN("[A*-RISK-DEBUG] setRiskMap called: risk_map_=%p, w_risk_=%.3f, k_risk_=%.3f, z_gate_=%.3f",
  //          risk_map_.get(), w_risk_, k_risk_, z_gate_);
  
  if (risk_map_)
  {
    bool is_valid = risk_map_->isValid();
    // ROS_WARN("[A*-RISK-DEBUG] risk_map_ pointer exists, isValid()=%s", is_valid ? "true" : "false");
    
    if (is_valid)
    {
      double res = risk_map_->getResolution();
      Eigen::Vector2i size = risk_map_->getSize();
      Eigen::Vector2d origin = risk_map_->getOrigin();
      // ROS_WARN("[A*-RISK-DEBUG] Risk map valid: resolution=%.3f, size=(%d,%d), origin=(%.2f,%.2f)",
      //          res, size(0), size(1), origin(0), origin(1));
      
      // 检查数据是否非空
      // 注意：这里不能直接访问 data_，但可以通过查询几个点来验证
      // 使用 try-catch 保护，防止 queryBilinear 内部出现问题
      try {
        double test_val = risk_map_->queryBilinear(origin(0), origin(1));
        // ROS_WARN("[A*-RISK-DEBUG] Test query at origin: queryBilinear(%.2f,%.2f)=%.6f",
        //          origin(0), origin(1), test_val);
      } catch (const std::exception& e) {
        // ROS_ERROR("[A*-RISK-DEBUG] Exception in test queryBilinear: %s", e.what());
      } catch (...) {
        // ROS_ERROR("[A*-RISK-DEBUG] Unknown exception in test queryBilinear");
      }
    }
    else
    {
      // ROS_WARN("[A*-RISK-DEBUG] Risk map pointer exists but isValid()=false");
    }
  }
  else
  {
    // ROS_WARN("[A*-RISK-DEBUG] Risk map pointer is NULL");
  }
  
  if (w_risk_ > 0.0)
  {
    // ROS_WARN("[A*-RISK-DEBUG] w_risk_ > 0, risk cost will be used if risk_map_ is valid");
  }
  else
  {
    // ROS_WARN("[A*-RISK-DEBUG] w_risk_ = 0, risk cost will NOT be used");
  }
}

void AStarOccMap::setRiskMap25D(const std::shared_ptr<RiskMap25D>& risk_map_25d)
{
  risk_map_25d_ = risk_map_25d;
  use_risk_map_25d_ = true;  // 🔧 启用2.5D风险地图
  
  ROS_INFO("[A*] 🎯 Using RiskMap25D (2.5D fused static+dynamic risk map)");
  
  if (risk_map_25d_ && risk_map_25d_->isValid())
  {
    // 获取栅格信息
    double resolution;
    int width, height;
    Eigen::Vector2d origin;
    risk_map_25d_->getGridInfo(resolution, width, height, origin);
    
    ROS_INFO("[A*-RiskMap25D] Grid: %dx%d, res=%.2fm, origin=(%.2f,%.2f)",
             width, height, resolution, origin(0), origin(1));
    
    // 测试查询
    try {
      Eigen::Vector3d test_pos(0.0, 0.0, 1.0);
      double test_risk = risk_map_25d_->query(test_pos);
      ROS_INFO("[A*-RiskMap25D] Test query at (0,0,1): risk=%.6f", test_risk);
    } catch (const std::exception& e) {
      ROS_ERROR("[A*-RiskMap25D] Exception in query: %s", e.what());
    }
  }
  else
  {
    ROS_WARN("[A*-RiskMap25D] Risk map not valid or nullptr");
  }
}

bool AStarOccMap::posToIndex(const Eigen::Vector3d &pos, int &ix, int &iy, int &iz) const
{
  if (!map_) return false;
  if (grid_res_ <= 0) {
    return false;
  }
  if (!map_->isInMap(pos)) return false;

  // 必须考虑地图原点偏移，与occupancyMap的posToIndex一致
  // occupancyMap: idx(0) = floor((pos(0) - mapSizeMin_(0)) / mapRes_)
  ix = static_cast<int>(std::floor((pos.x() - map_origin_.x()) / grid_res_));
  iy = static_cast<int>(std::floor((pos.y() - map_origin_.y()) / grid_res_));
  iz = static_cast<int>(std::floor((pos.z() - map_origin_.z()) / grid_res_));
  return true;
}

Eigen::Vector3d AStarOccMap::indexToPos(int ix, int iy, int iz) const
{
  Eigen::Vector3d pos;
  if (grid_res_ <= 0) {
    return Eigen::Vector3d::Zero();
  }
  // 必须考虑地图原点偏移，与occupancyMap的indexToPos一致
  // occupancyMap: pos(0) = (idx(0) + 0.5) * mapRes_ + mapSizeMin_(0)
  pos.x() = (static_cast<double>(ix) + 0.5) * grid_res_ + map_origin_.x();
  pos.y() = (static_cast<double>(iy) + 0.5) * grid_res_ + map_origin_.y();
  pos.z() = (static_cast<double>(iz) + 0.5) * grid_res_ + map_origin_.z();
  return pos;
}

bool AStarOccMap::isFree(int ix, int iy, int iz) const
{
  if (!map_) return false;
  if (grid_res_ <= 0) return false;
  
  // 先检查坐标范围，防止溢出
  const int MAX_COORD = 100000;
  const int MIN_COORD = -100000;
  if (ix < MIN_COORD || ix > MAX_COORD || 
      iy < MIN_COORD || iy > MAX_COORD || 
      iz < MIN_COORD || iz > MAX_COORD) {
    return false;
  }
  
  // 将栅格索引转为世界坐标（格子中心）
  Eigen::Vector3d center = indexToPos(ix, iy, iz);
  
  // 直接使用世界坐标查询地图，和MPC一样
  if (!map_->isInMap(center)) {
    ROS_DEBUG_THROTTLE(0.5, "[A*] isFree: grid=(%d,%d,%d) -> world=(%.3f,%.3f,%.3f) NOT IN MAP",
                       ix, iy, iz, center.x(), center.y(), center.z());
    return false;
  }
  
  // 使用 try-catch 保护，防止地图查询时崩溃
  try {
    bool occupied = map_->isInflatedOccupied(center);
    if (occupied) {
      ROS_DEBUG_THROTTLE(0.5, "[A*] isFree: grid=(%d,%d,%d) -> world=(%.3f,%.3f,%.3f) OCCUPIED",
                         ix, iy, iz, center.x(), center.y(), center.z());
      return false;
    }
    
    // 检查动态障碍物方框硬约束
    if (checkDynamicObstacleCollision(center)) {
      ROS_DEBUG_THROTTLE(0.5, "[A*] isFree: grid=(%d,%d,%d) -> world=(%.3f,%.3f,%.3f) COLLIDES WITH DYNAMIC OBSTACLE",
                         ix, iy, iz, center.x(), center.y(), center.z());
      return false;
    }
    
    return true;
  } catch (const std::exception& e) {
    ROS_WARN_THROTTLE(1.0, "[A*] Exception in isFree: %s, grid=(%d,%d,%d)", e.what(), ix, iy, iz);
    return false;
  }
}

void AStarOccMap::setDynamicPredictions(const std::vector<DynObstaclePred> &preds)
{
  // 不使用
}

double AStarOccMap::calculateDynamicCost(const Eigen::Vector3d &pos) const
{
  return 0.0;
}

void AStarOccMap::setDynamicObstacleBoxes(const std::vector<Eigen::Vector3d>& obstacles_pos,
                                           const std::vector<Eigen::Vector3d>& obstacles_size)
{
  std::lock_guard<std::mutex> lock(dynamic_obstacles_mutex_);
  dynamic_obstacle_boxes_.clear();
  
  if (obstacles_pos.size() != obstacles_size.size()) {
    ROS_WARN("[A*] setDynamicObstacleBoxes: obstacles_pos.size()=%zu != obstacles_size.size()=%zu",
             obstacles_pos.size(), obstacles_size.size());
    return;
  }
  
  for (size_t i = 0; i < obstacles_pos.size(); ++i) {
    DynamicObstacleBox box;
    box.center = obstacles_pos[i];
    box.size = obstacles_size[i];
    dynamic_obstacle_boxes_.push_back(box);
  }
  
  ROS_WARN("[A*] setDynamicObstacleBoxes: set %zu obstacle boxes", dynamic_obstacle_boxes_.size());
}

bool AStarOccMap::checkDynamicObstacleCollision(const Eigen::Vector3d& pos) const
{
  std::lock_guard<std::mutex> lock(dynamic_obstacles_mutex_);
  
  for (const auto& box : dynamic_obstacle_boxes_) {
    // 计算障碍物方框的边界
    Eigen::Vector3d lower_bound = box.center - box.size / 2.0;
    Eigen::Vector3d upper_bound = box.center + box.size / 2.0;
    
    // 检查点是否在方框内
    if (pos(0) >= lower_bound(0) && pos(0) <= upper_bound(0) &&
        pos(1) >= lower_bound(1) && pos(1) <= upper_bound(1) &&
        pos(2) >= lower_bound(2) && pos(2) <= upper_bound(2)) {
      return true;  // 碰撞
    }
  }
  
  return false;  // 无碰撞
}

void AStarOccMap::makePlan(nav_msgs::Path &path)
{
  path.poses.clear();
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  
  if (!map_)
  {
    ROS_WARN("[A*] map is null.");
    return;
  }
  if (grid_res_ <= 0)
  {
    ROS_ERROR("[A*] grid_res_ is invalid (%.6f).", grid_res_);
    return;
  }
  
  ROS_INFO("[A*] makePlan: grid_res_=%.3f, grid_res_param_=%.3f, use26dir=%s", 
           grid_res_, grid_res_param_, use26dir_ ? "true" : "false");

  int sx, sy, sz, gx, gy, gz;
  Eigen::Vector3d s(start_.position.x, start_.position.y, start_.position.z);
  Eigen::Vector3d g(goal_.position.x, goal_.position.y, goal_.position.z);

  // 使用静态变量记录上次规划的参数，用于检测不一致
  static Eigen::Vector3d last_start = Eigen::Vector3d::Zero();
  static Eigen::Vector3d last_goal = Eigen::Vector3d::Zero();
  static int plan_count = 0;
  plan_count++;
  
  ROS_INFO("[A*] ========== START PLANNING #%d ==========", plan_count);
  ROS_INFO("[A*] Planning from start=(%.3f,%.3f,%.3f) to goal=(%.3f,%.3f,%.3f)",
           s.x(), s.y(), s.z(), g.x(), g.y(), g.z());
  ROS_INFO("[A*] grid_res_=%.3f, use26dir_=%s, map_origin=(%.3f,%.3f,%.3f)", 
           grid_res_, use26dir_ ? "true" : "false", 
           map_origin_.x(), map_origin_.y(), map_origin_.z());
  
  // 检查起点和终点是否与上次相同
  if (plan_count > 1) {
    double start_diff = (s - last_start).norm();
    double goal_diff = (g - last_goal).norm();
    if (start_diff < 0.01 && goal_diff < 0.01) {
      ROS_WARN("[A*] WARNING: Same start/goal as last planning! start_diff=%.6f, goal_diff=%.6f",
               start_diff, goal_diff);
    } else {
      ROS_INFO("[A*] Start/goal changed: start_diff=%.6f, goal_diff=%.6f", start_diff, goal_diff);
    }
  }
  last_start = s;
  last_goal = g;

  if (!posToIndex(s, sx, sy, sz) || !posToIndex(g, gx, gy, gz))
  {
    ROS_WARN("[A*] start or goal out of map. start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f)",
             s.x(), s.y(), s.z(), g.x(), g.y(), g.z());
    return;
  }
  
  // 检查起点和终点的坐标范围，防止溢出
  const int MAX_COORD = 100000;
  const int MIN_COORD = -100000;
  if (sx < MIN_COORD || sx > MAX_COORD || sy < MIN_COORD || sy > MAX_COORD || sz < MIN_COORD || sz > MAX_COORD ||
      gx < MIN_COORD || gx > MAX_COORD || gy < MIN_COORD || gy > MAX_COORD || gz < MIN_COORD || gz > MAX_COORD) {
    ROS_ERROR("[A*] Start or goal coordinates out of safe range! start=(%d,%d,%d) goal=(%d,%d,%d)",
              sx, sy, sz, gx, gy, gz);
    ROS_ERROR("[A*] Distance too large for A* planning. Consider using RRT or increasing map resolution.");
    return;
  }
  
  Eigen::Vector3d s_check = indexToPos(sx, sy, sz);
  Eigen::Vector3d g_check = indexToPos(gx, gy, gz);
  ROS_INFO("[A*] Grid indices: start=(%d,%d,%d) goal=(%d,%d,%d)", sx, sy, sz, gx, gy, gz);
  ROS_INFO("[A*] Converted back: start=(%.3f,%.3f,%.3f) goal=(%.3f,%.3f,%.3f)",
           s_check.x(), s_check.y(), s_check.z(), g_check.x(), g_check.y(), g_check.z());
  
  // 检查起点到终点的直线路径是否被阻挡
  double straight_dist = (g - s).norm();
  int num_checks = std::max(10, static_cast<int>(straight_dist / (grid_res_ * 0.5)));
  int blocked_count = 0;
  int not_in_map_count = 0;
  ROS_INFO("[A*] Checking straight line path (%d points, dist=%.3f m)...", num_checks, straight_dist);
  for (int i = 0; i <= num_checks; ++i) {
    double t = static_cast<double>(i) / num_checks;
    Eigen::Vector3d check_pos = s + t * (g - s);
    
    // 直接使用世界坐标查询地图（和MPC一样）
    if (!map_->isInMap(check_pos)) {
      not_in_map_count++;
      if (i == 0 || i == num_checks) {
        ROS_WARN("[A*] Straight line point at t=%.3f, pos=(%.3f,%.3f,%.3f) NOT IN MAP",
                 t, check_pos.x(), check_pos.y(), check_pos.z());
      }
      continue;
    }
    
    // 直接查询地图，不经过栅格转换
    if (map_->isInflatedOccupied(check_pos)) {
      ROS_WARN("[A*] Straight line BLOCKED at t=%.3f, pos=(%.3f,%.3f,%.3f)",
               t, check_pos.x(), check_pos.y(), check_pos.z());
      blocked_count++;
    }
    
    // 同时检查栅格转换后的查询
    int cx, cy, cz;
    if (posToIndex(check_pos, cx, cy, cz)) {
      Eigen::Vector3d grid_center = indexToPos(cx, cy, cz);
      bool grid_free = isFree(cx, cy, cz);
      bool direct_free = !map_->isInflatedOccupied(check_pos);
      if (grid_free != direct_free) {
        ROS_ERROR("[A*] MISMATCH at t=%.3f: direct_free=%s, grid_free=%s, pos=(%.3f,%.3f,%.3f), grid=(%d,%d,%d), grid_center=(%.3f,%.3f,%.3f)",
                  t, direct_free ? "true" : "false", grid_free ? "true" : "false",
                  check_pos.x(), check_pos.y(), check_pos.z(), cx, cy, cz,
                  grid_center.x(), grid_center.y(), grid_center.z());
      }
    }
  }
  ROS_INFO("[A*] Straight line check: %d/%d blocked, %d not_in_map, %d free",
           blocked_count, num_checks + 1, not_in_map_count, num_checks + 1 - blocked_count - not_in_map_count);
  
  if (blocked_count == 0 && straight_dist > 0.1) {
    ROS_WARN("[A*] WARNING: Straight line is FREE but A* may still plan a detour! This indicates a bug.");
  }
  
  if (!isFree(gx, gy, gz))
  {
    ROS_WARN("[A*] goal in obstacle. Trying to find nearest free cell...");
    const double search_radius = 2.0;
    const int max_search_cells = static_cast<int>(std::ceil(search_radius / grid_res_));
    bool found_free = false;
    int best_gx = gx, best_gy = gy, best_gz = gz;
    double min_dist = std::numeric_limits<double>::max();
    
    for (int r = 1; r <= max_search_cells && !found_free; ++r) {
      for (int dx = -r; dx <= r && !found_free; ++dx) {
        for (int dy = -r; dy <= r && !found_free; ++dy) {
          for (int dz = -r; dz <= r; ++dz) {
            int max_abs = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
            if (max_abs != r) continue;
            
            int nx = gx + dx;
            int ny = gy + dy;
            int nz = gz + dz;
            
            if (isFree(nx, ny, nz)) {
              double dist = std::sqrt(dx*dx + dy*dy + dz*dz) * grid_res_;
              if (dist < min_dist) {
                min_dist = dist;
                best_gx = nx;
                best_gy = ny;
                best_gz = nz;
                found_free = true;
              }
            }
          }
        }
      }
    }
    
    if (found_free) {
      gx = best_gx;
      gy = best_gy;
      gz = best_gz;
    } else {
      ROS_ERROR("[A*] Cannot find free cell near goal.");
      return;
    }
  }

  // 启发函数：欧几里得距离 + tie-breaking 因子
  // tie-breaking 因子让路径更倾向于直线，减少扭曲
  // 🔧 增大 tie-breaker，更强烈地偏好直线和对角线路径
  const double tie_breaker = 1.001;  // 0.1% 的提升，偏好直线路径
  auto heuristic = [&](int x, int y, int z) -> double {
    double dx = static_cast<double>(x - gx);
    double dy = static_cast<double>(y - gy);
    double dz = static_cast<double>(z - gz);
    double h = std::sqrt(dx * dx + dy * dy + dz * dz) * grid_res_;
    return h * tie_breaker;  // 添加 tie-breaking 因子
  };

  // 使用map存储节点信息，key是idx1d
  std::unordered_map<long long, Node> all_nodes;
  std::unordered_set<long long> closed_set;
  std::unordered_map<long long, double> gscore;
  
  // 优先队列：pair<f值, node_key>
  // 🔧 简化的tie-breaking：只比较f值，让启发式函数的tie-breaker发挥作用
  auto cmp = [&](const std::pair<double, long long> &a, const std::pair<double, long long> &b) {
    // 比较f值（由于启发式函数已经包含了tie-breaker，这里只需简单比较）
    if (std::abs(a.first - b.first) > 1e-12) {
      return a.first > b.first;  // f值小的优先
    }
    
    // f值几乎相同时，偏好g值大的节点（更接近目标）
    // 🔧 修复：这次真正实现 g-max（h-min）策略
    double g_a = gscore.count(a.second) ? gscore[a.second] : 0.0;
    double g_b = gscore.count(b.second) ? gscore[b.second] : 0.0;
    if (std::abs(g_a - g_b) > 1e-12) {
      return g_a < g_b;  // 返回true表示a优先级低，所以g大的(b)优先
    }
    
    // 最后的tie-breaker：使用key（避免坐标字典序带来的方向偏好）
    return a.second > b.second;
  };
  std::priority_queue<std::pair<double, long long>, 
                      std::vector<std::pair<double, long long>>, 
                      decltype(cmp)> open(cmp);

  // 6邻域
  const int dx6[6] = {1, -1, 0, 0, 0, 0};
  const int dy6[6] = {0, 0, 1, -1, 0, 0};
  const int dz6[6] = {0, 0, 0, 0, 1, -1};
  
  // 26邻域（简化版，确保没有重复）
  const int dx26[26] = {1,1,1, 1,1,1, 1,1,1,  0,0,0, 0,0,0, 0,0,  -1,-1,-1, -1,-1,-1, -1,-1,-1};
  const int dy26[26] = {1,1,1, 0,0,0, -1,-1,-1,  1,1,1, 0,0,0, -1,-1,  1,1,1, 0,0,0, -1,-1,-1};
  const int dz26[26] = {1,0,-1, 1,0,-1, 1,0,-1,  1,0,-1, 1,-1, 1,0,-1,  1,0,-1, 1,0,-1, 1,0,-1};

  const int *dx = use26dir_ ? dx26 : dx6;
  const int *dy = use26dir_ ? dy26 : dy6;
  const int *dz = use26dir_ ? dz26 : dz6;
  const int neigh = use26dir_ ? 26 : 6;

  // 初始化起点
  long long start_key = idx1d(sx, sy, sz);
  double start_h = heuristic(sx, sy, sz);
  Node startNode{sx, sy, sz, 0.0, start_h, -1};
  all_nodes[start_key] = startNode;
  gscore[start_key] = 0.0;
  open.push({startNode.f(), start_key});
  
  ROS_INFO("[A*] Start node: grid=(%d,%d,%d), g=0.0, h=%.3f, f=%.3f",
           sx, sy, sz, start_h, startNode.f());
  ROS_INFO("[A*] Goal node: grid=(%d,%d,%d), expected h=%.3f",
           gx, gy, gz, heuristic(gx, gy, gz));

  bool found = false;
  long long goal_key = -1;
  std::size_t expanded_nodes = 0;
  std::size_t tie_break_count = 0;
  
  while (!open.empty())
  {
    long long cur_key = open.top().second;
    double cur_f = open.top().first;
    open.pop();
    
    if (closed_set.count(cur_key)) {
      ROS_DEBUG_THROTTLE(0.1, "[A*] Node %lld already in closed_set, skipping", cur_key);
      continue;
    }
    closed_set.insert(cur_key);
    
    if (!all_nodes.count(cur_key)) {
      ROS_WARN_THROTTLE(1.0, "[A*] Node not found in all_nodes: %lld", cur_key);
      continue;
    }
    
    Node &cur = all_nodes[cur_key];
    
    // 每1000个节点打印一次进度
    if (expanded_nodes % 1000 == 0 && expanded_nodes > 0) {
      double dist_to_goal = std::sqrt((cur.x-gx)*(cur.x-gx) + (cur.y-gy)*(cur.y-gy) + (cur.z-gz)*(cur.z-gz)) * grid_res_;
      ROS_INFO("[A*] Progress: expanded=%zu, current=(%d,%d,%d), dist_to_goal=%.3f, f=%.3f",
               expanded_nodes, cur.x, cur.y, cur.z, dist_to_goal, cur_f);
    }
    
    if (cur.x == gx && cur.y == gy && cur.z == gz)
    {
      found = true;
      goal_key = cur_key;
      ROS_INFO("[A*] Goal reached! Expanded %zu nodes, tie-breaks=%zu", expanded_nodes, tie_break_count);
      break;
    }

    for (int k = 0; k < neigh; ++k)
    {
      int nx = cur.x + dx[k];
      int ny = cur.y + dy[k];
      int nz = cur.z + dz[k];

      // 检查坐标范围，防止溢出和越界
      // 使用更严格的限制，避免 idx1d 溢出
      const int MAX_COORD = 100000;
      const int MIN_COORD = -100000;
      if (nx < MIN_COORD || nx > MAX_COORD || 
          ny < MIN_COORD || ny > MAX_COORD || 
          nz < MIN_COORD || nz > MAX_COORD) {
        ROS_DEBUG_THROTTLE(0.5, "[A*] Neighbor (%d,%d,%d) out of range, skipping",
                           nx, ny, nz);
        continue;
      }

      // 先检查是否在地图范围内，再检查是否自由
      Eigen::Vector3d neighbor_pos = indexToPos(nx, ny, nz);
      if (!map_->isInMap(neighbor_pos)) {
        ROS_DEBUG_THROTTLE(0.5, "[A*] Neighbor (%d,%d,%d) -> (%.3f,%.3f,%.3f) not in map",
                           nx, ny, nz, neighbor_pos.x(), neighbor_pos.y(), neighbor_pos.z());
        continue;
      }

      if (!isFree(nx, ny, nz)) {
        ROS_DEBUG_THROTTLE(0.1, "[A*] Neighbor (%d,%d,%d) from (%d,%d,%d) is not free",
                           nx, ny, nz, cur.x, cur.y, cur.z);
        continue;
      }
      
      double step = std::sqrt(dx[k]*dx[k] + dy[k]*dy[k] + dz[k]*dz[k]) * grid_res_;
      
      // Step4-B: 计算风险代价
      double risk_cost = 0.0;
      
      // 🔧 新逻辑：优先使用RiskMap25D（如果可用）
      if (w_risk_ > 0.0)
      {
        if (use_risk_map_25d_ && risk_map_25d_ && risk_map_25d_->isValid())
        {
          // ✨ 使用RiskMap25D的3D查询（融合静态+动态风险）
          Eigen::Vector3d neighbor_world = indexToPos(nx, ny, nz);
          
          // 直接查询融合后的风险值
          double fused_risk = risk_map_25d_->query(neighbor_world);
          
          // RiskMap25D已经融合了静态和动态风险，直接作为代价
          // 不需要liftRiskGated和riskToCostLog
          risk_cost = w_risk_ * fused_risk;
        }
        else if (risk_map_ && risk_map_->isValid())
        {
          // 🔧 回退到旧版RiskMap2D逻辑
          Eigen::Vector3d neighbor_world = indexToPos(nx, ny, nz);
          double wx = neighbor_world(0);
          double wy = neighbor_world(1);
          double wz = neighbor_world(2);
          
          // 查询2D风险值（双线性插值）
          double p2d = risk_map_->queryBilinear(wx, wy);
          
          // 根据 z_gate 门限提升到3D风险
          double p3d = liftRiskGated(p2d, wz, z_gate_);
          
          // 转换为代价（使用二次方公式：cost = k * risk * risk）
          double risk_cost_raw = riskToCostLog(p3d, k_risk_);
          risk_cost = w_risk_ * risk_cost_raw;
          
          // 统计信息：记录高风险节点
          static int high_risk_count = 0;
          if (p3d > 0.1) {  // 风险值大于0.1
            high_risk_count++;
            if (high_risk_count <= 10) {  // 只打印前10个高风险节点
              ROS_WARN("[A*-RISK-HIGH] ⚠️  High risk node: nb(%d,%d,%d) -> world(%.3f,%.3f,%.3f), "
                       "p2d=%.4f, p3d=%.4f, risk_cost=%.4f (w_risk=%.1f)",
                       nx, ny, nz, wx, wy, wz, p2d, p3d, risk_cost, w_risk_);
            }
          }
          
          // Step4-A: 打印前20次扩展的风险值（用于验证）
          if (expanded_nodes < 20)
          {
            ROS_WARN("[A*-RISK] nb(%d,%d,%d) -> world(%.3f,%.3f,%.3f), p2d=%.6f, p3d=%.6f, risk_cost_raw=%.6f, risk_cost=%.6f",
                     nx, ny, nz, wx, wy, wz, p2d, p3d, risk_cost_raw, risk_cost);
            
            // 详细的双线性插值调试信息（前5次）
            if (expanded_nodes < 5)
            {
              // 手动计算栅格坐标用于调试
              double res = risk_map_->getResolution();
              Eigen::Vector2d origin = risk_map_->getOrigin();
              double gx = (wx - origin(0)) / res;
              double gy = (wy - origin(1)) / res;
              int x0 = static_cast<int>(std::floor(gx));
              int y0 = static_cast<int>(std::floor(gy));
              int x1 = x0 + 1;
              int y1 = y0 + 1;
              double fx = gx - static_cast<double>(x0);
              double fy = gy - static_cast<double>(y0);
              
              ROS_WARN("[A*-RISK-BILINEAR] world(%.3f,%.3f) -> grid(%.3f,%.3f), "
                       "corners: (%d,%d),(%d,%d),(%d,%d),(%d,%d), "
                       "fractions: fx=%.3f, fy=%.3f, result=%.6f",
                       wx, wy, gx, gy,
                       x0, y0, x1, y0, x0, y1, x1, y1,
                       fx, fy, p2d);
            }
          }
        }
        else
        {
          // 风险地图无效时的调试信息（前5次）
          if (expanded_nodes < 5)
          {
            ROS_WARN("[A*-RISK-DEBUG] ⚠️  Risk map invalid during expansion! expanded_nodes=%zu, neighbor=(%d,%d,%d)",
                     expanded_nodes, nx, ny, nz);
          }
        }
      }
      else
      {
        // 风险地图未设置或 w_risk_=0 时的调试信息（只打印一次）
        static bool warned_once = false;
        if (!warned_once && expanded_nodes == 0)
        {
          ROS_ERROR("[A*-RISK-DEBUG] ❌ Risk cost NOT used: w_risk_=%.3f, risk_map_=%p",
                   w_risk_, risk_map_.get());
          if (w_risk_ <= 0.0) {
            ROS_ERROR("[A*-RISK-DEBUG] ❌ w_risk_ is 0! Please set astar/w_risk > 0 in config file!");
          }
          if (!risk_map_) {
            ROS_ERROR("[A*-RISK-DEBUG] ❌ risk_map_ is NULL! Risk map not set!");
          }
          warned_once = true;
        }
      }
      
      // 总代价 = 移动代价 + 风险代价
      double tentative_g = cur.g + step + risk_cost;
      
      long long h = idx1d(nx, ny, nz);
      
      if (closed_set.count(h)) continue;
      
      bool is_new = !gscore.count(h);
      bool is_better = is_new || tentative_g < gscore[h];
      
      if (is_better)
      {
        double old_g = is_new ? std::numeric_limits<double>::max() : gscore[h];
        gscore[h] = tentative_g;
        double h_val = heuristic(nx, ny, nz);
        double new_f = tentative_g + h_val;
        
        // 检查是否是tie-breaking情况
        if (!is_new && std::abs(new_f - (old_g + h_val)) < 1e-9) {
          tie_break_count++;
        }
        
        Node nb{nx, ny, nz, tentative_g, h_val, cur_key};
        all_nodes[h] = nb;
        open.push({new_f, h});
        
        // 打印前几个扩展的节点，用于调试
        if (expanded_nodes < 10) {
          ROS_INFO("[A*] Expand [%zu]: (%d,%d,%d) -> (%d,%d,%d), g=%.3f->%.3f, h=%.3f, f=%.3f",
                   expanded_nodes, cur.x, cur.y, cur.z, nx, ny, nz, cur.g, tentative_g, h_val, new_f);
        }
        
        // 检查是否是tie-breaking情况（f值相同但g值不同）
        if (!is_new) {
          double old_h = heuristic(nx, ny, nz);
          double old_f = old_g + old_h;
          if (std::abs(new_f - old_f) < 1e-9 && std::abs(tentative_g - old_g) > 1e-9) {
            tie_break_count++;
            if (tie_break_count <= 10) {
              ROS_INFO("[A*] Tie-break #%zu: node=(%d,%d,%d), old_g=%.6f, new_g=%.6f, f=%.6f",
                       tie_break_count, nx, ny, nz, old_g, tentative_g, new_f);
            }
          }
        }
      }
    }

    ++expanded_nodes;
    if (expanded_nodes >= max_expanded_nodes_)
    {
      ROS_ERROR("[A*] Reached max_expanded_nodes_=%zu. Path may be too long or complex.", max_expanded_nodes_);
      ROS_ERROR("[A*] Consider: 1) Increasing max_expanded_nodes, 2) Using RRT for long distances, 3) Increasing grid resolution");
      break;
    }
    
    // 额外的安全检查：如果扩展节点数过多，可能是死循环或路径过长
    if (expanded_nodes > 100000 && expanded_nodes % 10000 == 0) {
      ROS_WARN("[A*] Large number of expanded nodes: %zu. This may indicate a very long path or planning difficulty.",
               expanded_nodes);
    }
  }
  
  if (!found)
  {
    ROS_WARN("[A*] No path found. Expanded %zu nodes.", expanded_nodes);
    return;
  }

  // 重建路径
  std::vector<geometry_msgs::PoseStamped> poses;
  long long trace = goal_key;
  std::unordered_set<long long> visited;
  int max_path_length = 100000;
  int path_count = 0;
  
  while (trace >= 0 && path_count < max_path_length)
  {
    if (visited.count(trace)) {
      ROS_ERROR("[A*] Cycle detected in path.");
      break;
    }
    visited.insert(trace);
    
    if (!all_nodes.count(trace)) {
      ROS_ERROR("[A*] Node not found in path reconstruction: %lld", trace);
      break;
    }
    
    const Node &n = all_nodes[trace];
    Eigen::Vector3d wp = indexToPos(n.x, n.y, n.z);
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = ros::Time::now();
    ps.pose.position.x = wp(0);
    ps.pose.position.y = wp(1);
    ps.pose.position.z = wp(2);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    poses.push_back(ps);
    trace = n.parent;
    path_count++;
  }
  std::reverse(poses.begin(), poses.end());

  // 注意：A*输出原始折线路径，不做平滑处理
  // 平滑由后续的polyTraj处理
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses = poses;
  
  // Step4-B: 计算并打印路径的风险统计
  ROS_WARN("[A*-RISK-DEBUG] Path statistics: w_risk_=%.3f, risk_map_=%p, poses.size()=%zu",
           w_risk_, risk_map_.get(), poses.size());
  
  if (w_risk_ > 0.0 && risk_map_ && !poses.empty())
  {
    ROS_WARN("[A*-RISK-DEBUG] Checking risk_map_->isValid()...");
    // 双重检查：在调用前再次检查有效性（防止并发修改）
    if (risk_map_->isValid())
    {
      ROS_WARN("[A*-RISK-DEBUG] Risk map is valid, computing path statistics...");
      double mean_risk = 0.0;
      double max_risk = 0.0;
      int valid_points = 0;
      int zero_risk_points = 0;
      int out_of_map_points = 0;
      
      // 获取风险地图参数用于调试
      double res = risk_map_->getResolution();
      Eigen::Vector2d origin = risk_map_->getOrigin();
      Eigen::Vector2i size = risk_map_->getSize();
      ROS_WARN("[A*-RISK-DEBUG] Risk map params: resolution=%.3f, size=(%d,%d), origin=(%.2f,%.2f)",
               res, size(0), size(1), origin(0), origin(1));
      
      for (size_t i = 0; i < poses.size(); ++i)
      {
        const auto& pose = poses[i];
        double wx = pose.pose.position.x;
        double wy = pose.pose.position.y;
        double wz = pose.pose.position.z;
        
        // 检查坐标是否在地图范围内
        double gx = (wx - origin(0)) / res;
        double gy = (wy - origin(1)) / res;
        bool in_map = (gx >= 0 && gx < size(0) && gy >= 0 && gy < size(1));
        
        if (!in_map)
        {
          out_of_map_points++;
          if (i < 3 || i >= poses.size() - 3)
          {
            ROS_WARN("[A*-RISK-DEBUG] Path point %zu out of map: world(%.3f,%.3f,%.3f) -> grid(%.3f,%.3f)",
                     i, wx, wy, wz, gx, gy);
          }
          continue;
        }
        
        double p2d = risk_map_->queryBilinear(wx, wy);
        double p3d = liftRiskGated(p2d, wz, z_gate_);
        
        // 打印前3个和后3个点的详细信息
        if (i < 3 || i >= poses.size() - 3)
        {
          ROS_WARN("[A*-RISK-DEBUG] Path point %zu: world(%.3f,%.3f,%.3f) -> grid(%.3f,%.3f), "
                   "p2d=%.6f, p3d=%.6f, z_gate=%.2f",
                   i, wx, wy, wz, gx, gy, p2d, p3d, z_gate_);
        }
        
        mean_risk += p3d;
        if (p3d > max_risk) max_risk = p3d;
        if (p3d < 1e-6) zero_risk_points++;
        valid_points++;
      }
      
      if (valid_points > 0)
      {
        mean_risk /= valid_points;
        ROS_WARN("[A*-RISK] path_len=%zu mean_risk=%.6f max_risk=%.6f w_risk=%.1f k_risk=%.1f z_gate=%.2f",
                 poses.size(), mean_risk, max_risk, w_risk_, k_risk_, z_gate_);
        ROS_WARN("[A*-RISK-DEBUG] Path stats details: valid_points=%d, zero_risk_points=%d, out_of_map_points=%d",
                 valid_points, zero_risk_points, out_of_map_points);
      }
      else
      {
        ROS_WARN("[A*-RISK-DEBUG] No valid points for risk calculation! out_of_map_points=%d",
                 out_of_map_points);
      }
    }
    else
    {
      ROS_WARN("[A*-RISK-DEBUG] Risk map is INVALID during path statistics calculation!");
    }
  }
  else
  {
    if (w_risk_ <= 0.0)
    {
      ROS_WARN("[A*-RISK-DEBUG] Risk cost not used: w_risk_=%.3f <= 0", w_risk_);
    }
    if (!risk_map_)
    {
      ROS_WARN("[A*-RISK-DEBUG] Risk cost not used: risk_map_ is NULL");
    }
    if (poses.empty())
    {
      ROS_WARN("[A*-RISK-DEBUG] Risk cost not used: poses is empty");
    }
  }
  
  // 发布A*原始路径到专用话题
  pathPub_.publish(path);
  ROS_INFO("[A*] Published raw path (%zu waypoints) to /astar/planned_path", poses.size());
  
  ROS_INFO("[A*] Path found! Expanded %zu nodes, path has %zu waypoints.", expanded_nodes, poses.size());
  
  // 打印完整路径，用于调试路径形状
  ROS_INFO("[A*] ========== FINAL PATH (%zu waypoints) ==========", poses.size());
  if (poses.size() > 0) {
    // 打印所有路径点（如果不超过30个），否则打印前15个和后15个
    size_t print_limit = 30;
    if (poses.size() <= print_limit) {
      for (size_t i = 0; i < poses.size(); ++i) {
        ROS_INFO("[A*]   [%zu] (%.3f, %.3f, %.3f)", i,
                 poses[i].pose.position.x, poses[i].pose.position.y, poses[i].pose.position.z);
      }
    } else {
      ROS_INFO("[A*] Path too long, printing first 15 and last 15:");
      for (size_t i = 0; i < 15; ++i) {
        ROS_INFO("[A*]   [%zu] (%.3f, %.3f, %.3f)", i,
                 poses[i].pose.position.x, poses[i].pose.position.y, poses[i].pose.position.z);
      }
      ROS_INFO("[A*]   ... (%zu points) ...", poses.size() - 30);
      for (size_t i = poses.size() - 15; i < poses.size(); ++i) {
        ROS_INFO("[A*]   [%zu] (%.3f, %.3f, %.3f)", i,
                 poses[i].pose.position.x, poses[i].pose.position.y, poses[i].pose.position.z);
      }
    }
    
    // 计算路径的转向角度，检测S形路径
    if (poses.size() >= 3) {
      int left_turns = 0, right_turns = 0;
      double max_curvature = 0.0;
      for (size_t i = 1; i < poses.size() - 1; ++i) {
        Eigen::Vector3d v1(poses[i].pose.position.x - poses[i-1].pose.position.x,
                           poses[i].pose.position.y - poses[i-1].pose.position.y,
                           0);
        Eigen::Vector3d v2(poses[i+1].pose.position.x - poses[i].pose.position.x,
                           poses[i+1].pose.position.y - poses[i].pose.position.y,
                           0);
        if (v1.norm() > 1e-6 && v2.norm() > 1e-6) {
          double cross_z = v1.x() * v2.y() - v1.y() * v2.x();
          double curvature = std::abs(cross_z) / (v1.norm() * v2.norm());
          max_curvature = std::max(max_curvature, curvature);
          if (cross_z > 0.1) left_turns++;
          else if (cross_z < -0.1) right_turns++;
        }
      }
      ROS_INFO("[A*] Path curvature: %d left turns, %d right turns, max_curvature=%.6f (S-shape indicator)", 
               left_turns, right_turns, max_curvature);
      
      // 如果路径有明显的S形（左右转向交替），发出警告
      if (left_turns > 2 && right_turns > 2) {
        ROS_WARN("[A*] WARNING: Path has S-shape pattern! This suggests tie-breaking instability or map query issues.");
      }
    }
    
    // 使用静态变量记录上次路径，用于检测路径不一致
    static std::vector<Eigen::Vector3d> last_path;
    static int path_check_count = 0;
    path_check_count++;
    
    std::vector<Eigen::Vector3d> current_path;
    for (const auto &p : poses) {
      current_path.push_back(Eigen::Vector3d(p.pose.position.x, p.pose.position.y, p.pose.position.z));
    }
    
    if (path_check_count > 1 && last_path.size() == current_path.size()) {
      double path_diff = 0.0;
      for (size_t i = 0; i < current_path.size(); ++i) {
        path_diff += (current_path[i] - last_path[i]).norm();
      }
      path_diff /= current_path.size();
      if (path_diff > 0.1) {
        ROS_ERROR("[A*] ERROR: Path inconsistency detected! Average point difference=%.6f m", path_diff);
        ROS_ERROR("[A*] This indicates non-deterministic behavior. Check tie-breaking and map queries.");
      } else {
        ROS_INFO("[A*] Path consistency check: average point difference=%.6f m (OK)", path_diff);
      }
    }
    last_path = current_path;
  }
  ROS_INFO("[A*] =================================================");
  
  // 检查路径质量：计算路径总长度
  if (poses.size() > 1) {
    double path_length = 0.0;
    for (size_t i = 1; i < poses.size(); ++i) {
      double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
      double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
      double dz = poses[i].pose.position.z - poses[i-1].pose.position.z;
      path_length += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    double straight_dist = std::sqrt(
      (poses.back().pose.position.x - poses[0].pose.position.x) * 
      (poses.back().pose.position.x - poses[0].pose.position.x) +
      (poses.back().pose.position.y - poses[0].pose.position.y) * 
      (poses.back().pose.position.y - poses[0].pose.position.y) +
      (poses.back().pose.position.z - poses[0].pose.position.z) * 
      (poses.back().pose.position.z - poses[0].pose.position.z)
    );
    ROS_INFO_THROTTLE(1.0, "[A*] Path quality: length=%.3f m, straight=%.3f m, ratio=%.3f",
                      path_length, straight_dist, path_length / (straight_dist + 1e-6));
  }
}

}  // namespace globalPlanner
