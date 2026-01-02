#include <global_planner/a_star_occ.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <limits>

namespace globalPlanner
{

AStarOccMap::AStarOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  nh_.param("astar/use_26_dir", use26dir_, false);  // 默认使用6邻域，更稳定
  
  int max_nodes_tmp = 300000;
  nh_.param("astar/max_expanded_nodes", max_nodes_tmp, max_nodes_tmp);
  max_expanded_nodes_ = static_cast<std::size_t>(std::max(1000, max_nodes_tmp));

  nh_.param("astar/grid_resolution", grid_res_param_, 0.5);
  
  // 初始化A*专用路径发布器
  pathPub_ = nh_.advertise<nav_msgs::Path>("astar/planned_path", 10);
  
  ROS_INFO("[A*] Simple 3D A* initialized (grid_res=%.2f m, use26dir=%s).", 
           grid_res_param_, use26dir_ ? "true" : "false");
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
  
  // 将栅格索引转为世界坐标（格子中心）
  Eigen::Vector3d center = indexToPos(ix, iy, iz);
  
  // 直接使用世界坐标查询地图，和MPC一样
  if (!map_->isInMap(center)) {
    ROS_DEBUG_THROTTLE(0.5, "[A*] isFree: grid=(%d,%d,%d) -> world=(%.3f,%.3f,%.3f) NOT IN MAP",
                       ix, iy, iz, center.x(), center.y(), center.z());
    return false;
  }
  bool occupied = map_->isInflatedOccupied(center);
  if (occupied) {
    ROS_DEBUG_THROTTLE(0.5, "[A*] isFree: grid=(%d,%d,%d) -> world=(%.3f,%.3f,%.3f) OCCUPIED",
                       ix, iy, iz, center.x(), center.y(), center.z());
  }
  return !occupied;
}

void AStarOccMap::setDynamicPredictions(const std::vector<DynObstaclePred> &preds)
{
  // 不使用
}

double AStarOccMap::calculateDynamicCost(const Eigen::Vector3d &pos) const
{
  return 0.0;
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

  // 简化的启发函数：只使用欧几里得距离，tie-breaking由优先队列处理
  // 移除cross-product项，因为它可能导致数值不稳定和路径不一致
  auto heuristic = [&](int x, int y, int z) -> double {
    double dx = static_cast<double>(x - gx);
    double dy = static_cast<double>(y - gy);
    double dz = static_cast<double>(z - gz);
    return std::sqrt(dx * dx + dy * dy + dz * dz) * grid_res_;
  };

  // 使用map存储节点信息，key是idx1d
  std::unordered_map<long long, Node> all_nodes;
  std::unordered_set<long long> closed_set;
  std::unordered_map<long long, double> gscore;
  
  // 优先队列：pair<f值, node_key>
  // 强化的tie-breaking：确保完全确定性，避免路径抖动
  auto cmp = [&](const std::pair<double, long long> &a, const std::pair<double, long long> &b) {
    // 首先比较f值
    if (std::abs(a.first - b.first) > 1e-9) {
      return a.first > b.first;  // f值小的优先
    }
    
    // f值相同时，使用g-max策略（偏好更接近起点的节点，即直线路径）
    double g_a = gscore.count(a.second) ? gscore[a.second] : 0.0;
    double g_b = gscore.count(b.second) ? gscore[b.second] : 0.0;
    if (std::abs(g_a - g_b) > 1e-9) {
      return g_a < g_b;  // g值大的优先（更接近起点，更接近直线）
    }
    
    // g值也相同时，使用节点坐标作为最后的tie-breaker（确保完全确定性）
    if (all_nodes.count(a.second) && all_nodes.count(b.second)) {
      const Node &na = all_nodes[a.second];
      const Node &nb = all_nodes[b.second];
      // 按字典序比较坐标，确保完全确定性
      if (na.x != nb.x) return na.x > nb.x;
      if (na.y != nb.y) return na.y > nb.y;
      return na.z > nb.z;
    }
    
    // 如果节点不存在，使用key作为最后的tie-breaker
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

      // 检查坐标范围，防止溢出
      if (nx < -99999 || nx > 99999 || ny < -99999 || ny > 99999 || nz < -99999 || nz > 99999) {
        continue;
      }

      if (!isFree(nx, ny, nz)) {
        ROS_DEBUG_THROTTLE(0.1, "[A*] Neighbor (%d,%d,%d) from (%d,%d,%d) is not free",
                           nx, ny, nz, cur.x, cur.y, cur.z);
        continue;
      }
      
      double step = std::sqrt(dx[k]*dx[k] + dy[k]*dy[k] + dz[k]*dz[k]) * grid_res_;
      double tentative_g = cur.g + step;
      
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
      ROS_ERROR("[A*] Reached max_expanded_nodes_=%zu.", max_expanded_nodes_);
      break;
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
