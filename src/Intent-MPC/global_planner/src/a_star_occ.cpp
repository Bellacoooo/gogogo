#include <global_planner/a_star_occ.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

namespace globalPlanner
{

AStarOccMap::AStarOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  // 注意：一定要给 param 一个确定的常量默认值，避免读取未初始化成员
  nh_.param("astar/use_26_dir",    use26dir_,       true);
  nh_.param("astar/avg_velocity",  avg_velocity_,   1.5);
  nh_.param("astar/w1_dist",       w1_dist_,        1.0);
  nh_.param("astar/w2_static",     w2_static_,      0.0);
  nh_.param("astar/w3_dynamic",     w3_dynamic_,    0.0);
  nh_.param("astar/local_range_xy", local_range_xy_, 12.0);

  // 注意：ROS 的 param 模板不支持 size_t，这里先读成 int 再赋值
  int max_nodes_tmp = 300000;
  nh_.param("astar/max_expanded_nodes", max_nodes_tmp, max_nodes_tmp);
  // 至少保证一个合理的下限，防止设置过小导致路径搜索很快就早停
  max_expanded_nodes_ = static_cast<std::size_t>(std::max(1000, max_nodes_tmp));

  // A* 内部虚拟栅格分辨率（单位：米），若 <=0 则退化为使用占据地图分辨率
  nh_.param("astar/grid_resolution", grid_res_param_, 0.0);

  // 订阅动态风险地图（来自 dynamic_predictor）
  risk_map_sub_ = nh_.subscribe(
      "/dynamic_predictor/dynamic_risk_map",
      1,
      &AStarOccMap::riskMapCallback,
      this);
  ROS_INFO("[A*] Subscribed to /dynamic_predictor/dynamic_risk_map for dynamic risk field.");
  
  // 订阅高度地图
  risk_height_map_sub_ = nh_.subscribe(
      "/dynamic_predictor/dynamic_risk_height_map",
      1,
      &AStarOccMap::riskHeightMapCallback,
      this);
  ROS_INFO("[A*] Subscribed to /dynamic_predictor/dynamic_risk_height_map for obstacle height information.");
}

void AStarOccMap::setMap(const std::shared_ptr<mapManager::occMap> &map)
{
  map_ = map;
  if (map_)
  {
    res_ = map_->getRes();
    // 若未指定单独的 A* 分辨率，则使用占据地图分辨率
    if (grid_res_param_ <= 1e-6)
    {
      grid_res_ = res_;
    }
    else
    {
      grid_res_ = grid_res_param_;
      // 为避免“比真实地图还细”的情况，若用户设置过小则向上取占据地图分辨率
      if (grid_res_ < res_)
      {
        ROS_WARN("[A*] astar/grid_resolution=%.3f is finer than map resolution %.3f, clamp to map resolution.",
                 grid_res_, res_);
        grid_res_ = res_;
      }
    }
    ROS_INFO("[A*] Using grid resolution %.3f m (map res=%.3f m).", grid_res_, res_);
  }
}

void AStarOccMap::updateStart(const geometry_msgs::Pose &start)
{
  start_ = start;
  start_pos_ = Eigen::Vector3d(start.position.x, start.position.y, start.position.z);
}

void AStarOccMap::updateGoal(const geometry_msgs::Pose &goal)
{
  goal_ = goal;
}

bool AStarOccMap::posToIndex(const Eigen::Vector3d &pos, int &ix, int &iy, int &iz) const
{
  if (!map_) return false;
  // 先用占据地图检查位置是否在地图内
  if (!map_->isInMap(pos)) return false;

  // 在 A* 内部使用独立的“虚拟粗栅格”坐标系
  ix = static_cast<int>(std::floor(pos.x() / grid_res_));
  iy = static_cast<int>(std::floor(pos.y() / grid_res_));
  iz = static_cast<int>(std::floor(pos.z() / grid_res_));
  return true;
}

Eigen::Vector3d AStarOccMap::indexToPos(int ix, int iy, int iz) const
{
  // 将虚拟粗栅格索引转换为世界坐标（使用格子中心点）
  Eigen::Vector3d pos;
  pos.x() = (static_cast<double>(ix) + 0.5) * grid_res_;
  pos.y() = (static_cast<double>(iy) + 0.5) * grid_res_;
  pos.z() = (static_cast<double>(iz) + 0.5) * grid_res_;
  return pos;
}

bool AStarOccMap::isFree(int ix, int iy, int iz) const
{
  if (!map_) return false;
  Eigen::Vector3d center = indexToPos(ix, iy, iz);
  // 超出真实占据地图范围的粗格子视为不可通行
  if (!map_->isInMap(center)) return false;
  // 使用细分占据图+膨胀结果进行碰撞检测
  return !map_->isInflatedOccupied(center);
}

void AStarOccMap::setDynamicPredictions(const std::vector<DynObstaclePred> &preds)
{
  dyn_preds_ = preds;
}

double AStarOccMap::estimateArrivalTime(const Eigen::Vector3d &pos) const
{
  double dist = (pos - start_pos_).norm();
  if (avg_velocity_ < 1e-6) return std::numeric_limits<double>::infinity();
  return dist / avg_velocity_;
}

double AStarOccMap::calculateDynamicCost(const Eigen::Vector3d &pos) const
{
  // 默认不使用动态风险时，直接返回 0
  if (w3_dynamic_ <= 1e-6)
  {
    return 0.0;
  }

  // 使用查询函数，从风险图中获取当前位置的风险值 [0,100]，再归一化
  // 传入高度信息，如果查询点高度 > 障碍物高度，返回 0
  double occ = getDynamicRisk(pos.x(), pos.y(), pos.z());
  double cost = (occ > 0.0) ? occ / 100.0 : 0.0;
  // 提升小风险的惩罚强度，避免风险值过低时几乎不起作用
  if (cost > 0.0 && cost < 0.2)
  {
    cost = 0.2; // 至少 0.2，再由 w3_dynamic_ 放大
  }
  
  // 调试：打印所有非零风险采样，便于确认采样是否命中
  if (occ > 0.0)
  {
    ROS_INFO_THROTTLE(0.5, "[A*] dyn sample at (%.2f,%.2f): occ=%.1f, cost=%.3f, w3*cost=%.3f",
                      pos.x(), pos.y(), occ, cost, w3_dynamic_ * cost);
  }
  
  return cost;
}

void AStarOccMap::riskMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(risk_map_mutex_);
  latest_risk_map_ = msg;
  ROS_INFO_THROTTLE(1.0, "[A*] Received new dynamic risk map (w=%u,h=%u).",
                    msg->info.width, msg->info.height);
}

void AStarOccMap::riskHeightMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  std::lock_guard<std::mutex> lock(height_map_mutex_);
  latest_height_map_ = msg;
  ROS_INFO_THROTTLE(1.0, "[A*] Received new height map (w=%u,h=%u).",
                    msg->info.width, msg->info.height);
}

double AStarOccMap::getDynamicRisk(double world_x, double world_y, double world_z) const
{
  std::lock_guard<std::mutex> lock(risk_map_mutex_);
  if (!latest_risk_map_)
  {
    ROS_WARN_THROTTLE(2.0, "[A*] getDynamicRisk: latest_risk_map_ is NULL!");
    return 0.0;
  }

  const auto &info = latest_risk_map_->info;
  const double res = info.resolution;
  const int width  = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  if (width <= 0 || height <= 0 || res <= 0.0)
  {
    return 0.0;
  }

  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;

  // 使用 round 而非 floor，可减小半格偏移导致的取样失败
  int grid_x = static_cast<int>(std::round((world_x - ox) / res));
  int grid_y = static_cast<int>(std::round((world_y - oy) / res));

  if (grid_x < 0 || grid_x >= width || grid_y < 0 || grid_y >= height)
  {
    ROS_DEBUG_THROTTLE(2.0, "[A*] getDynamicRisk: (%.2f,%.2f) out of map bounds [%d,%d]x[%d,%d], origin=(%.2f,%.2f)",
                       world_x, world_y, 0, width-1, 0, height-1, ox, oy);
    return 0.0;  // 地图外，认为无风险
  }

  std::size_t index = static_cast<std::size_t>(grid_y) * static_cast<std::size_t>(width)
                    + static_cast<std::size_t>(grid_x);
  if (index >= latest_risk_map_->data.size())
  {
  return 0.0;
  }

  // 检查高度：如果查询点高度 > 障碍物高度，返回 0
  {
    std::lock_guard<std::mutex> height_lock(height_map_mutex_);
    if (latest_height_map_ && index < latest_height_map_->data.size()) {
      // 高度地图存储的是 0-100，表示 0-10 米（分辨率 0.1 米）
      const double height_scale = 10.0;
      int8_t height_val = latest_height_map_->data[index];
      double obstacle_height = (static_cast<double>(height_val) / 100.0) * height_scale;
      
      // 如果查询点高度 > 障碍物高度，返回 0（无风险）
      if (world_z > obstacle_height + 0.1) {  // 加 0.1m 容差
        ROS_DEBUG_THROTTLE(2.0, "[A*] getDynamicRisk: query z=%.2f > obstacle height=%.2f, returning 0",
                           world_z, obstacle_height);
        return 0.0;
      }
    }
  }

  int8_t occ = latest_risk_map_->data[index];
  // 调试：在此打印一次 origin/res，避免多处查找
  ROS_DEBUG_THROTTLE(1.0, "[A*] getDynamicRisk origin=(%.2f,%.2f) res=%.3f query=(%.2f,%.2f,%.2f)->(%d,%d) occ=%.1f",
                     ox, oy, res, world_x, world_y, world_z, grid_x, grid_y, static_cast<double>(occ));
  return static_cast<double>(occ);  // 直接返回 0~100
}

void AStarOccMap::makePlan(nav_msgs::Path &path)
{
  ROS_INFO("[A*] makePlan() called. w1=%.2f, w2=%.2f, w3=%.2f", 
           w1_dist_, w2_static_, w3_dynamic_);
  ROS_INFO("[A*] use26dir=%s, avg_velocity=%.2f, local_range_xy=%.2f, "
           "grid_res=%.3f (param=%.3f), max_expanded_nodes=%zu",
           use26dir_ ? "true" : "false",
           avg_velocity_,
           local_range_xy_,
           grid_res_,
           grid_res_param_,
           max_expanded_nodes_);
  path.poses.clear();
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  if (!map_)
  {
    ROS_WARN("[A*] map is null.");
    return;
  }

  int sx, sy, sz, gx, gy, gz;
  Eigen::Vector3d s(start_.position.x, start_.position.y, start_.position.z);
  Eigen::Vector3d g_full(goal_.position.x, goal_.position.y, goal_.position.z);
  
  // 调试：打印起点风险值
  double start_risk = getDynamicRisk(s.x(), s.y(), s.z());
  ROS_INFO_THROTTLE(1.0, "[A*] Risk at start (%.2f, %.2f, %.2f) = %.1f, latest_risk_map_=%s",
                    s.x(), s.y(), s.z(), start_risk, 
                    latest_risk_map_ ? "OK" : "NULL");


  // 计算从当前起点指向全局终点的方向，并根据最大段长生成局部目标点
  const double max_segment = 10.0;  // 局部 A* 规划的最大直线距离（米）
  Eigen::Vector3d dir = g_full - s;
  double dist_full = dir.norm();
  Eigen::Vector3d g_target;
  if (dist_full > max_segment && dist_full > 1e-6)
  {
    dir.normalize();
    g_target = s + dir * max_segment;
    ROS_INFO("[A*] Using local target (segment %.2f m of %.2f m).", max_segment, dist_full);
  }
  else
  {
    g_target = g_full;
    ROS_INFO("[A*] Using full goal (distance %.2f m).", dist_full);
  }

  if (!posToIndex(s, sx, sy, sz) || !posToIndex(g_target, gx, gy, gz))
  {
    ROS_WARN("[A*] start or goal out of map. start=(%.2f,%.2f,%.2f) goal=(%.2f,%.2f,%.2f)",
             s.x(), s.y(), s.z(), g_target.x(), g_target.y(), g_target.z());
    return;
  }
  if (!isFree(gx, gy, gz))
  {
    ROS_WARN("[A*] goal in obstacle at index (%d,%d,%d). Trying to find nearest free cell...", gx, gy, gz);
    // 尝试在目标点周围寻找最近的可通行位置（BFS搜索，最多搜索 radius 米范围）
    const double search_radius = 2.0; // 搜索半径（米）
    const int max_search_cells = static_cast<int>(std::ceil(search_radius / grid_res_));
    bool found_free = false;
    int best_gx = gx, best_gy = gy, best_gz = gz;
    double min_dist = std::numeric_limits<double>::max();
    
    // 从目标点开始，逐层向外搜索（BFS）
    for (int r = 1; r <= max_search_cells && !found_free; ++r) {
      for (int dx = -r; dx <= r && !found_free; ++dx) {
        for (int dy = -r; dy <= r && !found_free; ++dy) {
          for (int dz = -r; dz <= r; ++dz) {
            // 只检查在当前"层"的边界上的点（L∞范数等于r，即max(|dx|,|dy|,|dz|)==r）
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
                found_free = true; // 找到第一个可通行点就停止（最近的就是第一个）
              }
            }
          }
        }
      }
    }
    
    if (found_free) {
      ROS_INFO("[A*] Adjusted goal from (%d,%d,%d) to nearest free cell (%d,%d,%d), distance %.2f m.",
               gx, gy, gz, best_gx, best_gy, best_gz, min_dist);
      gx = best_gx;
      gy = best_gy;
      gz = best_gz;
      // 更新 g_target 的世界坐标
      g_target = indexToPos(gx, gy, gz);
    } else {
      ROS_ERROR("[A*] Cannot find free cell near goal within %.2f m radius. Aborting path planning.", search_radius);
    return;
    }
  }

  // 启发函数：与 g 中的距离代价保持同一尺度（米 × w1_dist_），乘 1.001 打破平局，避免偏曼哈顿
  auto heuristic = [&](int x, int y, int z) {
    double dx = static_cast<double>(x - gx);
    double dy = static_cast<double>(y - gy);
    double dz = static_cast<double>(z - gz);
    double dist_m = std::sqrt(dx * dx + dy * dy + dz * dz) * grid_res_;
    return w1_dist_ * dist_m * 1.001;
  };

  std::vector<Node> closed;
  auto cmp = [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; };
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> open(cmp);
  std::unordered_map<long long, double> gscore;
  std::unordered_map<long long, double> gdist; // 纯距离累计

  Node startNode{sx, sy, sz, 0.0, heuristic(sx, sy, sz), -1};
  closed.push_back(startNode);
  open.push({startNode.f(), 0});
  gscore[idx1d(sx, sy, sz)] = 0.0;
  gdist[idx1d(sx, sy, sz)] = 0.0;

  // 基于起点构造局部重规划窗口（以起点为中心，local_range_xy_ 米的正方形区域）
  const int half_range_xy_cells = static_cast<int>(std::ceil(0.5 * local_range_xy_ / grid_res_));
  const int xmin_win = sx - half_range_xy_cells;
  const int xmax_win = sx + half_range_xy_cells;
  const int ymin_win = sy - half_range_xy_cells;
  const int ymax_win = sy + half_range_xy_cells;
  ROS_INFO("[A*] Local window (soft constraint): x[%d,%d], y[%d,%d], start_idx=(%d,%d,%d), goal_idx=(%d,%d,%d).",
           xmin_win, xmax_win, ymin_win, ymax_win, sx, sy, sz, gx, gy, gz);

  // 26邻域（含6/18/26），或仅6邻域
  // 26-neighborhood (exclude 0,0,0)
  // 26-neighborhood (6 faces + 12 edges + 8 corners), excluding (0,0,0)
  const int dx26[26] = {  1, 1, 1,  1, 1, 1,  1, 1, 1,
                          0, 0, 0,  0, 0, 0,  0, 0,
                         -1,-1,-1, -1,-1,-1, -1,-1,-1 };
  const int dy26[26] = {  1, 1, 1,  0, 0, 0, -1,-1,-1,
                          1, 1, 1,  0, 0, 0, -1,-1,
                          1, 1, 1,  0, 0, 0, -1,-1,-1 };
  const int dz26[26] = {  1, 0,-1,  1, 0,-1,  1, 0,-1,
                          1, 0,-1,  1,-1, 1,  0,-1,
                          1, 0,-1,  1, 0,-1,  1, 0,-1 };
  const int dx6[6]   = {1,-1,0,0,0,0};
  const int dy6[6]   = {0,0,1,-1,0,0};
  const int dz6[6]   = {0,0,0,0,1,-1};

  bool found = false;
  int goalIdx = -1;
  std::size_t expanded_nodes = 0;

  while (!open.empty())
  {
    int curIdx = open.top().second;
    open.pop();
    const Node &cur = closed[curIdx];
    if (cur.x == gx && cur.y == gy && cur.z == gz)
    {
      found = true;
      goalIdx = curIdx;
      break;
    }

    const int *dx = use26dir_ ? dx26 : dx6;
    const int *dy = use26dir_ ? dy26 : dy6;
    const int *dz = use26dir_ ? dz26 : dz6;
    int neigh = use26dir_ ? 26 : 6;
    for (int k = 0; k < neigh; ++k)
    {
      int nx = cur.x + dx[k];
      int ny = cur.y + dy[k];
      int nz = cur.z + dz[k];

      if (!isFree(nx, ny, nz)) continue;
      // 在虚拟粗栅格中，一步的物理距离与 grid_res_ 成正比
      double step = std::sqrt(dx[k]*dx[k] + dy[k]*dy[k] + dz[k]*dz[k]) * grid_res_;
      // 计算代价分解
      Eigen::Vector3d nbPos = indexToPos(nx, ny, nz);
      double cost_dist = step;
      double cost_static = 0.0; // 预留：若加入距离场，可在此填写
      double cost_dynamic = calculateDynamicCost(nbPos);
      double move_cost = w1_dist_ * cost_dist + w2_static_ * cost_static + w3_dynamic_ * cost_dynamic;

      double tentative = cur.g + move_cost;
      double tentative_dist = gdist[idx1d(cur.x, cur.y, cur.z)] + step;
      long long h = idx1d(nx, ny, nz);
      if (!gscore.count(h) || tentative < gscore[h])
      {
        gscore[h] = tentative;
        gdist[h] = tentative_dist;
        Node nb{nx, ny, nz, tentative, heuristic(nx, ny, nz), curIdx};
        int newIdx = closed.size();
        closed.push_back(nb);
        open.push({nb.f(), newIdx});
      }
    }

    ++expanded_nodes;
    // 安全上限：避免在复杂环境中无限膨胀导致崩溃
    if (expanded_nodes >= max_expanded_nodes_)
    {
      ROS_ERROR("[A*] Reached max_expanded_nodes_=%zu, aborting search to avoid overload.", max_expanded_nodes_);
      break;
    }
    if (expanded_nodes % 50000 == 0)
    {
      ROS_INFO("[A*] Expanded %zu nodes so far...", expanded_nodes);
    }
  }

  if (!found)
  {
    ROS_WARN("[A*] no path found within local window. Expanded %zu nodes.", expanded_nodes);
    return;
  }

  ROS_INFO("[A*] Search finished. Expanded %zu nodes.", expanded_nodes);

  std::vector<geometry_msgs::PoseStamped> poses;
  int trace = goalIdx;
  while (trace >= 0)
  {
    const Node &n = closed[trace];
    Eigen::Vector3d wp = indexToPos(n.x, n.y, n.z);
    geometry_msgs::PoseStamped ps;
    // 使用 goal 的 frame，如果为空则使用 "map"
    ps.header.frame_id = "map";
    ps.pose.position.x = wp(0);
    ps.pose.position.y = wp(1);
    ps.pose.position.z = wp(2);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    poses.push_back(ps);
    trace = n.parent;
  }
  std::reverse(poses.begin(), poses.end());

  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses = poses;
}

}  // namespace globalPlanner

