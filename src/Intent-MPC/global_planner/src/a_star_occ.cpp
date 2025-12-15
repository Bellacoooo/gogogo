#include <global_planner/a_star_occ.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

namespace globalPlanner
{

AStarOccMap::AStarOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  nh_.param("astar/use_26_dir", use26dir_, use26dir_);
  nh_.param("astar/avg_velocity", avg_velocity_, avg_velocity_);
  nh_.param("astar/w1_dist", w1_dist_, w1_dist_);
  nh_.param("astar/w2_static", w2_static_, w2_static_);
  nh_.param("astar/w3_dynamic", w3_dynamic_, w3_dynamic_);
}

void AStarOccMap::setMap(const std::shared_ptr<mapManager::occMap> &map)
{
  map_ = map;
  if (map_)
    res_ = map_->getRes();
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
  Eigen::Vector3i idx;
  map_->posToIndex(pos, idx);
  ix = idx(0);
  iy = idx(1);
  iz = idx(2);
  return map_->isInMap(idx);
}

Eigen::Vector3d AStarOccMap::indexToPos(int ix, int iy, int iz) const
{
  Eigen::Vector3i idx(ix, iy, iz);
  Eigen::Vector3d pos;
  map_->indexToPos(idx, pos);
  return pos;
}

bool AStarOccMap::isFree(int ix, int iy, int iz) const
{
  if (!map_) return false;
  Eigen::Vector3i idx(ix, iy, iz);
  return map_->isInMap(idx) && (!map_->isInflatedOccupied(idx));
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
  // 若没有预测数据，动态代价为 0
  if (dyn_preds_.empty()) return 0.0;

  double t_arrival = estimateArrivalTime(pos);
  if (!std::isfinite(t_arrival)) return 0.0;

  double total_cost = 0.0;
  for (const auto &ob : dyn_preds_)
  {
    for (const auto &intent : ob.intents)
    {
      if (intent.probability <= 0.0) continue;
      // 本实现假设 mean/cov_inv 已对应 t_arrival 预先给好；
      // 如需插值，可在外部准备好 t_arrival 对应的 mean/cov_inv。
      const Eigen::Vector3d &mu = intent.mean;
      const Eigen::Matrix3d &cov_inv = intent.cov_inv;

      // 马氏距离平方
      Eigen::Vector3d delta = pos - mu;
      double mahalanobis_sq = delta.transpose() * cov_inv * delta;
      // 风险值
      double risk = std::exp(-0.5 * mahalanobis_sq);
      total_cost += intent.probability * risk;
    }
  }
  return total_cost;
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

  int sx, sy, sz, gx, gy, gz;
  Eigen::Vector3d s(start_.position.x, start_.position.y, start_.position.z);
  Eigen::Vector3d g(goal_.position.x, goal_.position.y, goal_.position.z);
  if (!posToIndex(s, sx, sy, sz) || !posToIndex(g, gx, gy, gz))
  {
    ROS_WARN("[A*] start or goal out of map.");
    return;
  }
  if (!isFree(gx, gy, gz))
  {
    ROS_WARN("[A*] goal in obstacle.");
    return;
  }

  auto heuristic = [&](int x, int y, int z) {
    return std::sqrt((x - gx) * (x - gx) + (y - gy) * (y - gy) + (z - gz) * (z - gz));
  };

  std::vector<Node> closed;
  auto cmp = [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; };
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> open(cmp);
  std::unordered_map<int, double> gscore;
  std::unordered_map<int, double> gdist; // 纯距离累计

  Node startNode{sx, sy, sz, 0.0, heuristic(sx, sy, sz), -1};
  closed.push_back(startNode);
  open.push({startNode.f(), 0});
  gscore[idx1d(sx, sy, sz)] = 0.0;
  gdist[idx1d(sx, sy, sz)] = 0.0;

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
      double step = std::sqrt(dx[k]*dx[k] + dy[k]*dy[k] + dz[k]*dz[k]);
      // 计算代价分解
      Eigen::Vector3d nbPos = indexToPos(nx, ny, nz);
      double cost_dist = step;
      double cost_static = 0.0; // 预留：若加入距离场，可在此填写
      double cost_dynamic = calculateDynamicCost(nbPos);
      double move_cost = w1_dist_ * cost_dist + w2_static_ * cost_static + w3_dynamic_ * cost_dynamic;

      double tentative = cur.g + move_cost;
      double tentative_dist = gdist[idx1d(cur.x, cur.y, cur.z)] + step;
      int h = idx1d(nx, ny, nz);
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
  }

  if (!found)
  {
    ROS_WARN("[A*] no path found.");
    return;
  }

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

