#include <global_planner/a_star_occ.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

namespace globalPlanner
{

AStarOccMap::AStarOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  nh_.param("astar/use_26_dir", use26dir_, use26dir_);
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

  Node startNode{sx, sy, sz, 0.0, heuristic(sx, sy, sz), -1};
  closed.push_back(startNode);
  open.push({startNode.f(), 0});
  gscore[idx1d(sx, sy, sz)] = 0.0;

  // 26邻域（含6/18/26），或仅6邻域
  const int dx26[26] = {1,1,1,1,1,1, 1,1,1, 0,0,0,0,0,0,0,0,0, -1,-1,-1,-1,-1,-1,-1,-1,-1};
  const int dy26[26] = {1,1,1,0,0,0,-1,-1,-1, 1,1,1,0,0,0,-1,-1,-1, 1,1,1,0,0,0,-1,-1,-1};
  const int dz26[26] = {1,0,-1,1,0,-1,1,0,-1, 1,0,-1,1,0,-1,1,0,-1, 1,0,-1,1,0,-1,1,0,-1};
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
      double tentative = cur.g + step;
      int h = idx1d(nx, ny, nz);
      if (!gscore.count(h) || tentative < gscore[h])
      {
        gscore[h] = tentative;
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
    ps.header.frame_id = !goal_.header.frame_id.empty() ? goal_.header.frame_id : "map";
    ps.pose.position.x = wp(0);
    ps.pose.position.y = wp(1);
    ps.pose.position.z = wp(2);
    ps.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    poses.push_back(ps);
    trace = n.parent;
  }
  std::reverse(poses.begin(), poses.end());

  path.header.stamp = ros::Time::now();
  path.header.frame_id = !goal_.header.frame_id.empty() ? goal_.header.frame_id : "map";
  path.poses = poses;
}

}  // namespace globalPlanner

