#include <global_planner/a_star_occ.h>
#include <tf/transform_datatypes.h>
#include <algorithm>

namespace globalPlanner
{

AStarOccMap::AStarOccMap(const ros::NodeHandle &nh) : nh_(nh)
{
  nh_.param("astar/use_8_dir", use8dir_, use8dir_);
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

bool AStarOccMap::posToIndex(const Eigen::Vector3d &pos, int &ix, int &iy) const
{
  if (!map_) return false;
  Eigen::Vector3i idx;
  map_->posToIndex(pos, idx);
  ix = idx(0);
  iy = idx(1);
  return map_->isInMap(idx);
}

Eigen::Vector3d AStarOccMap::indexToPos(int ix, int iy, double z) const
{
  Eigen::Vector3i idx(ix, iy, 0);
  Eigen::Vector3d pos;
  map_->indexToPos(idx, pos);
  pos(2) = z;
  return pos;
}

bool AStarOccMap::isFree(int ix, int iy) const
{
  if (!map_) return false;
  Eigen::Vector3i idx(ix, iy, 0);
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

  int sx, sy, gx, gy;
  Eigen::Vector3d s(start_.position.x, start_.position.y, start_.position.z);
  Eigen::Vector3d g(goal_.position.x, goal_.position.y, goal_.position.z);
  if (!posToIndex(s, sx, sy) || !posToIndex(g, gx, gy))
  {
    ROS_WARN("[A*] start or goal out of map.");
    return;
  }
  if (!isFree(gx, gy))
  {
    ROS_WARN("[A*] goal in obstacle.");
    return;
  }

  auto heuristic = [&](int x, int y) { return std::hypot(x - gx, y - gy); };
  auto idx1d = [&](int x, int y) {
    // assume map size within int
    return y * 100000 + x;  // simple hash; better to use pair hash but sufficient if map size <1e5
  };

  std::vector<Node> closed;
  auto cmp = [](const std::pair<double, int> &a, const std::pair<double, int> &b) { return a.first > b.first; };
  std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, decltype(cmp)> open(cmp);
  std::unordered_map<int, double> gscore;

  Node startNode{sx, sy, 0.0, heuristic(sx, sy), -1};
  closed.push_back(startNode);
  open.push({startNode.f(), 0});
  gscore[idx1d(sx, sy)] = 0.0;

  const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  const int dx4[4] = {1, 0, -1, 0};
  const int dy4[4] = {0, 1, 0, -1};

  bool found = false;
  int goalIdx = -1;

  while (!open.empty())
  {
    int curIdx = open.top().second;
    open.pop();
    const Node &cur = closed[curIdx];
    if (cur.x == gx && cur.y == gy)
    {
      found = true;
      goalIdx = curIdx;
      break;
    }

    const int *dx = use8dir_ ? dx8 : dx4;
    const int *dy = use8dir_ ? dy8 : dy4;
    int neigh = use8dir_ ? 8 : 4;
    for (int k = 0; k < neigh; ++k)
    {
      int nx = cur.x + dx[k];
      int ny = cur.y + dy[k];
      if (!isFree(nx, ny)) continue;
      double step = (k % 2 == 0 || !use8dir_) ? 1.0 : std::sqrt(2.0);
      double tentative = cur.g + step;
      int h = idx1d(nx, ny);
      if (!gscore.count(h) || tentative < gscore[h])
      {
        gscore[h] = tentative;
        Node nb{nx, ny, tentative, heuristic(nx, ny), curIdx};
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
    Eigen::Vector3d wp = indexToPos(n.x, n.y, goal_.position.z);
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

