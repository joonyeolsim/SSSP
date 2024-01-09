#include "RRT.h"

Path RRT::run() {
  start_node = make_shared<LLNode>(start_point);
  start_node->adjacent_nodes.emplace_back(start_node);
  nodes.emplace_back(start_node);
  int iteration = env.iterations[agent_id];
  while (iteration--) {
    Point random_point = generateRandomPoint();
    shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    shared_ptr<LLNode> new_node = steer(nearest_node->point, random_point);
    if (new_node) {
      new_node->parent = nearest_node;
      new_node->adjacent_nodes.emplace_back(nearest_node);
      new_node->adjacent_nodes.emplace_back(new_node);
      nearest_node->adjacent_nodes.emplace_back(new_node);
      nodes.emplace_back(new_node);
      if (calculateDistance(new_node->point, goal_point) < env.epsilon) {
        goal_node = new_node;
        return updatePath(goal_node);
      }
    }
  }
  cout << "RRT: No path found" << endl;
  return {};
}

Point RRT::generateRandomPoint() {
  const bool selectGoalPoint = dis_100(env.gen) < env.goal_sample_rates[agent_id];

  return selectGoalPoint ? goal_point : make_tuple(dis_width(env.gen), dis_height(env.gen));
}

shared_ptr<LLNode> RRT::getNearestNode(const Point& point) const {
  if (nodes.empty()) {
    return nullptr;
  }
  double min_distance = numeric_limits<double>::max();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}

shared_ptr<LLNode> RRT::steer(const Point& from_point, const Point& random_point) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_point, random_point));
  const double theta =
      atan2(get<1>(random_point) - get<1>(from_point), get<0>(random_point) - get<0>(from_point));
  Point to_point = make_tuple(get<0>(from_point) + cos(theta) * expand_distance,
                              get<1>(from_point) + sin(theta) * expand_distance);

  if (obstacleConstrained(from_point, to_point, env.radii[agent_id])) {
    return nullptr;
  }
  shared_ptr<LLNode> new_node = make_shared<LLNode>(to_point);
  return new_node;
}

bool RRT::obstacleConstrained(const Point& from_point, const Point& to_point, double radius) const {
  vector<Point> interpolated_points;
  interpolatePoint(agent_id, from_point, to_point, interpolated_points);
  for (auto& interpolated_point : interpolated_points) {
    for (const auto& obstacle : env.obstacles) {
      if (obstacle->constrained(interpolated_point, radius)) return true;
    }
  }
  return false;
}

Path RRT::updatePath(const shared_ptr<LLNode>& goal_node) const {
  Path path = {};
  shared_ptr<LLNode> node = goal_node;
  while (node) {
    path.emplace_back(node->point, 0.0);
    node = node->parent.lock();
  }
  reverse(path.begin(), path.end());
  return path;
}

void RRT::interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                           vector<Point>& interpolated_points) const {
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const auto timesteps = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));
  for (int timestep = 0; timestep < timesteps; ++timestep) {
    Point interpoated_point = from_point;
    if (theta != 0.0) {
      interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * timestep,
                                     get<1>(from_point) + env.velocities[agent_id] * sin(theta) * timestep);
    }
    interpolated_points.emplace_back(interpoated_point);
  }
  const double remain_time = fmod(expand_distance, env.velocities[agent_id]);
  if (remain_time > env.epsilon) {
    Point interpoated_point = from_point;
    if (theta != 0.0) {
      interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                     get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
    }
    interpolated_points.emplace_back(interpoated_point);
  }

  if (interpolated_points.empty()) {
    interpolated_points.emplace_back(from_point);
  }

  assert(!interpolated_points.empty());
}