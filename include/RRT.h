//
// Created by joonyeol on 24. 1. 4.
//

#ifndef RRT_H
#define RRT_H

#include "LLNode.h"
#include "SharedEnv.h"
#include "common.h"

class RRT {
 public:
  std::uniform_real_distribution<> dis_width;
  std::uniform_real_distribution<> dis_height;
  std::uniform_real_distribution<> dis_100;
  vector<shared_ptr<LLNode>> nodes;
  shared_ptr<LLNode> goal_node;
  Point start_point;
  Point goal_point;
  Path path;
  int agent_id;
  SharedEnv& env;
  int path_resolution = 1.0;

  RRT(int agent_id, SharedEnv& env)
      : dis_width(env.radii[agent_id], env.width - env.radii[agent_id]),
        dis_height(env.radii[agent_id], env.height - env.radii[agent_id]),
        dis_100(0.0, 100.0),
        env(env),
        agent_id(agent_id),
        start_point(env.start_points[agent_id]),
        goal_point(env.goal_points[agent_id]) {}
  ~RRT() = default;
  Path run();
  Point generateRandomPoint();
  shared_ptr<LLNode> getNearestNode(const Point& point) const;
  shared_ptr<LLNode> steer(const shared_ptr<LLNode>& from_node, const Point& random_point) const;
  Path updatePath(const shared_ptr<LLNode>& goal_node) const;
  void interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                        vector<Point>& interpolated_points) const;
  bool obstacleConstrained(const Point& from_point, const Point& to_point, double radius) const;
};

#endif  // RRT_H
