#include "SSSP.h"

struct HLNodeComparator {
  bool operator()(const shared_ptr<HLNode>& lhs, const shared_ptr<HLNode>& rhs) const {
    return lhs->cost > rhs->cost;
  }
};

Solution SSSP::run() {
  Solution solution;
  initRoadmaps();
  while (true) {
    auto open = priority_queue<shared_ptr<HLNode>, vector<shared_ptr<HLNode>>, HLNodeComparator>();
    auto explored = unordered_set<shared_ptr<HLNode>>();
    // update cost of all nodes
    for (int i = 0; i < env.num_of_robots; ++i) {
      dijkstra(roadmaps[i], roadmap_constructors[i].goal_node);
    }
    double cost = 0;
    for (int i = 0; i < env.num_of_robots; ++i) {
      cost += roadmap_constructors[i].start_node->cost;
    }
    vector<shared_ptr<LLNode>> nodes;
    nodes.reserve(env.num_of_robots);
    for (int i = 0; i < env.num_of_robots; ++i) {
      nodes.emplace_back(roadmap_constructors[i].start_node);
    }
    shared_ptr<HLNode> start_hl_node = make_shared<HLNode>(nodes, 0, cost);
    open.push(start_hl_node);
    explored.insert(start_hl_node);

    while (!open.empty()) {
      const auto curr_hl_node = open.top();
      open.pop();

      // check goal
      vector<Point> configurations;
      for (int i = 0; i < env.num_of_robots; ++i) {
        configurations.emplace_back(curr_hl_node->nodes[i]->point);
        cout << "Agent " << i << ": (" << get<0>(curr_hl_node->nodes[i]->point) << ", "
             << get<1>(curr_hl_node->nodes[i]->point) << ")" << endl;
      }
      if (configurations == env.goal_points) {
        cout << "SSSP: Found solution" << endl;
        solution = updatePath(curr_hl_node);
        return solution;
      }

      // vertex expansion
      const auto agent_id = curr_hl_node->next;
      const auto from_node = curr_hl_node->nodes[agent_id];
      cout << "Vertex expansion: " << agent_id << endl;

      for (int i = 0; i < num_of_sampling; i++) {
        auto new_point = roadmap_constructors[agent_id].generateRandomPoint();
        shared_ptr<LLNode> new_node = make_shared<LLNode>(new_point);
        if (dis_100(gen) > random_sampling_rate) {
          // steering
          new_node = roadmap_constructors[agent_id].steer(from_node->point, new_point);
        }
        // find minimum distance from q_new to nodes in roadmap
        auto min_distance = numeric_limits<double>::max();
        for (const auto& node : roadmaps[agent_id]) {
          const double distance = calculateDistance(node->point, new_point);
          if (distance < min_distance) {
            min_distance = distance;
          }
        }
        if (min_distance > threshold) {
          // add q_new to roadmap
          roadmaps[agent_id].emplace_back(new_node);
          cout << "Add node (" << get<0>(new_node->point) << ", " << get<1>(new_node->point) << ") to roadmap "
               << agent_id << endl;
          for (const auto& node : roadmaps[agent_id]) {
            if (!roadmap_constructors[agent_id].obstacleConstrained(node->point, new_node->point,
                                                                    env.radii[agent_id])) {
              new_node->adjacent_nodes.emplace_back(node);
              node->adjacent_nodes.emplace_back(new_node);
            }
          }
        }
      }

      // update cost of agent_id
      dijkstra(roadmaps[agent_id], roadmap_constructors[agent_id].goal_node);

      // node expansion
      cout << "Node expansion: " << agent_id << endl;
      const auto next_agent_id = (agent_id + 1) % env.num_of_robots;
      for (const auto& adjacent_node : from_node->adjacent_nodes) {
        auto new_nodes = curr_hl_node->nodes;
        new_nodes[agent_id] = adjacent_node;

        double new_cost = 0;
        for (int i = 0; i < env.num_of_robots; ++i) {
          new_cost += adjacent_node->cost;
        }

        auto new_hl_node = make_shared<HLNode>(new_nodes, next_agent_id, new_cost);
        if (explored.find(new_hl_node) == explored.end()) {
          if (agentConstrained(agent_id, from_node->point, adjacent_node->point, curr_hl_node->nodes,
                               env.radii[agent_id])) {
            continue;
          }
          new_hl_node->parent = curr_hl_node;
          open.push(new_hl_node);
          explored.insert(new_hl_node);
        }
      }
    }
    threshold = threshold * decay_rate;
  }
}

void SSSP::initRoadmaps() {
  for (int agent_id = 0; agent_id < env.num_of_robots; ++agent_id) {
    roadmap_constructors[agent_id].run();
    roadmaps.push_back(roadmap_constructors[agent_id].nodes);
  }
}

bool SSSP::agentConstrained(int agent_id, const Point& from_point, const Point& to_point,
                            const vector<shared_ptr<LLNode>>& nodes, double radius) const {
  vector<Point> interpolated_points;
  roadmap_constructors[agent_id].interpolatePoint(agent_id, from_point, to_point, interpolated_points);
  for (const auto& interpolated_point : interpolated_points) {
    for (int other_agent_id = 0; other_agent_id < env.num_of_robots; ++other_agent_id) {
      if (other_agent_id == agent_id) {
        continue;
      }
      if (calculateDistance(interpolated_point, nodes[other_agent_id]->point) < radius + env.radii[other_agent_id]) {
        return true;
      }
    }
  }
  return false;
}

struct LLNodeComparator {
  bool operator()(const shared_ptr<LLNode>& lhs, const shared_ptr<LLNode>& rhs) const {
    return lhs->cost > rhs->cost;
  }
};

void SSSP::dijkstra(Roadmap roadmap, const shared_ptr<LLNode> source_node) {
  using NodeHandle = boost::heap::fibonacci_heap<shared_ptr<LLNode>, boost::heap::compare<LLNodeComparator>>::handle_type;

  boost::heap::fibonacci_heap<shared_ptr<LLNode>, boost::heap::compare<LLNodeComparator>> open;
  std::unordered_map<shared_ptr<LLNode>, NodeHandle> handles;
  std::unordered_set<shared_ptr<LLNode>> explored;

  for (const auto& node : roadmap) {
    node->cost = numeric_limits<double>::max();
  }

  source_node->cost = 0;
  handles[source_node] = open.push(source_node);

  while (!open.empty()) {
    auto curr_node = open.top();
    open.pop();
    explored.insert(curr_node);

    for (const auto& adjacent_node : curr_node->adjacent_nodes) {
      if (explored.find(adjacent_node) == explored.end()) {
        const double new_cost = curr_node->cost + calculateDistance(curr_node->point, adjacent_node->point);
        if (new_cost < adjacent_node->cost) {
          adjacent_node->cost = new_cost;

          // Decrease key if the node is already in the heap, otherwise insert it
          if (handles.find(adjacent_node) != handles.end()) {
            open.decrease(handles[adjacent_node], adjacent_node);
          } else {
            handles[adjacent_node] = open.push(adjacent_node);
          }
        }
      }
    }
  }
}

Solution SSSP::updatePath(const shared_ptr<HLNode>& goal_node) const {
  Solution solution;
  solution.resize(env.num_of_robots);
  shared_ptr<HLNode> node = goal_node;
  double timestep = 0.0;
  while (node) {
    for (int i = 0; i < env.num_of_robots; ++i) {
      solution[i].emplace_back(node->nodes[i]->point, timestep);
    }
    node = node->parent.lock();
    timestep += 1.0;
  }
  return solution;
}