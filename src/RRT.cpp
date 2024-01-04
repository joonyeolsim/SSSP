#include "RRT.h"

Path RRT::run() {
    nodes.emplace_back(make_shared<LLNode>(start_point));
    int iteration = env.iterations[agent_id];
    while (iteration--) {
        Point random_point = generateRandomPoint();
        shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
        shared_ptr<LLNode> new_node = steer(nearest_node, random_point);
        if (new_node) {
            nodes.emplace_back(new_node);
            if (new_node->point == goal_point) {
                goal_node = new_node;
                return updatePath(goal_node);
            }
        }
    }
    return {};
}

Point RRT::generateRandomPoint() {
    const bool selectGoalPoint = dis_100(env.gen) < env.goal_sample_rates[agent_id];

    return selectGoalPoint ? make_tuple(get<0>(goal_point), get<1>(goal_point))
                           : make_tuple(dis_width(env.gen), dis_height(env.gen));
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

shared_ptr<LLNode> RRT::steer(const shared_ptr<LLNode>& from_node, const Point& random_point) const {
    const double expand_distance =
        min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
    const double theta =
        atan2(get<1>(random_point) - get<1>(from_node->point), get<0>(random_point) - get<0>(from_node->point));
    Point to_point = from_node->point;

    const auto steps = static_cast<int>(floor(expand_distance / path_resolution));
    for (int step = 0; step < steps; step++) {
        to_point = make_tuple(get<0>(from_node->point) + cos(theta) * path_resolution * step,
                              get<1>(from_node->point) + sin(theta) * path_resolution * step);
        if (obstacleConstrained(to_point, env.radii[agent_id])) {
            return nullptr;
        }
    }
    const double remain_step = fmod(expand_distance, path_resolution);
    if (remain_step > env.epsilon) {
        to_point = make_tuple(get<0>(from_node->point) + cos(theta) * (expand_distance / path_resolution),
                              get<1>(from_node->point) + sin(theta) * (expand_distance / path_resolution));
        if (obstacleConstrained(to_point, env.radii[agent_id])) {
            return nullptr;
        }
    }

    shared_ptr<LLNode> new_node = make_shared<LLNode>(to_point);
    new_node->parent = from_node;
    new_node->cost = from_node->cost + expand_distance;
    from_node->children.emplace_back(new_node);
    return new_node;
}

shared_ptr<LLNode> steer(const shared_ptr<LLNode>& from_node, const Point& random_point) const;
Path updatePath(const shared_ptr<LLNode>& goal_node) const;