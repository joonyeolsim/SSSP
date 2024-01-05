//
// Created by joonyeol on 23. 12. 7.
//

#ifndef HLNODE_H
#define HLNODE_H

#include "common.h"

class HLNode {
 public:
  vector<Point> configurations;
  vector<shared_ptr<HLNode>> nodes;
  int next;
  weak_ptr<HLNode> parent;
  double cost;

  explicit HLNode(vector<Point> configurations, int next, double cost)
      : configurations(std::move(configurations)), next(next) {}

  // operator for priority queue
  bool operator<(const HLNode& other) const { return cost > other.cost; }

  // operator for unordered_set
  bool operator==(const HLNode& other) const {
    if (configurations.size() != other.configurations.size()) return false;
    for (int i = 0; i < configurations.size(); i++) {
      if (configurations[i] != other.configurations[i]) return false;
    }
    if (next != other.next) return false;
    return true;
  }
};

#endif  // HLNODE_H
