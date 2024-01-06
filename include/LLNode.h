//
// Created by joonyeol on 23. 12. 7.
//

#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
 public:
  Point point;
  weak_ptr<LLNode> parent;
  vector<shared_ptr<LLNode>> adjacent_nodes;
  double cost = 0.0;

  explicit LLNode(Point point) : point(std::move(point)) {}

  // operator for priority queue
        bool operator<(const LLNode& other) const { return cost < other.cost; }

  // operator for unordered_set
        bool operator==(const LLNode& other) const { return point == other.point; }
};

#endif  // LLNODE_H
