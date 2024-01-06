//
// Created by joonyeol on 23. 12. 7.
//

#ifndef HLNODE_H
#define HLNODE_H

#include "common.h"

class HLNode {
 public:
  vector<shared_ptr<LLNode>> nodes;
  int next;
  weak_ptr<HLNode> parent;
  double cost;

  explicit HLNode(vector<shared_ptr<LLNode>> nodes, int next, double cost)
      : nodes(std::move(nodes)), next(next), cost(cost) {}

  // operator for priority queue
  bool operator<(const HLNode& other) const {
    return cost < other.cost;
  }

  // operator for unordered_set
  bool operator==(const HLNode& other) const {
    if (nodes.size() != other.nodes.size()) return false;
    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i]->point != other.nodes[i]->point) return false;
    }
    if (next != other.next) return false;
    return true;
  }
};

#endif  // HLNODE_H
