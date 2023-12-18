//
// Created by joonyeol on 23. 12. 7.
//

#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
 public:
  Point point;
  shared_ptr<LLNode> parent;
  vector<double> earliest_arrival_times;
  vector<Interval> intervals;
  vector<int> parent_interval_indicies;

  explicit LLNode(Point point) : point(std::move(point)), parent(nullptr) {}
};

#endif  // LLNODE_H
