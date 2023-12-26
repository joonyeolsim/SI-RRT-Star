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
  vector<shared_ptr<LLNode>> children;
  double earliest_arrival_time;
  vector<Interval> intervals;
  vector<int> parent_interval_indicies;

  explicit LLNode(Point point) : point(std::move(point)), earliest_arrival_time(numeric_limits<double>::max()) {}
};

#endif  // LLNODE_H
