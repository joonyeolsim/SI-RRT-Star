//
// Created by joonyeol on 23. 12. 7.
//

#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
 public:
  Point point;
  double time;
  shared_ptr<LLNode> parent;
  vector<Interval> intervals;

  LLNode(Point point, double time) : point(std::move(point)), time(time), parent(nullptr) {}
};

#endif  // LLNODE_H
