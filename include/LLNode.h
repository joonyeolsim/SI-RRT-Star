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
  Interval interval;

  explicit LLNode(Point point) : point(std::move(point)), interval(0, numeric_limits<double>::infinity()) {}
};

#endif  // LLNODE_H
