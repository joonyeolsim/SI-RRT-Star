#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
 public:
  Point point;
  shared_ptr<LLNode> parent;
  double earliest_arrival_time;
  double velocity;
  Interval interval;

  explicit LLNode(Point point, double lower_bound, double upper_bound) :
  point(std::move(point)),
  earliest_arrival_time(numeric_limits<double>::infinity()),
  velocity(0.0),
  interval(lower_bound, upper_bound) {}
};

#endif  // LLNODE_H
