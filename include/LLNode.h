#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
 public:
  Point point;
  Interval interval;
  double velocity;
  shared_ptr<LLNode> parent;
  double earliest_arrival_time;

  explicit LLNode(Point point, double lower_bound, double upper_bound, double velocity) :
  point(std::move(point)),
  interval(lower_bound, upper_bound),
  velocity(velocity),
  earliest_arrival_time(numeric_limits<double>::infinity()) {}
};

#endif  // LLNODE_H
