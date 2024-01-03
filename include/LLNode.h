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
  int num_of_soft_conflicts{};

  explicit LLNode(Point point)
      : point(std::move(point)),
        interval(0, numeric_limits<double>::infinity()),
        num_of_soft_conflicts(numeric_limits<int>::max()) {}

  // compare node by earliest_arrival_time
  bool operator<(const LLNode& other) const {
    if (get<0>(interval) == get<0>(other.interval)) {
      return num_of_soft_conflicts < other.num_of_soft_conflicts;
    }
    return get<0>(interval) < get<0>(other.interval);
  }

  // compare node by min soft conflict
  // bool operator<(const LLNode& other) const {
  //   if (min_soft_conflict == other.min_soft_conflict) {
  //     return earliest_arrival_time < other.earliest_arrival_time;
  //   }
  //   return min_soft_conflict < other.min_soft_conflict;
  // }
};

#endif  // LLNODE_H
