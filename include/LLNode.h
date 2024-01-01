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
  int min_soft_conflict;
  vector<int> soft_conflicts;
  vector<int> parent_interval_indices;

  explicit LLNode(Point point)
      : point(std::move(point)),
        earliest_arrival_time(numeric_limits<double>::infinity()),
        min_soft_conflict(numeric_limits<double>::infinity()) {}

  // compare node by earliest_arrival_time
  // bool operator<(const LLNode& other) const {
  //   if (earliest_arrival_time == other.earliest_arrival_time) {
  //     return min_soft_conflict < other.min_soft_conflict;
  //   }
  //   return earliest_arrival_time < other.earliest_arrival_time;
  // }

  // compare node by min soft conflict
  bool operator<(const LLNode& other) const {
    if (min_soft_conflict == other.min_soft_conflict) {
      return earliest_arrival_time < other.earliest_arrival_time;
    }
    return min_soft_conflict < other.min_soft_conflict;
  }
};

#endif  // LLNODE_H
