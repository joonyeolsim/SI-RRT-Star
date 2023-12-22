//
// Created by joonyeol on 23. 12. 8.
//

#ifndef SICBS_H
#define SICBS_H

#include "ConstraintTable.h"
#include "HLNode.h"
#include "SharedEnv.h"
#include "common.h"

#include <SIRRT.h>

struct compare_function {
  bool operator()(const HLNode& a, const HLNode& b) const { return a.cost > b.cost; }
};

class SICBS {
 public:
  boost::heap::fibonacci_heap<HLNode, boost::heap::compare<compare_function>> open_list;
  vector<shared_ptr<HLNode>> nodes;
  SharedEnv& env;
  ConstraintTable& constraint_table;
  vector<SIRRT> low_level_planners;

  SICBS(SharedEnv& env, ConstraintTable& constraint_table) : env(env), constraint_table(constraint_table) {
    low_level_planners.reserve(env.num_of_robots);
    for (int i = 0; i < env.num_of_robots; i++) {
      low_level_planners.emplace_back(i, env, constraint_table);
    }
  }
  ~SICBS() = default;
  Solution run();
  Solution getInitialSolution();
  static double calculateCost(const Solution& solution);
  void findConflicts(const Solution& solution, vector<Conflict>& conflicts) const;
};

#endif  // SICBS_H
