//
// Created by joonyeol on 23. 12. 8.
//

#ifndef CONSTRAINTTABLE_H
#define CONSTRAINTTABLE_H

#include "LLNode.h"
#include "SharedEnv.h"
#include "common.h"

class ConstraintTable {
 public:
  vector<Path> path_table;
  vector<Constraint> constraint_table;
  SharedEnv& env;

  ConstraintTable(SharedEnv& env, int num_of_agents) : env(env) { path_table.resize(num_of_agents); }
  void insertPathToConstraint(int agent_id, Path path);
  bool obstacleConstrained(const Point& other_point, double other_radius) const;
  bool pathConstrained(const Point& other_point, double other_time, double other_radius) const;
  bool targetConstrained(const Point& other_point, double other_time, double other_radius) const;
};

#endif  // CONSTRAINTTABLE_H
