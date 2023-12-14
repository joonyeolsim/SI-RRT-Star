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
  bool obstacleConstrained(int agent_id, const shared_ptr<LLNode>& from_node, const Point& to_point,
                           double radius) const;
  bool pathConstrained(int agent_id, const shared_ptr<LLNode>& from_node, const Point& to_point, double to_time,
                       double radius) const;
  bool targetConstrained(const Point& other_point, double other_time, double other_radius) const;
  void interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                        vector<Point>& interpoate_points) const;
  void interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                            double to_time, vector<Point>& interpoate_points, vector<double>& interpoate_times) const;
};
#endif  // CONSTRAINTTABLE_H
