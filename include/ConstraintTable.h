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
  vector<vector<Constraint>> constraint_table;
  SharedEnv& env;

  ConstraintTable(SharedEnv& env) : env(env), path_table(env.num_of_robots), constraint_table(env.num_of_robots) {}
  void insertPathToConstraint(int agent_id, Path path);
  bool obstacleConstrained(int agent_id, const Point& from_point, const Point& to_point, double radius) const;
  void getSafeIntervalTablePath(int agent_id, const Point& to_point, double radius,
                                vector<Interval>& safe_intervals) const;
  void getSafeIntervalTableConstraint(int agent_id, const Point& to_point, double radius,
                                      vector<Interval>& safe_intervals) const;
  bool pathConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time, double to_time,
                       double radius) const;
  bool constrained(int agent_id, const Point& from_point, const Point& to_point, double from_time, double to_time,
                   double radius) const;
  bool targetConstrainedPath(const Point& other_point, double other_time, double other_radius) const;
  static void insertToSafeIntervalTable(vector<Interval>& safe_intervals, double t_min, double t_max);
  void interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                        vector<Point>& interpoate_points) const;
  void interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                            double to_time, vector<Point>& interpoate_points, vector<double>& interpoate_times) const;
  void updateConstraints(int agent_id, const vector<Constraint>& constraints);
  bool targetConstrained(int agent_id, const Point& other_point, double other_time, double other_radius) const;
};
#endif  // CONSTRAINTTABLE_H
