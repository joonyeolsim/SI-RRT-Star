#ifndef CONSTRAINTTABLE_H
#define CONSTRAINTTABLE_H

#include "SharedEnv.h"
#include "common.h"

class ConstraintTable {
 public:
  vector<Path> path_table;
  vector<vector<Constraint>> hard_constraint_table;
  vector<vector<Constraint>> soft_constraint_table;
  SharedEnv& env;

  ConstraintTable(SharedEnv& env)
      : env(env),
        path_table(env.num_of_robots),
        hard_constraint_table(env.num_of_robots),
        soft_constraint_table(env.num_of_robots) {}
  bool obstacleConstrained(const Point& point, double radius) const;
  bool targetConstrained(int agent_id, const Point& point, int timestep, double radius) const;
  bool pathConstrained(int agent_id, const Point& point, int timestep, double radius) const;
  bool hardConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time, double to_time,
                       double radius) const;
  void getSafeIntervalTablePath(int agent_id, const Point& point, double radius, vector<Interval>& safe_intervals) const;
  void getSafeIntervalTable(int agent_id, const Point& to_point, double radius, vector<Interval>& safe_intervals) const;
  int getEarliestArrivalTime(int agent_id, const Point& from_point, const Point& to_point, int lower_bound, int upper_bound, double radius) const;
  void insertCollisionIntervalToSIT(vector<Interval>& safe_intervals, int t_min, int t_max) const;
  void interpolatePoint(int agent_id, const Point& from_point, const Point& to_point, vector<Point>& interpoate_points) const;
};
#endif  // CONSTRAINTTABLE_H
