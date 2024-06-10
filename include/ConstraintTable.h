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
  bool obstacleConstrained(int agent_id, const Point& from_point, const Point& to_point, double velocity, double radius) const;
  bool pathConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time, double to_time,
                       double radius) const;
  bool hardConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time, double to_time,
                       double radius) const;
  bool targetConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time, double to_time,
                         double radius) const;
  void getSafeIntervalTablePath(int agent_id, const Point& to_point, double radius,
                                vector<Interval>& safe_intervals) const;
  void getSafeIntervalTable(int agent_id, const Point& to_point, double radius, vector<Interval>& safe_intervals) const;
  double getEarliestArrivalTime(int agent_id, const Point& from_point, const Point& to_point, double from_time, double lower_bound, double upper_bound, double radius) const;
  void insertCollisionIntervalToSIT(vector<Interval>& safe_intervals, double t_min, double t_max) const;
  void interpolatePoint(int agent_id, const Point& from_point, const Point& to_point, double velocity, vector<Point>& interpolated_points) const;
  void interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point,
                                           double from_time, double to_time, double velocity, vector<Point>& interpolated_points,
                                           vector<double>& interpolated_times) const;
  bool checkConflicts(const Solution &solution) const;
};
#endif  // CONSTRAINTTABLE_H
