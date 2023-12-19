#ifndef RESERVATIONTABLE_H
#define RESERVATIONTABLE_H

#include "SharedEnv.h"
#include "common.h"

class ReservationTable {
 public:
  unordered_map<Point, vector<Interval>, PointHash> safe_interval_table;
  SharedEnv& env;
  explicit ReservationTable(SharedEnv& env) : env(env) {}
  void insertSafeIntervalTable(const Point& point, const vector<Interval>& safe_intervals);
};
#endif  // RESERVATIONTABLE_H
