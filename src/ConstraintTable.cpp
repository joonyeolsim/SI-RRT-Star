#include "ConstraintTable.h"

bool ConstraintTable::obstacleConstrained(int agent_id, const Point& from_point, const Point& to_point,
                                          double radius) const {
  vector<Point> interpolated_points;
  interpolatePoint(agent_id, from_point, to_point, interpolated_points);
  for (auto& interpolated_point : interpolated_points) {
    for (const auto& obstacle : env.obstacles) {
      if (obstacle->constrained(interpolated_point, radius)) return true;
    }
  }
  return false;
}

bool ConstraintTable::pathConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                                      double to_time, double radius) const {
  assert(from_time < to_time);
  const auto expand_time = 1.0;
  const auto velcotiy_from2to = calculateDistance(from_point, to_point) / expand_time;
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];

      // check if temporal constraint is satisfied
      if (next_time <= from_time) continue;
      if (prev_time >= to_time) break;

      double start_time = max(from_time, prev_time);
      double end_time = min(to_time, next_time);

      const auto velcotiy_prev2next = calculateDistance(prev_point, next_point) / expand_time;

      auto curr_time = start_time;
      while (curr_time < end_time) {
        // get point at start_time
        const auto occupied_expand_time = curr_time - prev_time;
        assert(occupied_expand_time >= 0.0);
        const auto occupied_theta =
            atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));

        auto occupied_point = prev_point;
        if (occupied_theta != 0.0) {
          occupied_point = make_tuple(
              get<0>(prev_point) + velcotiy_prev2next * cos(occupied_theta) * occupied_expand_time,
              get<1>(prev_point) + velcotiy_prev2next * sin(occupied_theta) * occupied_expand_time);
        }

        // get point2 at start_time
        const auto expand_time = curr_time - from_time;
        assert(expand_time >= 0.0);
        const auto theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

        auto point = from_point;
        if (theta != 0.0) {
          point = make_tuple(get<0>(from_point) + velcotiy_from2to * cos(theta) * expand_time,
                             get<1>(from_point) + velcotiy_from2to * sin(theta) * expand_time);
        }

        if (calculateDistance(point, occupied_point) < radius + env.radii[occupied_agent_id] + env.epsilon) {
          return true;
        }

        curr_time += env.time_resolution;
      }
    }
  }
  return false;
}

bool ConstraintTable::hardConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                                      double to_time, double radius) const {
  assert(from_time < to_time);
  const double expand_distance = calculateDistance(from_point, to_point);
  // vertex-edge conflict
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    if (constrained_path.empty()) continue;
    for (int i = 0; i < constrained_path.size() - 1; i++) {
      auto [prev_point, prev_time] = constrained_path[i];
      auto [next_point, next_time] = constrained_path[i + 1];

      // check if temporal constraint is satisfied
      if (prev_time > to_time || from_time >= next_time) continue;
      // check if spatial constraint is satisfied
      if (calculateDistance(from_point, prev_point) >=
          expand_distance + radius + calculateDistance(prev_point, next_point) + constrained_radius)
        continue;

      // set check time
      double start_time = -1.0;
      double end_time = -1.0;
      if (prev_time <= from_time && next_time <= to_time) {
        start_time = from_time;
        end_time = next_time;
      } else if (prev_time <= from_time && next_time >= to_time) {
        start_time = from_time;
        end_time = to_time;
      } else if (prev_time >= from_time && next_time <= to_time) {
        start_time = prev_time;
        end_time = next_time;
      } else if (prev_time >= from_time && next_time >= to_time) {
        start_time = prev_time;
        end_time = to_time;
      } else {
        assert(false);
      }

      auto curr_time = start_time;
      while (curr_time < end_time) {
        // get point at start_time
        const auto occupied_expand_time = curr_time - prev_time;
        assert(occupied_expand_time >= 0.0);
        const auto occupied_theta =
            atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));

        auto occupied_point = prev_point;
        if (occupied_theta != 0.0) {
          occupied_point =
              make_tuple(get<0>(prev_point) + expand_distance * cos(occupied_theta),
                         get<1>(prev_point) + expand_distance * sin(occupied_theta));
        }

        // get point2 at start_time
        const auto expand_time = curr_time - from_time;
        assert(expand_time >= 0.0);
        const auto theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

        auto point = from_point;
        if (theta != 0.0) {
          point = make_tuple(get<0>(from_point) + expand_distance * cos(theta),
                             get<1>(from_point) + expand_distance * sin(theta));
        }

        if (calculateDistance(point, occupied_point) < radius + constrained_radius + env.epsilon) {
          return true;
        }

        curr_time += 1.0;
      }
    }
  }
  return false;
}

bool ConstraintTable::targetConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                                        double to_time, double radius) const {
  vector<Point> interpolated_points;
  vector<double> interpolated_times;
  interpolatePointTime(agent_id, from_point, to_point, from_time, to_time, interpolated_points, interpolated_times);
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (path_table[occupied_agent_id].empty()) continue;
    // target conflict
    auto [last_point, last_time] = path_table[occupied_agent_id].back();
    // check if temporal constraint is satisfied
    if (last_time >= to_time) continue;

    for (int i = 0; i < interpolated_points.size(); ++i) {
      if (last_time > interpolated_times[i]) continue;
      if (calculateDistance(last_point, interpolated_points[i]) < radius + env.radii[occupied_agent_id] + env.epsilon) {
        return true;
      }
    }
  }
  return false;
}

// THIS FUNCTION IS FOR PRIORITIZED PLANNING
void ConstraintTable::getSafeIntervalTablePath(int agent_id, const Point& to_point, double radius,
                                               vector<Interval>& safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    bool is_safe = true;
    double collision_start_time = 0.0;
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(occupied_agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
                           interpolated_times);
      for (int j = 0; j < interpolated_points.size(); ++j) {
        if (is_safe && calculateDistance(to_point, interpolated_points[j]) < radius + env.radii[occupied_agent_id] + env.epsilon) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (!is_safe && calculateDistance(to_point, interpolated_points[j]) >= radius + env.radii[occupied_agent_id] + env.epsilon) {
          is_safe = true;
          assert(collision_start_time < interpolated_times[j]);
          insertCollisionIntervalToSIT(safe_intervals, collision_start_time, interpolated_times[j]);
          if (safe_intervals.empty()) return;
        }
      }
    }
    if (!is_safe) {  // target conflict
      insertCollisionIntervalToSIT(safe_intervals, collision_start_time, numeric_limits<double>::infinity());
      if (safe_intervals.empty()) return;
    }
  }
}

void ConstraintTable::getSafeIntervalTable(int agent_id, const Point& to_point, double radius,
                                           vector<Interval>& safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::infinity());
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    bool is_safe = true;
    double collision_start_time = 0.0;

    for (int i = 0; i < constrained_path.size() - 1; ++i) {
      auto [prev_point, prev_time] = constrained_path[i];
      auto [next_point, next_time] = constrained_path[i + 1];

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
                           interpolated_times);
      for (int j = 0; j < interpolated_points.size(); ++j) {
        if (calculateDistance(to_point, interpolated_points[j]) < radius + constrained_radius + env.epsilon & is_safe) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (calculateDistance(to_point, interpolated_points[j]) >= radius + constrained_radius + env.epsilon &
                   !is_safe) {
          is_safe = true;
          assert(collision_start_time < interpolated_times[j]);
          insertCollisionIntervalToSIT(safe_intervals, collision_start_time - env.time_resolution,
                                    interpolated_times[j] + env.time_resolution);
          if (safe_intervals.empty()) return;
        }
      }
    }
    if (!is_safe) {
      insertCollisionIntervalToSIT(safe_intervals, collision_start_time - env.time_resolution,
                                get<1>(constrained_path.back()) + env.time_resolution);
      if (safe_intervals.empty()) return;
    }
  }
}

double ConstraintTable::getEarliestArrivalTime(int agent_id, const Point& from_point, const Point& to_point,
                                               double expand_time, double lower_bound, double upper_bound,
                                               double radius) const {
  double earliest_arrival_time = lower_bound;
  double from_time = earliest_arrival_time - expand_time;
  while (earliest_arrival_time < upper_bound) {
    if (env.algorithm == "pp") {
      if (targetConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius)) return -1.0;
      if (!pathConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
        return earliest_arrival_time;
    } else if (env.algorithm == "cbs") {
      if (!hardConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
        return earliest_arrival_time;
    }
    earliest_arrival_time += 1.0;
    from_time += 1.0;
  }
  return -1.0;
}

void ConstraintTable::insertCollisionIntervalToSIT(vector<Interval>& safe_intervals, double t_min, double t_max) const {
  assert(t_min >= 0 && t_min < t_max && !safe_intervals.empty());

  int i = 0;
  while (i < safe_intervals.size()) {
    if (t_min >= safe_intervals[i].second) {
      // Collision interval is after the current safe interval
      ++i;
      continue;
    }
    if (t_max <= safe_intervals[i].first) {
      // Collision interval is before the current safe interval
      break;
    }

    if (t_min <= safe_intervals[i].first && t_max >= safe_intervals[i].second) {
      // Collision interval completely covers the current safe interval
      safe_intervals.erase(safe_intervals.begin() + i);
    } else if (t_min <= safe_intervals[i].first && t_max < safe_intervals[i].second) {
      // Collision interval covers the beginning of the current safe interval
      safe_intervals[i].first = t_max;
      ++i;
    } else if (t_min > safe_intervals[i].first && t_max >= safe_intervals[i].second) {
      // Collision interval covers the end of the current safe interval
      safe_intervals[i].second = t_min;
      ++i;
    } else if (t_min > safe_intervals[i].first && t_max < safe_intervals[i].second) {
      // Collision interval covers the middle of the current safe interval
      safe_intervals.insert(safe_intervals.begin() + i + 1, {t_max, safe_intervals[i].second});
      safe_intervals[i].second = t_min;
      ++i;
    }
  }
}

void ConstraintTable::interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                                       vector<Point>& interpolated_points) const {
  const double velocity = calculateDistance(from_point, to_point);
  const auto expand_time = 1.0;
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

  double elapsed_time = 0.0;
  while (elapsed_time < expand_time / env.time_resolution) {
    Point interpolated_point = from_point;
    if (theta != 0.0) {
      interpolated_point = make_tuple(get<0>(from_point) + velocity * cos(theta) * elapsed_time,
                                      get<1>(from_point) + velocity * sin(theta) * elapsed_time);
    }
    interpolated_points.emplace_back(interpolated_point);
    elapsed_time += env.time_resolution;
  }

  assert(!interpolated_points.empty());
}

void ConstraintTable::interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point,
                                           double from_time, double to_time, vector<Point>& interpolated_points,
                                           vector<double>& interpolated_times) const {
  assert(from_time < to_time);
  const double velocity = calculateDistance(from_point, to_point);
  const auto expand_time = 1.0;
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

  double elapsed_time = 0.0;
  while (elapsed_time < to_time - from_time) {
    Point interpolated_point = from_point;
    if (theta != 0.0) {
      interpolated_point = make_tuple(get<0>(from_point) + velocity * cos(theta) * elapsed_time,
                                      get<1>(from_point) + velocity * sin(theta) * elapsed_time);
    }
    interpolated_points.emplace_back(interpolated_point);
    interpolated_times.emplace_back(from_time + elapsed_time);
    elapsed_time += env.time_resolution;
  }

  assert(!interpolated_points.empty());
  assert(interpolated_points.size() == interpolated_times.size());
}
