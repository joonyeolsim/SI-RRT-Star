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
  const double expand_distance = calculateDistance(from_point, to_point);
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];

      // check if temporal constraint is satisfied
      if (prev_time > to_time || from_time >= next_time) continue;
      // check if spatial constraint is satisfied
      const Point center1 =
          make_tuple((get<0>(from_point) + get<0>(to_point)) / 2.0, (get<1>(from_point) + get<1>(to_point)) / 2.0);
      const double radius1 = calculateDistance(from_point, to_point) / 2.0 + env.radii[agent_id];
      const Point center2 =
          make_tuple((get<0>(prev_point) + get<0>(next_point)) / 2.0, (get<1>(prev_point) + get<1>(next_point)) / 2.0);
      const double radius2 = calculateDistance(prev_point, next_point) / 2.0 + env.radii[occupied_agent_id];
      if (calculateDistance(center1, center2) > radius1 + radius2) continue;

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
          occupied_point = make_tuple(
              get<0>(prev_point) + env.velocities[occupied_agent_id] * cos(occupied_theta) * occupied_expand_time,
              get<1>(prev_point) + env.velocities[occupied_agent_id] * sin(occupied_theta) * occupied_expand_time);
        }

        // get point2 at start_time
        const auto expand_time = curr_time - from_time;
        assert(expand_time >= 0.0);
        const auto theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

        auto point = from_point;
        if (theta != 0.0) {
          point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                             get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
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
              make_tuple(get<0>(prev_point) + env.velocities[agent_id] * cos(occupied_theta) * occupied_expand_time,
                         get<1>(prev_point) + env.velocities[agent_id] * sin(occupied_theta) * occupied_expand_time);
        }

        // get point2 at start_time
        const auto expand_time = curr_time - from_time;
        assert(expand_time >= 0.0);
        const auto theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

        auto point = from_point;
        if (theta != 0.0) {
          point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                             get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
        }

        if (calculateDistance(point, occupied_point) < radius + constrained_radius + env.epsilon) {
          return true;
        }

        curr_time += env.time_resolution;
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
    if (last_time > to_time) continue;
    // check if spatial constraint is satisfied
    const Point center1 =
        make_tuple((get<0>(from_point) + get<0>(to_point)) / 2.0, (get<1>(from_point) + get<1>(to_point)) / 2.0);
    const double radius1 = calculateDistance(from_point, to_point) / 2.0 + env.radii[agent_id];
    if (calculateDistance(center1, last_point) > radius1 + env.radii[occupied_agent_id]) continue;

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
  safe_intervals.emplace_back(0.0, numeric_limits<double>::max());
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
        if (calculateDistance(to_point, interpolated_points[j]) < radius + env.radii[occupied_agent_id] + env.epsilon &
            is_safe) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (calculateDistance(to_point, interpolated_points[j]) >=
                       radius + env.radii[occupied_agent_id] + env.epsilon &
                   !is_safe) {
          is_safe = true;
          assert(collision_start_time < interpolated_times[j]);
          insertToSafeIntervalTable(safe_intervals, collision_start_time - env.time_resolution,
                                    interpolated_times[j] + env.time_resolution);
          if (safe_intervals.empty()) return;
        }
      }
    }
    if (!is_safe) {  // target conflict
      insertToSafeIntervalTable(safe_intervals, collision_start_time - env.time_resolution,
                                numeric_limits<double>::max());
      if (safe_intervals.empty()) return;
    }
  }
}

void ConstraintTable::getSafeIntervalTable(int agent_id, const Point& to_point, double radius,
                                           vector<Interval>& safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0.0, numeric_limits<double>::max());
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
          insertToSafeIntervalTable(safe_intervals, collision_start_time - env.time_resolution,
                                    interpolated_times[j] + env.time_resolution);
          if (safe_intervals.empty()) return;
        }
      }
    }
    if (!is_safe) {
      insertToSafeIntervalTable(safe_intervals, collision_start_time - env.time_resolution,
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
  while (earliest_arrival_time + env.epsilon < upper_bound) {
    if (env.algorithm == "pp") {
      if (targetConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius)) return -1.0;
      if (!pathConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
        return earliest_arrival_time;
    } else if (env.algorithm == "cbs") {
      if (!hardConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
        return earliest_arrival_time;
    }
    earliest_arrival_time += env.time_resolution;
    from_time += env.time_resolution;
  }
  return -1.0;
}

void ConstraintTable::insertToSafeIntervalTable(vector<Interval>& safe_intervals, double t_min, double t_max) const {
  assert(t_min >= 0.0 and t_min < t_max and !safe_intervals.empty());
  for (int i = 0; i < safe_intervals.size(); ++i) {
    if (t_min > get<1>(safe_intervals[i])) continue;
    if (t_max < get<0>(safe_intervals[i])) break;
    if (t_min <= get<0>(safe_intervals[i]) && t_max >= get<1>(safe_intervals[i])) {
      safe_intervals.erase(safe_intervals.begin() + i);
    } else if (t_min <= get<0>(safe_intervals[i]) && t_max < get<1>(safe_intervals[i])) {
      get<0>(safe_intervals[i]) = t_max;
    } else if (t_min > get<0>(safe_intervals[i]) && t_max >= get<1>(safe_intervals[i])) {
      get<1>(safe_intervals[i]) = t_min;
    } else if (t_min > get<0>(safe_intervals[i]) && t_max < get<1>(safe_intervals[i])) {
      safe_intervals.insert(safe_intervals.begin() + i + 1, make_tuple(t_max, get<1>(safe_intervals[i])));
      get<1>(safe_intervals[i]) = t_min;
    }
  }
}

void ConstraintTable::interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                                       vector<Point>& interpolated_points) const {
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  interpolated_points.emplace_back(from_point);

  const auto timesteps = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));
  for (int timestep = 1; timestep < timesteps; ++timestep) {
    Point interpoated_point = from_point;
    if (theta != 0.0) {
      interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * timestep,
                                     get<1>(from_point) + env.velocities[agent_id] * sin(theta) * timestep);
    }
    interpolated_points.emplace_back(interpoated_point);
  }
  interpolated_points.emplace_back(to_point);

  assert(!interpolated_points.empty());
}

void ConstraintTable::interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point,
                                           double from_time, double to_time, vector<Point>& interpolated_points,
                                           vector<double>& interpoated_times) const {
  assert(from_time < to_time);
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  interpolated_points.emplace_back(from_point);
  interpoated_times.emplace_back(from_time);

  const auto timesteps = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));
  for (int timestep = 1; timestep < timesteps; ++timestep) {
    Point interpoated_point = from_point;
    if (theta != 0.0) {
      interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * timestep,
                                     get<1>(from_point) + env.velocities[agent_id] * sin(theta) * timestep);
    }
    interpolated_points.emplace_back(interpoated_point);
    interpoated_times.emplace_back(from_time + timestep);
  }
  interpolated_points.emplace_back(to_point);
  interpoated_times.emplace_back(to_time);

  assert(!interpolated_points.empty());
  assert(interpolated_points.size() == interpoated_times.size());
}