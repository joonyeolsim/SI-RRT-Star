#include "ConstraintTable.h"

void ConstraintTable::updateSoftConstraint(int agent_id, Path path) {
  soft_constraint_table[agent_id].clear();

  Path interpolated_path;

  int index = 0;
  auto [prev_point, prev_time] = path[index];
  auto [next_point, next_time] = path[index + 1];

  const double max_path_time = get<1>(path.back());

  double time = 0.0;
  while (time < max_path_time) {
    if (time >= next_time) {
      index++;
      prev_point = get<0>(path[min(static_cast<int>(path.size()) - 1, index)]);
      prev_time = get<1>(path[min(static_cast<int>(path.size()) - 1, index)]);
      next_point = get<0>(path[min(static_cast<int>(path.size()) - 1, index + 1)]);
      next_time = get<1>(path[min(static_cast<int>(path.size()) - 1, index + 1)]);
    }

    const auto theta = atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
    auto point = prev_point;
    const auto expand_time = time - prev_time;
    if (theta != 0.0) {
      point = make_tuple(get<0>(prev_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                         get<1>(prev_point) + env.velocities[agent_id] * sin(theta) * expand_time);
    }
    interpolated_path.emplace_back(point, time);
    time += 1.0;
  }
  const double remain_time = fmod(max_path_time, 1.0);
  if (remain_time > env.epsilon) {
    interpolated_path.emplace_back(path.back());
  }
  soft_constraint_table[agent_id].emplace_back(env.radii[agent_id], interpolated_path);
}

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

// THIS FUNCTION IS FOR PRIORITIZED PLANNING
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
      if (prev_time > to_time || from_time > next_time) continue;
      // check if spatial constraint is satisfied
      if (calculateDistance(prev_point, to_point) >=
          calculateDistance(prev_point, next_point) + radius + expand_distance + env.radii[occupied_agent_id])
        continue;

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(agent_id, from_point, to_point, from_time, to_time, interpolated_points, interpolated_times);
      for (int j = 0; j < interpolated_points.size(); ++j) {
        const auto occupied_expand_time = interpolated_times[j] - prev_time;
        const auto occupied_theta =
            atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));

        auto occupied_point = prev_point;
        if (occupied_theta != 0.0) {
          occupied_point = make_tuple(
              get<0>(prev_point) + env.velocities[occupied_agent_id] * cos(occupied_theta) * occupied_expand_time,
              get<1>(prev_point) + env.velocities[occupied_agent_id] * sin(occupied_theta) * occupied_expand_time);
        }
        if (calculateDistance(interpolated_points[j], occupied_point) < radius + env.radii[occupied_agent_id]) {
          return true;
        }
      }
    }
  }
  return false;
}

bool ConstraintTable::hardConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                                      double to_time, double radius) const {
  assert(from_time < to_time);
  // vertex-edge conflict
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    for (auto [constrained_point, constrained_time] : constrained_path) {
      // check if temporal constraint is satisfied
      if (constrained_time < from_time - env.epsilon) continue;
      if (constrained_time > to_time + env.epsilon) break;

      const auto occupied_expand_time = constrained_time - from_time;
      const auto occupied_theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

      auto occupied_point = from_point;
      if (occupied_theta != 0.0) {
        occupied_point =
            make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(occupied_theta) * occupied_expand_time,
                       get<1>(from_point) + env.velocities[agent_id] * sin(occupied_theta) * occupied_expand_time);
      }
      if (calculateDistance(occupied_point, constrained_point) < radius + constrained_radius) {
        return true;
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

  for (int i = 0; i < interpolated_points.size(); ++i) {
    for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
      if (path_table[occupied_agent_id].empty()) continue;
      // target conflict
      auto [last_point, last_time] = path_table[occupied_agent_id].back();
      // check if temporal constraint is satisfied
      if (last_time > interpolated_times[i]) continue;
      if (calculateDistance(last_point, interpolated_points[i]) < radius + env.radii[occupied_agent_id]) {
        return true;
      }
    }
  }
  return false;
}

bool ConstraintTable::softConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
                                      double to_time, double radius) const {
  assert(from_time < to_time);
  // vertex-edge conflict
  for (int other_agent_id = 0; other_agent_id < soft_constraint_table.size(); ++other_agent_id) {
    if (other_agent_id == agent_id) continue;
    for (auto [constrained_radius, constrained_path] : soft_constraint_table[other_agent_id]) {
      for (auto [constrained_point, constrained_time] : constrained_path) {
        // check if temporal constraint is satisfied
        if (constrained_time < from_time - env.epsilon) continue;
        if (constrained_time > to_time + env.epsilon) break;

        const auto occupied_expand_time = constrained_time - from_time;
        const auto occupied_theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));

        auto occupied_point = from_point;
        if (occupied_theta != 0.0) {
          occupied_point =
              make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(occupied_theta) * occupied_expand_time,
                         get<1>(from_point) + env.velocities[agent_id] * sin(occupied_theta) * occupied_expand_time);
        }
        if (calculateDistance(occupied_point, constrained_point) < radius + constrained_radius) {
          return true;
        }
      }
    }
  }
  return false;
}

// bool futureSoftConstrained() {
//
// }

// THIS FUNCTION IS FOR PRIORITIZED PLANNING
void ConstraintTable::getSafeIntervalTablePath(int agent_id, const Point& to_point, double radius,
                                               vector<Interval>& safe_intervals) const {
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

      // check if spatial constraint is satisfied
      if (calculateDistance(prev_point, to_point) >=
          calculateDistance(prev_point, next_point) + radius + env.radii[occupied_agent_id])
        continue;

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(occupied_agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
                           interpolated_times);
      for (int j = 0; j < interpolated_points.size(); ++j) {
        if (calculateDistance(to_point, interpolated_points[j]) < radius + env.radii[occupied_agent_id] & is_safe) {
          is_safe = false;
          collision_start_time = interpolated_times[j];
        } else if (calculateDistance(to_point, interpolated_points[j]) >= radius + env.radii[occupied_agent_id] &
                   !is_safe) {
          is_safe = true;
          assert(collision_start_time < interpolated_times[j]);
          insertToSafeIntervalTable(safe_intervals, collision_start_time, interpolated_times[j]);
          if (safe_intervals.empty()) return;
        }
      }
    }
    if (!is_safe) {  // target conflict
      insertToSafeIntervalTable(safe_intervals, collision_start_time, numeric_limits<double>::max());
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

    for (auto [constrained_point, constrained_time] : constrained_path) {
      if (calculateDistance(constrained_point, to_point) < radius + constrained_radius & is_safe) {
        is_safe = false;
        collision_start_time = constrained_time;
      } else if (calculateDistance(constrained_point, to_point) >= radius + constrained_radius & !is_safe) {
        is_safe = true;
        assert(collision_start_time < constrained_time);
        insertToSafeIntervalTable(safe_intervals, collision_start_time, constrained_time + 1.0);
        assert(!safe_intervals.empty());
      }
    }
    if (!is_safe) {
      // TODO : check if 1.0 is correct
      insertToSafeIntervalTable(safe_intervals, collision_start_time, get<1>(constrained_path.back()) + 1.0);
      assert(!safe_intervals.empty());
    }
  }
}

// double ConstraintTable::getEarliestArrivalTime(int agent_id, const Point& from_point, const Point& to_point,
//                                                double expand_time, double lower_bound, double upper_bound,
//                                                double radius) const {
//   double earliest_arrival_time = lower_bound;
//   double from_time = earliest_arrival_time - expand_time;
//   while (earliest_arrival_time < upper_bound) {
//     if (!hardConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
//       return earliest_arrival_time;
//     if (targetConstrained(agent_id, from_point, to_point, earliest_arrival_time, radius)) return -1.0;
//     earliest_arrival_time += 1.0;
//     from_time += 1.0;
//   }
//   return -1.0;
// }

double ConstraintTable::getEarliestArrivalTime(int agent_id, const Point& from_point, const Point& to_point,
                                               double expand_time, double lower_bound, double upper_bound,
                                               double radius) const {
  double earliest_arrival_time = lower_bound;
  double from_time = earliest_arrival_time - expand_time;
  while (earliest_arrival_time < upper_bound) {
    if (targetConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius)) return -1.0;
    if (!pathConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
      return earliest_arrival_time;
    earliest_arrival_time += 1.0;
    from_time += 1.0;
  }
  return -1.0;
}

double ConstraintTable::getEarliestArrivalTimeSoft(int agent_id, const Point& from_point, const Point& to_point,
                                                   double expand_time, double lower_bound, double upper_bound,
                                                   double radius) const {
  double earliest_arrival_time = lower_bound;
  double from_time = earliest_arrival_time - expand_time;
  while (earliest_arrival_time < upper_bound) {
    if (!softConstrained(agent_id, from_point, to_point, from_time, earliest_arrival_time, radius))
      return earliest_arrival_time;
    earliest_arrival_time += 1.0;
    from_time += 1.0;
  }
  return -1.0;
}

void ConstraintTable::insertToSafeIntervalTable(vector<Interval>& safe_intervals, double t_min, double t_max) {
  assert(t_min >= 0.0 and t_min < t_max and !safe_intervals.empty());
  for (int i = 0; i < safe_intervals.size(); ++i) {
    if (t_min > get<1>(safe_intervals[i])) continue;
    if (t_max < get<0>(safe_intervals[i])) break;
    if (t_min <= get<0>(safe_intervals[i]) && t_max >= get<1>(safe_intervals[i])) {
      safe_intervals.erase(safe_intervals.begin() + i);
    }
    if (t_min <= get<0>(safe_intervals[i]) && t_max < get<1>(safe_intervals[i])) {
      get<0>(safe_intervals[i]) = t_max;
    }
    if (t_min > get<0>(safe_intervals[i]) && t_max >= get<1>(safe_intervals[i])) {
      get<1>(safe_intervals[i]) = t_min;
    }
    if (t_min > get<0>(safe_intervals[i]) && t_max < get<1>(safe_intervals[i])) {
      safe_intervals.emplace_back(t_max, get<1>(safe_intervals[i]));
      get<1>(safe_intervals[i]) = t_min;
    }
  }
}

// other_point가 other_time에서 멈추어도 되는지..
// THIS FUNCTION IS FOR PRIORITIZED PLANNING
bool ConstraintTable::targetConstrainedPath(const Point& other_point, double other_time, double other_radius) const {
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];
      // check if temporal constraint is satisfied
      if (other_time >= next_time) continue;
      // check if spatial constraint is satisfied
      if (calculateDistance(prev_point, other_point) >=
          calculateDistance(prev_point, next_point) + other_radius + env.radii[occupied_agent_id])
        continue;

      vector<Point> interpolated_points;
      vector<double> interpolated_times;
      interpolatePointTime(occupied_agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
                           interpolated_times);
      for (int j = 0; j < interpolated_points.size(); ++j) {
        if (interpolated_times[j] < other_time) continue;
        if (calculateDistance(interpolated_points[j], other_point) < other_radius + env.radii[occupied_agent_id]) {
          return true;
        }
      }
    }
  }
  return false;
}

// other_point가 other_time에서 멈추어도 되는지..
bool ConstraintTable::targetConstrained(int agent_id, const Point& other_point, double other_time,
                                        double other_radius) const {
  // vertex-edge conflict
  for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
    for (auto [constrained_point, constrained_time] : constrained_path) {
      // check if temporal constraint is satisfied
      if (other_time > constrained_time) continue;
      if (calculateDistance(constrained_point, other_point) < other_radius + constrained_radius) {
        return true;
      }
    }
  }
  return false;
}

void ConstraintTable::interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                                       vector<Point>& interpolated_points) const {
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const auto timesteps = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));
  for (int timestep = 0; timestep < timesteps; ++timestep) {
    Point interpoated_point = from_point;
    if (theta != 0.0) {
      interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * timestep,
                                     get<1>(from_point) + env.velocities[agent_id] * sin(theta) * timestep);
    }
    interpolated_points.emplace_back(interpoated_point);
  }
  const double remain_time = fmod(expand_distance, env.velocities[agent_id]);
  if (remain_time > env.epsilon) {
    Point interpoated_point = from_point;
    if (theta != 0.0) {
      interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                     get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
    }
    interpolated_points.emplace_back(interpoated_point);
  }

  if (interpolated_points.empty()) {
    interpolated_points.emplace_back(from_point);
  }

  assert(!interpolated_points.empty());
  assert(calculateDistance(interpolated_points.back(), from_point) < env.max_expand_distances[agent_id] + env.epsilon);
  assert(calculateDistance(interpolated_points.front(), to_point) < env.max_expand_distances[agent_id] + env.epsilon);
}

void ConstraintTable::interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point,
                                           double from_time, double to_time, vector<Point>& interpolated_points,
                                           vector<double>& interpoated_times) const {
  assert(from_time < to_time);
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const auto timesteps = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));
  for (int timestep = 0; timestep < timesteps; ++timestep) {
    Point interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * timestep,
                                         get<1>(from_point) + env.velocities[agent_id] * sin(theta) * timestep);
    interpolated_points.emplace_back(interpoated_point);
    interpoated_times.emplace_back(from_time + timestep);
  }
  if (interpolated_points.empty()) {
    interpolated_points.emplace_back(from_point);
    interpoated_times.emplace_back(from_time);
  }
  const double remain_time = fmod(expand_distance, env.velocities[agent_id]);
  if (remain_time > env.epsilon) {
    Point interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                         get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
    interpolated_points.emplace_back(interpoated_point);
    interpoated_times.emplace_back(to_time);
  }

  assert(!interpolated_points.empty());
  assert(interpolated_points.size() == interpoated_times.size());
  assert(calculateDistance(interpolated_points.back(), from_point) < env.max_expand_distances[agent_id] + env.epsilon);
  assert(calculateDistance(interpolated_points.front(), to_point) < env.max_expand_distances[agent_id] + env.epsilon);
}