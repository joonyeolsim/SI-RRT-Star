#include "ConstraintTable.h"

bool ConstraintTable::obstacleConstrained(int agent_id, const Point& from_point, const Point& to_point, double radius) const {
  vector<Point> interpolated_points;
  interpolatePoint(agent_id, from_point, to_point, interpolated_points);
  for (auto& interpolated_point : interpolated_points) {
    for (const auto& obstacle : env.obstacles) {
      if (obstacle->constrained(interpolated_point, radius)) return true;
    }
  }
  return false;
}
                                                     
bool ConstraintTable::targetConstrained(int agent_id, const Point& point, int timestep, double radius) const {
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    // continue if the agent is the same
    if (occupied_agent_id == agent_id) continue;
    // continue if the agent has not started
    if (path_table[occupied_agent_id].empty()) break;
    // target conflict
    auto [last_occupied_point, last_occupied_timestep] = path_table[occupied_agent_id].back();
    // check if temporal constraint is satisfied
    if (last_occupied_timestep > timestep) continue;

    if (calculateDistance(last_occupied_point, point) < radius + env.radii[occupied_agent_id]) {
      return true;
    }
  }
  return false;
}

bool ConstraintTable::pathConstrained(int agent_id, const Point& point, int timestep, double radius) const {
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    // continue if the agent is the same
    if (occupied_agent_id == agent_id) continue;
    // continue if the size of the path is less than to_time_step
    if (path_table[occupied_agent_id].size() <= timestep) continue;
    // vertex-edge conflict
    auto [occupied_point, occupied_timestep] = path_table[occupied_agent_id][timestep];

    if (calculateDistance(point, occupied_point) < radius + env.radii[occupied_agent_id]) {
      return true;
    }
  }
  return false;
}

// TODO: Modify this function
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

        if (calculateDistance(point, occupied_point) < radius + constrained_radius) {
          return true;
        }

        curr_time += 1;
      }
    }
  }
  return false;
}

// THIS FUNCTION IS FOR PRIORITIZED PLANNING
void ConstraintTable::getSafeIntervalTablePath(int agent_id, const Point& point, double radius, vector<Interval>& safe_intervals) const {
  assert(safe_intervals.empty());
  safe_intervals.emplace_back(0, numeric_limits<int>::max() / 2);
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    // continue if the agent is the same
    if (occupied_agent_id == agent_id) continue;
    // continue if the agent has not started
    if (path_table[occupied_agent_id].empty()) continue;
    for (int i = 0; i < path_table[occupied_agent_id].size(); ++i) {
      auto [occupied_point, occupied_timestep] = path_table[occupied_agent_id][i];
      assert(occupied_timestep == i);

      if (calculateDistance(point, occupied_point) < radius + env.radii[occupied_agent_id]) {
        insertCollisionIntervalToSIT(safe_intervals, occupied_timestep, occupied_timestep + 1);
        if (safe_intervals.empty()) return;
      }
    }
    // target conflict
    auto [last_point, last_time_step] = path_table[occupied_agent_id].back();
    if (calculateDistance(point, last_point) < radius + env.radii[occupied_agent_id]) {
      insertCollisionIntervalToSIT(safe_intervals, last_time_step, numeric_limits<int>::max() / 2);
      if (safe_intervals.empty()) return;
    }
  }
}

// TODO: Modify this function
void ConstraintTable::getSafeIntervalTable(int agent_id, const Point& to_point, double radius,
                                           vector<Interval>& safe_intervals) const {
  assert(safe_intervals.empty());
  // safe_intervals.emplace_back(0.0, numeric_limits<double>::max());
  // for (auto [constrained_radius, constrained_path] : hard_constraint_table[agent_id]) {
  //   bool is_safe = true;
  //   double collision_start_time = 0.0;
  //
  //   for (int i = 0; i < constrained_path.size() - 1; ++i) {
  //     auto [prev_point, prev_time] = constrained_path[i];
  //     auto [next_point, next_time] = constrained_path[i + 1];
  //
  //     vector<Point> interpolated_points;
  //     vector<double> interpolated_times;
  //     interpolatePointTime(agent_id, prev_point, next_point, prev_time, next_time, interpolated_points,
  //                          interpolated_times);
  //     for (int j = 0; j < interpolated_points.size(); ++j) {
  //       if (calculateDistance(to_point, interpolated_points[j]) < radius + constrained_radius + env.epsilon & is_safe) {
  //         is_safe = false;
  //         collision_start_time = interpolated_times[j];
  //       } else if (calculateDistance(to_point, interpolated_points[j]) >= radius + constrained_radius + env.epsilon &
  //                  !is_safe) {
  //         is_safe = true;
  //         assert(collision_start_time < interpolated_times[j]);
  //         insertToSafeIntervalTable(safe_intervals, collision_start_time - env.time_resolution,
  //                                   interpolated_times[j] + env.time_resolution);
  //         if (safe_intervals.empty()) return;
  //       }
  //     }
  //   }
  //   if (!is_safe) {
  //     insertToSafeIntervalTable(safe_intervals, collision_start_time - env.time_resolution,
  //                               get<1>(constrained_path.back()) + env.time_resolution);
  //     if (safe_intervals.empty()) return;
  //   }
  // }
}

int ConstraintTable::getEarliestArrivalTime(int agent_id, const Point& from_point, const Point& to_point, int lower_bound, int upper_bound, double radius) const {
  for (int earliest_arrival_time = lower_bound; earliest_arrival_time < upper_bound; ++earliest_arrival_time) {
    if (env.algorithm == "pp") {
      if (targetConstrained(agent_id, to_point, earliest_arrival_time, radius)) return -1;
      if (!pathConstrained(agent_id, to_point, earliest_arrival_time, radius)) {
        return earliest_arrival_time;
      }
    } else if (env.algorithm == "cbs") {
      if (!hardConstrained(agent_id, from_point, to_point, earliest_arrival_time - 1, earliest_arrival_time, radius)) {
        return earliest_arrival_time;
      }
    }
  }
  return -1;
}

void ConstraintTable::insertCollisionIntervalToSIT(vector<Interval>& safe_intervals, int t_min, int t_max) const {
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
      safe_intervals.insert(safe_intervals.begin() + i + 1, std::make_pair(t_max, safe_intervals[i].second));
      safe_intervals[i].second = t_min;
      ++i;
    }
  }
}

void ConstraintTable::interpolatePoint(const int agent_id, const Point& from_point, const Point& to_point, vector<Point>& interpolated_points) const {
  interpolated_points.clear();
  int interpolation_step = env.max_expand_distances[agent_id] / env.max_distances_per_timestep[agent_id];
  for (int step = 0; step < interpolation_step; ++step) {
    double ratio = static_cast<double>(step) / interpolation_step;
    Point interpolated_point = make_tuple(
      get<0>(from_point) + ratio * (get<0>(to_point) - get<0>(from_point)),
      get<1>(from_point) + ratio * (get<1>(to_point) - get<1>(from_point))
    );
    interpolated_points.emplace_back(interpolated_point);
  }

  assert(interpolated_points.size() == env.interpolation_step);
}
