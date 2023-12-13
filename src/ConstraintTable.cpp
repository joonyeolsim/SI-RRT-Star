//
// Created by joonyeol on 23. 12. 8.
//

#include "ConstraintTable.h"

void ConstraintTable::insertPathToConstraint(int agent_id, Path path) { path_table[agent_id] = path; }

bool ConstraintTable::obstacleConstrained(const Point& other_point, double other_radius) const {
  return any_of(env.obstacles.begin(), env.obstacles.end(),
                [&](const shared_ptr<Obstacle>& obstacle) { return obstacle->constrained(other_point, other_radius); });
}

bool ConstraintTable::pathConstrained(const Point& other_point, double other_time, double other_radius) const {
  for (auto agent_id = 0; agent_id < path_table.size(); ++agent_id) {
    for (auto& path : path_table) {
      if (path.empty()) continue;
      // vertex-edge conflict
      for (int i = 0; i < path.size() - 1; ++i) {
        auto [prev_point, prev_time] = path[i];
        auto [next_point, next_time] = path[i + 1];
        // check time is matched
        if (prev_time > other_time || other_time >= next_time) continue;
        // check point is matched
        const auto expand_time = other_time - prev_time;
        const auto theta = atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
        const auto interpolated_point =
            make_tuple(get<0>(prev_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                       get<1>(prev_point) + env.velocities[agent_id] * sin(theta) * expand_time);
        if (calculateDistance(interpolated_point, other_point) < other_radius + env.radii[agent_id]) return true;
      }

      // target conflict
      auto [last_point, last_time] = path.back();
      if (last_time > other_time) continue;
      if (calculateDistance(last_point, other_point) < other_radius + env.radii[agent_id]) {
        return true;
      }
    }
  }
  return false;
}

bool ConstraintTable::targetConstrained(const Point& other_point, double other_time, double other_radius) const {
  for (auto agent_id = 0; agent_id < path_table.size(); ++agent_id) {
    for (auto& path : path_table) {
      if (path.empty()) continue;
      // vertex-edge conflict
      for (int i = 0; i < path.size() - 1; ++i) {
        auto [prev_point, prev_time] = path[i];
        auto [next_point, next_time] = path[i + 1];
        // check time is matched
        if (other_time >= next_time) continue;

        const double expand_distance = calculateDistance(prev_point, next_point);
        const double theta = atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
        const double expand_time = expand_distance / env.velocities[agent_id];

        const int timestep = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));

        for (int time = 0; time < timestep; ++time) {
          const auto interpolated_point = make_tuple(get<0>(prev_point) + env.velocities[agent_id] * cos(theta) * time,
                                                     get<1>(prev_point) + env.velocities[agent_id] * sin(theta) * time);
          if (calculateDistance(interpolated_point, other_point) < other_radius + env.radii[agent_id]) {
            return true;
          }
        }

        const double remain_time = fmod(expand_distance, env.velocities[agent_id]);
        if (remain_time > 0.0) {
          const auto interpolated_point =
              make_tuple(get<0>(prev_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                         get<1>(prev_point) + env.velocities[agent_id] * sin(theta) * expand_time);
          if (calculateDistance(interpolated_point, other_point) < other_radius + env.radii[agent_id]) {
            return true;
          }
        }
      }
    }
  }
  return false;
}