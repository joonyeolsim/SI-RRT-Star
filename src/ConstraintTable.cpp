//
// Created by joonyeol on 23. 12. 8.
//

#include "ConstraintTable.h"

void ConstraintTable::insertPathToConstraint(int agent_id, Path path) { path_table[agent_id] = path; }

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

// bool ConstraintTable::pathConstrained(int agent_id, const Point& from_point, const Point& to_point, double from_time,
//                                       double to_time, double radius) const {
//   const double expand_distance = calculateDistance(from_point, to_point);
//   for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
//     if (occupied_agent_id == agent_id) continue;
//     if (path_table[occupied_agent_id].empty()) continue;
//     // vertex-edge conflict
//     for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
//       auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
//       auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];
//
//       // check if temporal constraint is satisfied
//       if (prev_time > to_time || from_time > next_time) continue;
//       // check if spatial constraint is satisfied
//       if (calculateDistance(prev_point, to_point) >=
//           calculateDistance(prev_point, next_point) + radius + expand_distance + env.radii[occupied_agent_id])
//         continue;
//
//       vector<Point> interpolated_points;
//       vector<double> interpolated_times;
//       interpolatePointTime(agent_id, from_point, to_point, from_time, to_time, interpolated_points,
//       interpolated_times); for (int j = 0; j < interpolated_points.size(); ++j) {
//         const auto occupied_expand_time = interpolated_times[j] - prev_time;
//         const auto occupied_theta =
//             atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
//         const auto occupied_point = make_tuple(
//             get<0>(prev_point) + env.velocities[occupied_agent_id] * cos(occupied_theta) * occupied_expand_time,
//             get<1>(prev_point) + env.velocities[occupied_agent_id] * sin(occupied_theta) * occupied_expand_time);
//         if (calculateDistance(interpolated_points[j], occupied_point) < radius + env.radii[occupied_agent_id]) {
//           return true;
//         }
//       }
//     }
//
//     // target conflict
//     auto [last_point, last_time] = path_table[occupied_agent_id].back();
//     // check if temporal constraint is satisfied
//     if (last_time > to_time) continue;
//     // check if spatial constraint is satisfied
//     if (calculateDistance(last_point, to_point) >= radius + expand_distance + env.radii[occupied_agent_id]) continue;
//
//     vector<Point> interpolated_points;
//     interpolatePoint(agent_id, from_point, to_point, interpolated_points);
//     for (const auto& interpolated_point : interpolated_points) {
//       if (calculateDistance(last_point, interpolated_point) < radius + env.radii[agent_id]) {
//         return true;
//       }
//     }
//   }
//   return false;
// }

bool ConstraintTable::pathConstrained(int agent_id, const Point& from_point, const Point& to_point, double to_time,
                                      double radius) const {
  const double expand_distance = calculateDistance(from_point, to_point);
  for (auto occupied_agent_id = 0; occupied_agent_id < path_table.size(); ++occupied_agent_id) {
    if (occupied_agent_id == agent_id) continue;
    if (path_table[occupied_agent_id].empty()) continue;
    // vertex-edge conflict
    for (int i = 0; i < path_table[occupied_agent_id].size() - 1; ++i) {
      auto [prev_point, prev_time] = path_table[occupied_agent_id][i];
      auto [next_point, next_time] = path_table[occupied_agent_id][i + 1];

      // check if spatial constraint is satisfied
      if (calculateDistance(prev_point, to_point) >=
          calculateDistance(prev_point, next_point) + radius + expand_distance + env.radii[occupied_agent_id])
        continue;

      const auto occupied_expand_time = to_time - prev_time;
      const auto occupied_theta =
          atan2(get<1>(next_point) - get<1>(prev_point), get<0>(next_point) - get<0>(prev_point));
      const auto occupied_point = make_tuple(
          get<0>(prev_point) + env.velocities[occupied_agent_id] * cos(occupied_theta) * occupied_expand_time,
          get<1>(prev_point) + env.velocities[occupied_agent_id] * sin(occupied_theta) * occupied_expand_time);
      if (calculateDistance(to_point, occupied_point) < radius + env.radii[occupied_agent_id]) {
        return true;
      }
    }

    // target conflict
    auto [last_point, last_time] = path_table[occupied_agent_id].back();
    // check if temporal constraint is satisfied
    if (last_time > to_time) continue;
    // check if spatial constraint is satisfied
    if (calculateDistance(last_point, to_point) >= radius + expand_distance + env.radii[occupied_agent_id]) continue;

    if (calculateDistance(last_point, to_point) < radius + env.radii[agent_id]) {
      return true;
    }
  }
  return false;
}

// other_point가 other_time에서 멈추어도 되는지..
bool ConstraintTable::targetConstrained(const Point& other_point, double other_time, double other_radius) const {
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

void ConstraintTable::interpolatePoint(int agent_id, const Point& from_point, const Point& to_point,
                                       vector<Point>& interpolated_points) const {
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const auto timestep = static_cast<int>(floor(expand_distance / env.velocities[agent_id]));
  for (int time = 0; time < timestep; ++time) {
    Point interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * time,
                                         get<1>(from_point) + env.velocities[agent_id] * sin(theta) * time);
    interpolated_points.emplace_back(interpoated_point);
  }
  const double remain_time = fmod(expand_distance, env.velocities[agent_id]);
  if (remain_time > 0.0) {
    Point interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                         get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
    interpolated_points.emplace_back(interpoated_point);
  }

  if (interpolated_points.empty()) {
    interpolated_points.emplace_back(from_point);
  }

  assert(calculateDistance(interpolated_points.back(), from_point) <
         env.max_expand_distances[agent_id] + env.threshold);
  assert(calculateDistance(interpolated_points.front(), to_point) < env.max_expand_distances[agent_id] + env.threshold);
}

void ConstraintTable::interpolatePointTime(int agent_id, const Point& from_point, const Point& to_point,
                                           double from_time, double to_time, vector<Point>& interpolated_points,
                                           vector<double>& interpoated_times) const {
  const double expand_distance = calculateDistance(from_point, to_point);
  const double theta = atan2(get<1>(to_point) - get<1>(from_point), get<0>(to_point) - get<0>(from_point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const auto timestep = static_cast<int>(floor(expand_distance / env.velocities[agent_id])) + 1;
  for (int time = 1; time < timestep; ++time) {
    Point interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * time,
                                         get<1>(from_point) + env.velocities[agent_id] * sin(theta) * time);
    interpolated_points.emplace_back(interpoated_point);
    interpoated_times.emplace_back(from_time + time);
  }
  const double remain_time = fmod(expand_distance, env.velocities[agent_id]);
  if (remain_time > 0.0) {
    Point interpoated_point = make_tuple(get<0>(from_point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                         get<1>(from_point) + env.velocities[agent_id] * sin(theta) * expand_time);
    interpolated_points.emplace_back(interpoated_point);
    interpoated_times.emplace_back(to_time);
  }

  if (interpolated_points.empty()) {
    interpolated_points.emplace_back(from_point);
    interpoated_times.emplace_back(from_time);
  }
  assert(calculateDistance(interpolated_points.back(), from_point) <
         env.max_expand_distances[agent_id] + env.threshold);
  assert(calculateDistance(interpolated_points.front(), to_point) < env.max_expand_distances[agent_id] + env.threshold);
}