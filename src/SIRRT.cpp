#include "SIRRT.h"

Path SIRRT::run() {
  release();
  const auto start_node = make_shared<LLNode>(start_point);
  start_node->earliest_arrival_times = {0};
  start_node->intervals = {{0, numeric_limits<double>::max()}};
  nodes.push_back(start_node);

  while (env.iterations[agent_id]--) {
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    auto new_node = steer(nearest_node, random_point);
    if (new_node == nullptr) {
      continue;
    }
    nodes.push_back(new_node);

    // check goal
    if (calculateDistance(new_node->point, goal_point) < env.threshold) {
      for (int i = 0; i < new_node->intervals.size(); ++i) {
        if (constraint_table.targetConstrained(new_node->point, get<0>(new_node->intervals[i]), env.radii[agent_id]))
          continue;
        path = updatePath(new_node, i);
        assert(calculateDistance(get<0>(path.front()), start_point) < env.threshold);
        assert(calculateDistance(get<0>(path.back()), goal_point) < env.threshold);
        // print path
        cout << "path" << agent_id << ": ";
        for (const auto& state : path) {
          cout << "(" << get<0>(get<0>(state)) << ", " << get<1>(get<0>(state)) << ", " << get<1>(state) << ")->";
        }
        cout << endl;
        // assert velocity always be 1.0m/s
        for (int i = 0; i < path.size() - 1; ++i) {
          const double distance = calculateDistance(get<0>(path[i]), get<0>(path[i + 1]));
          const double time_diff = get<1>(path[i + 1]) - get<1>(path[i]);
          assert(distance / time_diff < env.velocities[agent_id] + env.threshold);
        }
        return path;
      }
    }
  }

  cout << "No solution found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  // Fixed seed for consistent results across runs
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return make_tuple(get<0>(goal_point), get<1>(goal_point));
  } else {
    return make_tuple(dis_10(env.gen), dis_10(env.gen));
  }
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point) const {
  double min_distance = numeric_limits<double>::max();
  shared_ptr<LLNode> nearest_node;
  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }
  return nearest_node;
}

shared_ptr<LLNode> SIRRT::steer(const shared_ptr<LLNode>& from_node, const Point& random_point) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
  const double theta =
      atan2(get<1>(random_point) - get<1>(from_node->point), get<0>(random_point) - get<0>(from_node->point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const Point to_point = make_tuple(get<0>(from_node->point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                    get<1>(from_node->point) + env.velocities[agent_id] * sin(theta) * expand_time);

  auto new_node = make_shared<LLNode>(to_point);
  if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) return nullptr;

  vector<Interval> safe_intervals;
  constraint_table.getSafeIntervalTable(agent_id, to_point, env.radii[agent_id], safe_intervals);
  for (int i = 0; i < from_node->intervals.size(); ++i) {
    const double lower_bound = from_node->earliest_arrival_times[i] + expand_time;
    const double upper_bound = get<1>(from_node->intervals[i]) + expand_time;
    for (auto& safe_interval : safe_intervals) {
      if (lower_bound > get<1>(safe_interval) || upper_bound < get<0>(safe_interval)) continue;
      if (constraint_table.pathConstrained(agent_id, from_node->point, to_point, from_node->earliest_arrival_times[i],
                                           from_node->earliest_arrival_times[i] + expand_time, env.radii[agent_id]))
        continue;
      new_node->earliest_arrival_times.emplace_back(lower_bound);
      new_node->intervals.emplace_back(safe_interval);
      new_node->parent_interval_indicies.emplace_back(i);
    }
  }
  if (new_node->earliest_arrival_times.empty()) {
    return nullptr;
  }
  new_node->parent = from_node;
  cout << "New safe interval size: " << new_node->intervals.size() << endl;
  return new_node;
}

double get_earliest_arrival_time(const shared_ptr<LLNode>& node, const int interval_index) {
  return node->earliest_arrival_times[interval_index];
}

Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node, const int interval_index) {
  Path path;
  shared_ptr<LLNode> current_node = goal_node;
  int current_interval_index = interval_index;
  while (current_node) {
    path.emplace_back(current_node->point, current_node->earliest_arrival_times[current_interval_index]);
    current_node = current_node->parent;
    if (current_node && !current_node->parent_interval_indicies.empty()) {
      current_interval_index = current_node->parent_interval_indicies[current_interval_index];
    }
  }
  reverse(path.begin(), path.end());

  return path;
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
}
