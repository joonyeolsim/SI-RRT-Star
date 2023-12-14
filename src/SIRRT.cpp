#include "SIRRT.h"

Path SIRRT::run() {
  release();
  const auto start_node = make_shared<LLNode>(start_point, 0.0);
  start_node->intervals = {{start_node->time, numeric_limits<double>::max()}};
  nodes.push_back(start_node);

  while (env.iterations[agent_id]--) {
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    auto new_node = steer(nearest_node, random_point);
    if (new_node == nullptr) {
      continue;
    }
    new_node->parent = nearest_node;
    // TODO : update intervals
    new_node->intervals = {{new_node->time, numeric_limits<double>::max()}};
    nodes.push_back(new_node);

    // check goal
    if (calculateDistance(new_node->point, goal_point) < env.threshold &&
        !constraint_table.targetConstrained(new_node->point, new_node->time, env.radii[agent_id])) {
      path = updatePath(new_node);
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

  auto new_node = make_shared<LLNode>(to_point, from_node->time + expand_time);

  for (auto& interval : from_node->intervals) {
    if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) return nullptr;
    if (constraint_table.pathConstrained(agent_id, from_node->point, to_point, from_node->time,
                                         from_node->time + expand_time, env.radii[agent_id]))
      return nullptr;
    new_node->intervals.emplace_back(get<0>(interval) + expand_time, get<1>(interval) + expand_time);
  }

  return new_node;
}

Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node) {
  Path path;
  shared_ptr<LLNode> node = goal_node;
  while (node != nullptr) {
    path.emplace_back(node->point, node->time);
    node = node->parent;
  }
  reverse(path.begin(), path.end());
  return path;
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
}
