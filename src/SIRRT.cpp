#include "SIRRT.h"

Path SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  vector<Interval> start_safe_intervals;
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, start_point, env.radii[agent_id], start_safe_intervals);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, start_point, env.radii[agent_id], start_safe_intervals);
  assert(!start_safe_intervals.empty());
  safe_interval_table.table[start_point] = {make_tuple(0.0, get<1>(start_safe_intervals.front()))};

  vector<Interval> goal_safe_intervals;
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, goal_point, env.radii[agent_id], goal_safe_intervals);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, goal_point, env.radii[agent_id], goal_safe_intervals);
  assert(!goal_safe_intervals.empty());
  safe_interval_table.table[goal_point] = {
      make_tuple(get<0>(goal_safe_intervals.back()), numeric_limits<double>::infinity())};

  // initialize start node
  const auto start_node = make_shared<LLNode>(start_point);
  start_node->interval = make_tuple(0.0, get<1>(start_safe_intervals.front()));
  nodes.push_back(start_node);

  double best_earliest_arrival_time = numeric_limits<double>::infinity();
  int iteration = env.iterations[agent_id];
  while (iteration--) {
    Point random_point = generateRandomPoint();
    const shared_ptr<LLNode> nearest_node = getNearestNode(random_point);
    Point new_point = steer(nearest_node, random_point, safe_interval_table);
    if (new_point == make_tuple(-1.0, -1.0)) {
      continue;
    }

    // SIRRT*
    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(new_point, neighbors);
    if (neighbors.empty()) {
      continue;
    }
    shared_ptr<LLNode> new_node = chooseParent(new_point, neighbors, safe_interval_table);
    if (new_node == nullptr) {
      continue;
    }
    rewire(new_node, neighbors, safe_interval_table);

    // check goal
    if (calculateDistance(new_node->point, goal_point) < env.epsilon) {
      if (goal_node == nullptr || get<0>(new_node->interval) < best_earliest_arrival_time) {
        goal_node = new_node;
        best_earliest_arrival_time = get<0>(new_node->interval);
      }
    } else {
      nodes.push_back(new_node);
    }
  }

  if (goal_node != nullptr) {
    nodes.push_back(goal_node);
    path = updatePath(goal_node);
    return path;
  }

  // cout << "No path found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  if (dis_100(env.gen) < env.goal_sample_rates[agent_id]) {
    return goal_point;
  }
  return make_tuple(dis_width(env.gen), dis_height(env.gen));
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::max();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < min_distance) {
      min_distance = distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}

Point SIRRT::steer(const shared_ptr<LLNode>& from_node, const Point& random_point,
                   SafeIntervalTable& safe_interval_table) const {
  const double expand_distance =
      min(env.max_expand_distances[agent_id], calculateDistance(from_node->point, random_point));
  const double theta =
      atan2(get<1>(random_point) - get<1>(from_node->point), get<0>(random_point) - get<0>(from_node->point));
  const double expand_time = expand_distance / env.velocities[agent_id];
  const Point to_point = make_tuple(get<0>(from_node->point) + env.velocities[agent_id] * cos(theta) * expand_time,
                                    get<1>(from_node->point) + env.velocities[agent_id] * sin(theta) * expand_time);

  if (constraint_table.obstacleConstrained(agent_id, from_node->point, to_point, env.radii[agent_id])) {
    return make_tuple(-1, -1);
  }

  if (calculateDistance(to_point, goal_point) < env.epsilon) {
    return goal_point;
  }

  vector<Interval> safe_intervals;
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, to_point, env.radii[agent_id], safe_intervals);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, to_point, env.radii[agent_id], safe_intervals);
  if (safe_intervals.empty()) {
    return make_tuple(-1.0, -1.0);
  }
  safe_interval_table.table[to_point] = safe_intervals;

  return to_point;
}

Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node) const {
  Path path;
  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    const auto prev_node = curr_node->parent;
    const auto prev_time = get<0>(prev_node->interval);
    const auto curr_time = get<0>(curr_node->interval);
    assert(prev_time < curr_time);

    const auto expand_time = calculateDistance(prev_node->point, curr_node->point) / env.velocities[agent_id];
    path.emplace_back(curr_node->point, curr_time);
    if (prev_time + expand_time + env.epsilon < curr_time) {
      path.emplace_back(prev_node->point, curr_time - expand_time);
    }
    curr_node = curr_node->parent;
  }
  path.emplace_back(curr_node->point, 0);
  reverse(path.begin(), path.end());

  assert(calculateDistance(get<0>(path.front()), start_point) < env.epsilon);
  assert(calculateDistance(get<0>(path.back()), goal_point) < env.epsilon);

  return path;
}

void SIRRT::getNeighbors(Point point, vector<shared_ptr<LLNode>>& neighbors) const {
  assert(!nodes.empty());
  assert(neighbors.empty());

  const double connection_radius = env.max_expand_distances[agent_id] + env.epsilon;
  for (const auto& node : nodes) {
    const double distance = calculateDistance(node->point, point);
    if (distance < connection_radius) {
      if (constraint_table.obstacleConstrained(agent_id, node->point, point, env.radii[agent_id])) continue;
      neighbors.emplace_back(node);
    }
  }
}

shared_ptr<LLNode> SIRRT::chooseParent(const Point& new_point, const vector<shared_ptr<LLNode>>& neighbors,
                                       SafeIntervalTable& safe_interval_table) const {
  assert(!neighbors.empty());

  shared_ptr<LLNode> new_node = make_shared<LLNode>(new_point);

  for (const auto& neighbor : neighbors) {
    const double expand_time = calculateDistance(neighbor->point, new_node->point) / env.velocities[agent_id];

    const double lower_bound = get<0>(neighbor->interval) + expand_time;
    const double upper_bound = get<1>(neighbor->interval) + expand_time;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    auto safe_intervals = safe_interval_table.table[new_node->point];
    for (auto& safe_interval : safe_intervals) {
      if (lower_bound + env.epsilon >= get<1>(safe_interval)) continue;
      if (upper_bound - env.epsilon <= get<0>(safe_interval)) break;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, neighbor->point, new_node->point, expand_time, max(get<0>(safe_interval), lower_bound),
          min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0) continue;

      if (new_node->parent == nullptr || earliest_arrival_time < get<0>(new_node->interval)) {
        new_node->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
        new_node->parent = neighbor;
      }
    }
  }

  if (new_node->parent == nullptr) {
    return nullptr;
  }
  return new_node;
}

void SIRRT::rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors,
                   SafeIntervalTable& safe_interval_table) {
  assert(!neighbors.empty());
  for (auto& neighbor : neighbors) {
    if (neighbor == new_node->parent) continue;
    const double expand_time = calculateDistance(new_node->point, neighbor->point) / env.velocities[agent_id];

    const double lower_bound = get<0>(new_node->interval) + expand_time;
    const double upper_bound = get<1>(new_node->interval) + expand_time;
    assert(lower_bound > 0);
    assert(lower_bound < upper_bound);

    auto safe_intervals = safe_interval_table.table[neighbor->point];
    for (auto& safe_interval : safe_intervals) {
      if (lower_bound + env.epsilon >= get<1>(safe_interval)) continue;
      if (upper_bound - env.epsilon <= get<0>(safe_interval)) break;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, new_node->point, neighbor->point, expand_time, max(get<0>(safe_interval), lower_bound),
          min(get<1>(safe_interval), upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0) continue;

      if (earliest_arrival_time < get<0>(neighbor->interval)) {
        if (get<1>(safe_interval) <= get<0>(neighbor->interval)) {
          shared_ptr<LLNode> new_neighbor_node = make_shared<LLNode>(neighbor->point);
          new_neighbor_node->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
          new_neighbor_node->parent = new_node;
          nodes.push_back(new_neighbor_node);
        } else {
          neighbor->interval = make_tuple(earliest_arrival_time, get<1>(safe_interval));
          neighbor->parent = new_node;
        }
        break;
      }
    }
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
}
