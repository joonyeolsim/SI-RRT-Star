#include "SIRRT.h"

Path SIRRT::run() {
  release();
  SafeIntervalTable safe_interval_table(env);

  // initialize start and goal safe intervals
  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, start_point, env.radii[agent_id], safe_interval_table.table[start_point]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, start_point, env.radii[agent_id], safe_interval_table.table[start_point]);
  assert(!safe_interval_table.table[start_point].empty());

  if (env.algorithm == "pp")
    constraint_table.getSafeIntervalTablePath(agent_id, goal_point, env.radii[agent_id], safe_interval_table.table[goal_point]);
  else if (env.algorithm == "cbs")
    constraint_table.getSafeIntervalTable(agent_id, goal_point, env.radii[agent_id], safe_interval_table.table[goal_point]);
  assert(!safe_interval_table.table[goal_point].empty());

  // initialize start node
  auto start_node = make_shared<LLNode>(start_point, 0.0, safe_interval_table.table[start_point].front().second, 0.0);
  start_node->earliest_arrival_time = 0.0;
  nodes.push_back(start_node);

  // initialize goal node
  auto goal_node = make_shared<LLNode>(goal_point, safe_interval_table.table[goal_point].back().first, numeric_limits<double>::infinity(), 0.0);
  goal_node->earliest_arrival_time = numeric_limits<double>::infinity();

  int iteration = env.iterations[agent_id];
  while (iteration--) {
    Point sample_point;
    double sample_velocity;
    if (dis_100(env.gen) >= env.goal_sample_rates[agent_id]) {
      sample_point = generateRandomPoint();
      sample_velocity = generateRandomVelocity();
    } else {
      sample_point = goal_point;
      sample_velocity = 0.0;
    }

    const shared_ptr<LLNode> nearest_node = getNearestNode(sample_point, sample_velocity);
    auto [new_point, new_velocity] = steer(nearest_node, sample_point, sample_velocity, safe_interval_table);
    if (new_point == make_tuple(-1.0, -1.0) && new_velocity == 0.0) {
      continue;
    }

    // SIRRT*
    vector<shared_ptr<LLNode>> neighbors;
    getNeighbors(new_point, new_velocity, neighbors);
    assert(!neighbors.empty());
    vector<shared_ptr<LLNode>> new_nodes = chooseParent(new_point, new_velocity, neighbors, safe_interval_table);
    if (new_nodes.empty()) {
      continue;
    }
    for (auto& new_node : new_nodes) {
      rewire(new_node, neighbors);
    }

    // check goal
    for (auto& new_node : new_nodes) {
      if (calculateDistance(new_node->point, goal_point) < env.epsilon && new_node->velocity < env.epsilon) {
        if (goal_node->interval.first <= new_node->earliest_arrival_time && new_node->earliest_arrival_time < goal_node->earliest_arrival_time) {
          assert(goal_node->interval.first == new_node->interval.first);
          assert(goal_node->interval.second == new_node->interval.second);
          goal_node = new_node;
        }
      } else {
        // print point of the new node and its parent
        cout << "(" << get<0>(new_node->parent->point) << " " << get<1>(new_node->parent->point) << ")" << "->";
        cout << "(" << get<0>(new_node->point) << " " << get<1>(new_node->point) << ")" << endl;
        cout << new_node->parent->velocity << "->" << new_node->velocity << endl;
        nodes.push_back(new_node);
      }
    }
  }

  if (goal_node->earliest_arrival_time < numeric_limits<double>::infinity()) {
    nodes.push_back(goal_node);
    path = updatePath(goal_node);
    return path;
  }

  // cout << "No path found!" << endl;
  return path;
}

Point SIRRT::generateRandomPoint() {
  return make_tuple(dis_width(env.gen), dis_height(env.gen));
}

double SIRRT::generateRandomVelocity() {
  return dis_velocity(env.gen);
}

shared_ptr<LLNode> SIRRT::getNearestNode(const Point& point, double velocity) const {
  if (nodes.empty()) {
    return nullptr;
  }

  double min_distance = numeric_limits<double>::infinity();
  shared_ptr<LLNode> nearest_node = nullptr;

  for (const auto& node : nodes) {
    double position_distance = calculateDistance(node->point, point);
    double velocity_distance = std::abs(node->velocity - velocity);
    double combined_distance = position_weight * position_distance + velocity_weight * velocity_distance;

    if (combined_distance < min_distance) {
      min_distance = combined_distance;
      nearest_node = node;
    }
  }

  return nearest_node;
}


pair<Point, double> SIRRT::steer(const shared_ptr<LLNode>& from_node, const Point& sample_point, double sample_velocity,
                                 SafeIntervalTable& safe_interval_table) const {
  // double theta = atan2(get<1>(sample_point) - get<1>(from_node->point), get<0>(sample_point) - get<0>(from_node->point));
  double from_velocity = from_node->velocity;
  double to_velocity = sample_velocity;
  double acceleration = (to_velocity - from_velocity) / env.expand_time;

  Point current_point = from_node->point;
  double current_velocity = from_velocity;
  double time_step = env.time_resolution;
  double elapsed_time = 0.0;

  while (env.expand_time - elapsed_time > env.epsilon) {
    double theta = atan2(get<1>(goal_point) - get<1>(current_point), get<0>(goal_point) - get<0>(current_point));
    current_velocity += acceleration * time_step;
    double distance_step = current_velocity * time_step;

    Point next_point = make_tuple(
      get<0>(current_point) + distance_step * cos(theta),
      get<1>(current_point) + distance_step * sin(theta)
    );

    if (constraint_table.obstacleConstrained(agent_id, current_point, next_point, env.radii[agent_id])) {
      return {make_tuple(-1.0, -1.0), 0.0};
    }

    current_point = next_point;
    elapsed_time += time_step;
  }

  if (calculateDistance(current_point, goal_point) < env.epsilon && abs(current_velocity) < env.epsilon) {
    return {goal_point, 0.0};
  }

  if (safe_interval_table.table[current_point].empty()) {
    if (env.algorithm == "pp") {
      constraint_table.getSafeIntervalTablePath(agent_id, current_point, env.radii[agent_id], safe_interval_table.table[current_point]);
    } else if (env.algorithm == "cbs") {
      constraint_table.getSafeIntervalTable(agent_id, current_point, env.radii[agent_id], safe_interval_table.table[current_point]);
    }
    if (safe_interval_table.table[current_point].empty()) {
      return {make_tuple(-1.0, -1.0), 0.0};
    }
  }

  return {current_point, current_velocity};
}


Path SIRRT::updatePath(const shared_ptr<LLNode>& goal_node) const {
  Path path;
  shared_ptr<LLNode> curr_node = goal_node;
  while (curr_node->parent != nullptr) {
    const auto prev_node = curr_node->parent;
    const auto prev_time = prev_node->earliest_arrival_time;
    const auto curr_time = curr_node->earliest_arrival_time;
    assert(prev_time < curr_time);

    // const auto expand_time = calculateDistance(prev_node->point, curr_node->point) / env.velocities[agent_id];
    const auto expand_time = 1.0;
    path.emplace_back(curr_node->point, curr_time);
    if (prev_time + expand_time + env.epsilon < curr_time) {
      path.emplace_back(prev_node->point, curr_time - expand_time);
    }
    curr_node = curr_node->parent;
  }
  path.emplace_back(curr_node->point, 0);
  reverse(path.begin(), path.end());

  if(calculateDistance(get<0>(path.front()), start_point) >= env.epsilon) {
    throw runtime_error("Start point is not correct!");
  }
  if(calculateDistance(get<0>(path.back()), goal_point) >= env.epsilon) {
    throw runtime_error("Goal point is not correct!");
  }

  return path;
}

void SIRRT::getNeighbors(Point point, double velocity, vector<shared_ptr<LLNode>>& neighbors) const {
  int k = 10;
  assert(!nodes.empty());
  assert(neighbors.empty());

  // Vector to store nodes and their combined distances
  vector<pair<shared_ptr<LLNode>, double>> distance_node_pairs;

  for (const auto& node : nodes) {
    double position_distance = calculateDistance(node->point, point);
    double velocity_distance = std::abs(node->velocity - velocity);
    double combined_distance = position_weight * position_distance + velocity_weight * velocity_distance;

    // Add the node and its combined distance to the vector
    distance_node_pairs.emplace_back(node, combined_distance);
  }

  // Sort the vector based on combined distances
  sort(distance_node_pairs.begin(), distance_node_pairs.end(), [](const pair<shared_ptr<LLNode>, double>& a, const pair<shared_ptr<LLNode>, double>& b) {
    return a.second < b.second;
  });

  // Select the top k nodes
  for (int i = 0; i < k && i < distance_node_pairs.size(); ++i) {
    const auto& node = distance_node_pairs[i].first;
    neighbors.emplace_back(node);
    // if (!constraint_table.obstacleConstrained(agent_id, node->point, point, env.radii[agent_id])) {
    //   neighbors.emplace_back(node);
    // }
  }
}


vector<shared_ptr<LLNode>> SIRRT::chooseParent(const Point& new_point, double new_velocity, const vector<shared_ptr<LLNode>>& neighbors,
                                       SafeIntervalTable& safe_interval_table) const {
  assert(!neighbors.empty());

  auto new_nodes = vector<shared_ptr<LLNode>>();

  for (auto& safe_interval : safe_interval_table.table[new_point]) {
    auto new_node = make_shared<LLNode>(new_point, safe_interval.first, safe_interval.second, new_velocity);

    for (const auto& neighbor : neighbors) {
      const double lower_bound = neighbor->earliest_arrival_time + env.expand_time;
      const double upper_bound = neighbor->interval.second + env.expand_time;

      if (lower_bound >= new_node->interval.second) continue;
      if (upper_bound <= new_node->interval.first) continue;

      const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
          agent_id, neighbor->point, new_node->point, env.expand_time, max(new_node->interval.first, lower_bound),
          min(new_node->interval.second, upper_bound), env.radii[agent_id]);
      if (earliest_arrival_time < 0.0) continue;

      if (earliest_arrival_time < new_node->earliest_arrival_time) {
        new_node->earliest_arrival_time = earliest_arrival_time;
        new_node->parent = neighbor;
      }
    }
    if (new_node->parent) {
      new_nodes.push_back(new_node);
    }
  }

  return new_nodes;
}

void SIRRT::rewire(const shared_ptr<LLNode>& new_node, const vector<shared_ptr<LLNode>>& neighbors) {
  assert(!neighbors.empty());
  for (auto& neighbor : neighbors) {
    if (neighbor == new_node->parent) continue;
    const double lower_bound = new_node->earliest_arrival_time + env.expand_time;
    const double upper_bound = new_node->interval.second + env.expand_time;

    if (lower_bound >= neighbor->interval.second) continue;
    if (upper_bound <= neighbor->interval.first) continue;

    const double earliest_arrival_time = constraint_table.getEarliestArrivalTime(
        agent_id, new_node->point, neighbor->point, env.expand_time, max(neighbor->interval.first, lower_bound),
        min(neighbor->interval.second, upper_bound), env.radii[agent_id]);
    if (earliest_arrival_time < 0.0) continue;

    if (earliest_arrival_time < neighbor->earliest_arrival_time) {
      neighbor->earliest_arrival_time = earliest_arrival_time;
      neighbor->parent = new_node;
    }
  }
}

void SIRRT::release() {
  nodes.clear();
  path.clear();
  goal_node = nullptr;
}
