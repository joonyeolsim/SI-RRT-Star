//
// Created by joonyeol on 23. 12. 7.
//

#ifndef SIRRT_H
#define SIRRT_H

#include <ReservationTable.h>

#include "ConstraintTable.h"
#include "LLNode.h"
#include "common.h"

class SIRRT {
 public:
  std::uniform_real_distribution<> dis_10;
  std::uniform_real_distribution<> dis_100;
  vector<shared_ptr<LLNode>> nodes;
  Point start_point;
  Point goal_point;
  Path path;
  int agent_id;
  SharedEnv& env;
  ConstraintTable& constraint_table;
  ReservationTable& reservation_table;

  SIRRT(int agent_id, SharedEnv& env, ConstraintTable& constraint_table, ReservationTable& reservation_table)
      : dis_10(0.0, 10.0),
        dis_100(0.0, 100.0),
        env(env),
        constraint_table(constraint_table),
        agent_id(agent_id),
        start_point(env.start_points[agent_id]),
        goal_point(env.goal_points[agent_id]),
        reservation_table(reservation_table) {}
  ~SIRRT() = default;
  Path run();
  Point generateRandomPoint();
  shared_ptr<LLNode> getNearestNode(const Point& point) const;
  shared_ptr<LLNode> steer(const shared_ptr<LLNode>& from_node, const Point& random_point) const;
  Path updatePath(const shared_ptr<LLNode>& goal_node, const int interval_index);
  void release();
};

#endif  // SIRRT_H
