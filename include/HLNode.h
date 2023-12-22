//
// Created by joonyeol on 23. 12. 8.
//

#ifndef HLNODE_H
#define HLNODE_H

#include "common.h"

class HLNode {
public:
  Solution solution;
  vector<Conflict> conflicts;
  vector<Constraint> constraints;
  double cost;
};

#endif  // HLNODE_H
