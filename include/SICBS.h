//
// Created by joonyeol on 23. 12. 8.
//

#ifndef SICBS_H
#define SICBS_H

#include "ConstraintTable.h"
#include "HLNode.h"
#include "SharedEnv.h"
#include "common.h"

struct compare_function {
  bool operator()(const int& n1, const int& n2) const {
    return n1 > n2;  // This will create a min-heap
  }
};

class SICBS {
 public:
  boost::heap::fibonacci_heap<int, boost::heap::compare<compare_function>> heap;
};

#endif  // SICBS_H
