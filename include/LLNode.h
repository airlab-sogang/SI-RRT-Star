//
// Created by joonyeol on 23. 12. 7.
//

#ifndef LLNODE_H
#define LLNODE_H

#include "common.h"

class LLNode {
 public:
  Point point;
  shared_ptr<LLNode> parent;
  Interval interval;
  int node_id;

  explicit LLNode(Point point) : point(std::move(point)), interval(0, numeric_limits<double>::infinity()) {}
};

#endif  // LLNODE_H
