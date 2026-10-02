/*
 * constraint.cpp
 *
 * Created on: Sep 23, 2026 19:06
 * Description: Implementation of the CBS constraint, see constraint.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/shared_type/constraint.hpp"

namespace pllee4::graph {

Constraint Constraint::Vertex(const Coordinate &a, int t) {
  return {Kind::kVertex, a, {-1, -1}, t};
}

Constraint Constraint::Edge(const Coordinate &a, const Coordinate &b, int t) {
  return {Kind::kEdge, a, b, t};
}

}  // namespace pllee4::graph
