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

Constraint Constraint::Vertex(int agent, const Coordinate &cell, int t) {
  return {Kind::kVertex, agent, cell, {-1, -1}, t};
}

Constraint Constraint::Vertex(const Coordinate &cell, int t) {
  return Vertex(-1, cell, t);
}

Constraint Constraint::Edge(int agent, const Coordinate &from,
                            const Coordinate &to, int t) {
  return {Kind::kEdge, agent, from, to, t};
}

Constraint Constraint::Edge(const Coordinate &from, const Coordinate &to,
                            int t) {
  return Edge(-1, from, to, t);
}

Constraint Constraint::LeqLength(int agent, const Coordinate &goal, int t) {
  return {Kind::kLeqLength, agent, goal, {-1, -1}, t};
}

Constraint Constraint::GLength(int agent, const Coordinate &goal, int t) {
  return {Kind::kGLength, agent, goal, {-1, -1}, t};
}

}  // namespace pllee4::graph
