/*
 * constraint_table.cpp
 *
 * Created on: Sep 23, 2026 19:06
 * Description: Implementation of the CBS constraint table, see
 * constraint_table.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/shared_type/constraint_table.hpp"

#include <algorithm>

namespace pllee4::graph {

ConstraintTable::ConstraintTable(const std::vector<Constraint> &constraints,
                                 const Coordinate &goal) {
  for (const auto &constraint : constraints) {
    if (constraint.kind == Constraint::Kind::kVertex) {
      vertex_.insert(State{constraint.a, constraint.t});
      if (constraint.a == goal) {
        holding_time_ = std::max(holding_time_, constraint.t + 1);
      }
    } else {
      edge_.insert(EdgeKey{constraint.a, constraint.b, constraint.t});
    }
    latest_constraint_ = std::max(latest_constraint_, constraint.t);
  }
}

bool ConstraintTable::IsBlocked(const Coordinate &coordinate, int t) const {
  return (vertex_.count(State{coordinate, t}) != 0);
}

bool ConstraintTable::IsBlocked(const Coordinate &from, const Coordinate &to,
                                int t) const {
  return (edge_.count(EdgeKey{from, to, t}) != 0);
}

int ConstraintTable::GetHoldingTime() const noexcept { return holding_time_; }

int ConstraintTable::GetLatestConstraint() const noexcept {
  return latest_constraint_;
}

}  // namespace pllee4::graph
