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

namespace {
template <class... Ts>
struct Overloaded : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

bool AgentApplies(int constraint_agent, int table_agent) {
  return (constraint_agent < 0 || table_agent < 0 ||
          constraint_agent == table_agent);
}
}  // namespace

ConstraintTable::ConstraintTable(const std::vector<Constraint> &constraints,
                                 const Coordinate &goal, int agent)
    : goal_(goal) {
  for (const auto &constraint : constraints) {
    std::visit(Overloaded{
                   [this, agent](const VertexConstraint &c) {
                     if (AgentApplies(c.agent, agent)) {
                       vertex_.insert(State{c.cell, c.t});
                       if (c.cell == goal_)
                         holding_time_ = std::max(holding_time_, c.t + 1);
                     }
                   },
                   [this, agent](const EdgeConstraint &c) {
                     if (AgentApplies(c.agent, agent))
                       edge_.insert(EdgeKey{c.from, c.to, c.t});
                   },
                   [this, agent](const LeqLengthConstraint &c) {
                     if (c.agent >= 0 && agent >= 0) {
                       if (c.agent == agent)
                         max_length_ = std::min(max_length_, c.t);
                       else
                         ranges_[c.goal].emplace_back(c.t, kMaxTimestep);
                     } else {
                       max_length_ = std::min(max_length_, c.t);
                     }
                   },
                   [this, agent](const GLengthConstraint &c) {
                     if (AgentApplies(c.agent, agent))
                       min_length_ = std::max(min_length_, c.t + 1);
                   },
               },
               constraint);
    latest_constraint_ = std::max(latest_constraint_, GetTimestep(constraint));
  }
  if (latest_constraint_ < min_length_) latest_constraint_ = min_length_;
  if (max_length_ < kMaxTimestep && latest_constraint_ < max_length_)
    latest_constraint_ = max_length_;
}

bool ConstraintTable::IsBlocked(const Coordinate &cell, int t) const {
  if (vertex_.count(State{cell, t}) != 0) return true;
  const auto it = ranges_.find(cell);
  if (it != ranges_.end()) {
    for (const auto &[t_min, t_max] : it->second) {
      if (t >= t_min && t < t_max) return true;
    }
  }
  return false;
}

bool ConstraintTable::IsBlocked(const Coordinate &from, const Coordinate &to,
                                int t) const {
  return (edge_.count(EdgeKey{from, to, t}) != 0);
}

int ConstraintTable::GetHoldingTime() const noexcept {
  int result = std::max(holding_time_, min_length_);
  const auto it = ranges_.find(goal_);
  if (it != ranges_.end()) {
    for (const auto &[t_min, t_max] : it->second)
      result = std::max(result, t_max);
  }
  return result;
}

int ConstraintTable::GetLatestConstraint() const noexcept {
  return latest_constraint_;
}

}  // namespace pllee4::graph
