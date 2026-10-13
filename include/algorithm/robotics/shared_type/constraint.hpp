/*
 * constraint.hpp
 *
 * Created on: Sep 23, 2026 19:06
 * Description: CBS constraint types. Each constraint kind is a distinct
 * struct with only the fields it needs, composed into a std::variant.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <cstddef>
#include <functional>
#include <variant>

#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
struct State {
  Coordinate at;
  int t;
  bool operator==(const State &other) const {
    return (at == other.at && t == other.t);
  }
};

struct CoordinateHash {
  std::size_t operator()(const Coordinate &c) const noexcept {
    return std::hash<int>{}(c.x) * 31 + std::hash<int>{}(c.y);
  }
};

struct StateHash {
  std::size_t operator()(const State &s) const noexcept {
    return CoordinateHash{}(s.at) * 31 + std::hash<int>{}(s.t);
  }
};

/// Cannot occupy `cell` at time `t`.
struct VertexConstraint {
  int agent{-1};
  Coordinate cell;
  int t;
};

/// Cannot move `from` -> `to` arriving at time `t`.
struct EdgeConstraint {
  int agent{-1};
  Coordinate from;
  Coordinate to;
  int t;
};

/// Owner's path length <= `t`; other agents blocked at `goal` from `t`.
struct LeqLengthConstraint {
  int agent;
  Coordinate goal;
  int t;
};

/// Owner's path length >= `t + 1`.
struct GLengthConstraint {
  int agent;
  Coordinate goal;
  int t;
};

using Constraint = std::variant<VertexConstraint, EdgeConstraint,
                                LeqLengthConstraint, GLengthConstraint>;

inline int GetTimestep(const Constraint &constraint) {
  return std::visit([](const auto &v) { return v.t; }, constraint);
}

inline int GetAgent(const Constraint &constraint) {
  return std::visit([](const auto &v) { return v.agent; }, constraint);
}

}  // namespace pllee4::graph
