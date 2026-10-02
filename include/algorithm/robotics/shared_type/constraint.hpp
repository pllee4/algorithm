/*
 * constraint.hpp
 *
 * Created on: Sep 23, 2026 19:06
 * Description: A CBS constraint: tells one agent where it may NOT be, and the
 * (location, timestep) state of the time-expanded graph it applies to.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <cstddef>
#include <functional>

#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
/** @brief A cell at a timestep: one vertex of the time-expanded graph. */
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

/** @brief A CBS constraint: tells one agent where it may NOT be. */
struct Constraint {
  enum class Kind {
    kVertex,  ///< Occupying `a` at timestep `t`.
    kEdge,    ///< Moving from `a` to `b`, arriving at timestep `t`.
  };
  Kind kind{Kind::kVertex};  ///< What is forbidden.
  Coordinate a{};            ///< The forbidden cell, or where the move starts.
  Coordinate b{};            ///< Where the move ends; only for Kind::kEdge.
  int t{};                   ///< Timestep the constraint applies to.

  /**
   * @brief Makes a vertex constraint.
   * @param a Cell the agent must not occupy.
   * @param t Timestep at which `a` is forbidden.
   * @return The constraint.
   */
  static Constraint Vertex(const Coordinate &a, int t);

  /**
   * @brief Makes an edge constraint, which forbids the swap conflict.
   * @param a Cell the agent moves from.
   * @param b Cell the agent moves to.
   * @param t Timestep at which the agent would arrive at `b`.
   * @return The constraint.
   */
  static Constraint Edge(const Coordinate &a, const Coordinate &b, int t);
};

}  // namespace pllee4::graph
