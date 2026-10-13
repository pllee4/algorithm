/*
 * constraint_table.hpp
 *
 * Created on: Sep 23, 2026 19:06
 * Description: Lookup table for one agent's CBS constraints. Handles vertex,
 * edge and range blocks, plus the dual LEQLENGTH behaviour (length bound for
 * the owner, cell block for everyone else).
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "algorithm/robotics/shared_type/constraint.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
class ConstraintTable {
 public:
  /// Sentinel for unbounded time ranges.
  static constexpr int kMaxTimestep = std::numeric_limits<int>::max() / 2;

  /**
   * @brief Builds the table from a list of constraints.
   *
   * When `agent >= 0`, LEQLENGTH constraints branch: the owner gets a
   * length bound while other agents get a range block on the goal cell.
   * When `agent < 0` (default), every constraint is applied directly.
   *
   * @param constraints All constraints from root to the current CT node.
   * @param goal The agent's goal cell.
   * @param agent Who this table is for (-1 = apply all, no branching).
   */
  ConstraintTable(const std::vector<Constraint> &constraints,
                  const Coordinate &goal, int agent = -1);

  /** @brief True if occupying `cell` at `t` is forbidden. */
  [[nodiscard]] bool IsBlocked(const Coordinate &cell, int t) const;

  /** @brief True if moving `from` -> `to` arriving at `t` is forbidden. */
  [[nodiscard]] bool IsBlocked(const Coordinate &from, const Coordinate &to,
                               int t) const;

  /**
   * @brief Earliest timestep from which the agent may hold its goal.
   *
   * A path is only valid if it arrives at or after this time. Considers
   * vertex constraints and range blocks on the goal, plus length_min.
   */
  [[nodiscard]] int GetHoldingTime() const noexcept;

  /** @brief Latest constrained timestep (-1 if none). */
  [[nodiscard]] int GetLatestConstraint() const noexcept;

  /** @brief Minimum path length from GLENGTH constraints, default 0. */
  [[nodiscard]] int MinLength() const noexcept { return min_length_; }

  /** @brief Maximum path length from LEQLENGTH constraints. */
  [[nodiscard]] int MaxLength() const noexcept { return max_length_; }

 private:
  struct EdgeKey {
    Coordinate from;
    Coordinate to;
    int t;
    bool operator==(const EdgeKey &other) const {
      return (from == other.from && to == other.to && t == other.t);
    }
  };

  struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey &k) const noexcept {
      const auto hash = CoordinateHash{};
      return (hash(k.from) * 31 + hash(k.to)) * 31 + std::hash<int>{}(k.t);
    }
  };

  std::unordered_set<State, StateHash> vertex_;
  std::unordered_set<EdgeKey, EdgeKeyHash> edge_;
  /// Half-open time ranges [t_min, t_max) per cell.
  std::unordered_map<Coordinate, std::vector<std::pair<int, int>>,
                     CoordinateHash>
      ranges_;
  Coordinate goal_;
  int min_length_{0};
  int max_length_{kMaxTimestep};
  int holding_time_{0};
  int latest_constraint_{-1};
};
}  // namespace pllee4::graph
