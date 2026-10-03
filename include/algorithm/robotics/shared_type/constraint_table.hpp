/*
 * constraint_table.hpp
 *
 * Created on: Sep 23, 2026 19:06
 * Description: Lookup tables for the CBS constraints of one agent, answering
 * the vertex and edge queries the low-level search makes per successor.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <cstddef>
#include <functional>
#include <unordered_set>
#include <vector>

#include "algorithm/robotics/shared_type/constraint.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
class ConstraintTable {
 public:
  ConstraintTable(const std::vector<Constraint> &constraints,
                  const Coordinate &goal);

  /**
   * @brief Checks for a vertex constraint.
   * @param coordinate Cell the agent would occupy.
   * @param t Timestep at which it would occupy it.
   * @return true if occupying `coordinate` at `t` is forbidden.
   */
  [[nodiscard]] bool IsBlocked(const Coordinate &coordinate, int t) const;

  /**
   * @brief Checks for an edge constraint.
   * @param from Cell the agent moves from.
   * @param to Cell the agent moves to.
   * @param t Timestep at which the agent would arrive at `to`.
   * @return true if the move is forbidden.
   */
  [[nodiscard]] bool IsBlocked(const Coordinate &from, const Coordinate &to,
                               int t) const;

  /**
   * @brief Gets the holding time.
   *
   * The holding time is the earliest timestep from which the agent is allowed
   * to sit on its goal forever. Reaching the goal before this is not a
   * solution: the agent would have to leave and come back.
   *
   * @return One more than the latest vertex constraint on the goal, or 0 if
   * there is none.
   */
  [[nodiscard]] int GetHoldingTime() const noexcept;

  /**
   * @brief Gets the latest constrained timestep.
   * @return The largest `t` of any constraint, or -1 if there are none.
   */
  [[nodiscard]] int GetLatestConstraint() const noexcept;

 private:
  /** @brief Set key for an edge constraint. */
  struct EdgeKey {
    Coordinate from;  ///< Where the move starts.
    Coordinate to;    ///< Where the move ends.
    int t;            ///< Arrival timestep.
    /** @brief Equal when all fields match. */
    bool operator==(const EdgeKey &other) const {
      return (from == other.from && to == other.to && t == other.t);
    }
  };

  /** @brief Hashes an EdgeKey. */
  struct EdgeKeyHash {
    /** @brief Hashes `k`. @param k The edge key. @return Its hash. */
    std::size_t operator()(const EdgeKey &k) const noexcept {
      const auto hash = CoordinateHash{};
      return (hash(k.from) * 31 + hash(k.to)) * 31 + std::hash<int>{}(k.t);
    }
  };

  std::unordered_set<State, StateHash> vertex_;    ///< Vertex constraints.
  std::unordered_set<EdgeKey, EdgeKeyHash> edge_;  ///< Edge constraints.
  int holding_time_{0};                            ///< See GetHoldingTime().
  int latest_constraint_{-1};  ///< See GetLatestConstraint().
};
}  // namespace pllee4::graph
