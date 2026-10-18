/*
 * mdd.hpp
 *
 * Created on: Oct 15, 2026 21:00
 * Description: Multi-Valued Decision Diagram (MDD). A layered DAG of all
 * optimal-cost paths for one agent under a set of constraints. Level t
 * contains every cell the agent could occupy at timestep t in some
 * optimal path. Width 1 at a level means the agent is forced through
 * that cell (singleton), which is the basis for cardinality classification
 * in CBS: both singletons → cardinal, one → semi, neither → non.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <vector>

#include "algorithm/robotics/shared_type/constraint.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {

class MDD {
 public:
  /**
   * @brief Builds the MDD for one agent.
   *
   * Forward pass: starting from `start` at t=0, generates all reachable
   * cells at each timestep where t + h(cell) <= optimal_cost, pruning by
   * constraints. Only the goal survives at the last level.
   *
   * Backward pass: removes cells that have no valid transition to a
   * surviving cell at the next level.
   *
   * @param width Grid columns.
   * @param height Grid rows.
   * @param obstacles Blocked cells.
   * @param start Agent's start cell.
   * @param goal Agent's goal cell.
   * @param constraints All constraints from root to this CT node.
   * @param agent Agent index (for ConstraintTable dual behaviour).
   * @param optimal_cost The agent's current path cost (path.size() - 1).
   */
  MDD(int width, int height, const std::vector<Coordinate> &obstacles,
      const Coordinate &start, const Coordinate &goal,
      const std::vector<Constraint> &constraints, int agent,
      int optimal_cost);

  /** @brief Number of cells at this timestep in the MDD. */
  [[nodiscard]] int GetWidth(int timestep) const;

  /** @brief True if there is exactly one cell at this timestep. */
  [[nodiscard]] bool IsSingleton(int timestep) const;

  /** @brief Number of levels (optimal_cost + 1). */
  [[nodiscard]] int GetNumLevels() const;

  /** @brief Cells at the given timestep. */
  [[nodiscard]] const std::vector<Coordinate> &GetLevel(int timestep) const;

 private:
  std::vector<std::vector<Coordinate>> levels_;
};

}  // namespace pllee4::graph
