/*
 * space_time_astar.hpp
 *
 * Created on: Sep 13, 2026 15:05
 * Description: Space-Time A* (time-expanded A*), the low-level solver of CBS.
 * Search space is the time-expanded graph: a node is (location, timestep), not
 * just location. Move cost is 1 per timestep, waiting also costs 1, therefore
 * g(node) == node.t      and      f(node) == node.t + h(node.loc) which is why
 * a (loc, t) pair never needs re-opening.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <unordered_set>
#include <vector>

#include "algorithm/robotics/astar_variant/map_storage.hpp"
#include "algorithm/robotics/path_finder/path_finder_interface.hpp"
#include "algorithm/robotics/shared_type/constraint.hpp"
#include "algorithm/robotics/shared_type/constraint_table.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"
#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {
/**
 * @brief Space-Time A* for one agent.
 *
 * Used like the other A* variants: SetMapStorageSize(), SetOccupiedGrid() and
 * SetStartAndDestination(), optionally SetConstraints() and SetMaxTimestep(),
 * then either FindPath() or StepOverPathFinding() until it returns
 * std::nullopt, and read the result with GetPath().
 *
 * Changing any input discards the current search. The heuristic is kept
 * across searches until the occupied grid or the destination changes, so
 * re-planning the same agent under new constraints stays cheap.
 */
class SpaceTimeAstar : public PathFinderInterface {
 public:
  /**
   * @brief Allocates the map, discarding the occupied grid and start and
   * destination.
   * @param x_size Number of columns.
   * @param y_size Number of rows.
   */
  void SetMapStorageSize(const size_t x_size, const size_t y_size);

  // Interface inherited
  bool SetOccupiedGrid(const std::vector<Coordinate> &occupied_grid) override;

  bool SetStartAndDestination(const Coordinate &start,
                              const Coordinate &dest) override;

  /**
   * @brief Expands the best node on the open list.
   * @return The cells this step added to the open list (possibly none), or
   * std::nullopt once the search is over: the path was found, the open list
   * ran out, or no start and destination is set.
   */
  std::optional<std::vector<Coordinate>> StepOverPathFinding() override;

  /**
   * @brief Runs the search until it finishes.
   * @return true if a constrained path reaches the destination by the max
   * timestep.
   */
  bool FindPath() override;

  /**
   * @brief Returns the path found by the current search.
   * @return One coordinate per timestep, with `path[0] == start` and
   * `path.back() == dest`; std::nullopt until the search has reached the
   * destination.
   */
  std::optional<std::vector<Coordinate>> GetPath() override;

  /**
   * @brief Clears the occupied grid, start and destination, constraints, max
   * timestep and the search. The map size is kept.
   */
  void Reset() override;

  // Space-time specific
  /**
   * @brief Sets the CBS constraints for the agent.
   * @param constraints Constraints on the agent.
   */
  void SetConstraints(const std::vector<Constraint> &constraints);

  /**
   * @brief Sets the latest timestep at which the path may reach the
   * destination. There is no limit until this is called.
   * @param max_timestep The limit.
   */
  void SetMaxTimestep(const int max_timestep);

  /**
   * @brief Expands the best node on the open list, like StepOverPathFinding(),
   * but keeps the timestep of what it added.
   * @return The states this step added to the open list (possibly none), or
   * std::nullopt once the search is over: the path was found, the open list
   * ran out, or no start and destination is set.
   */
  std::optional<std::vector<State>> StepOverSpaceTimePathFinding();

 private:
  /** @brief Distance for cells from which the goal cannot be reached. */
  static constexpr int kInf = std::numeric_limits<int>::max();
  /** @brief Max timestep used until SetMaxTimestep() is called. */
  static constexpr int kNoTimestepLimit = std::numeric_limits<int>::max();

  /** @brief A node of the search tree. */
  struct Node {
    Coordinate loc;  ///< Cell.
    int t;           ///< Timestep; equals g under unit costs.
    int parent;      ///< Index into nodes_, -1 for the root.
  };

  /**
   * @brief An open-list entry.
   *
   * std::priority_queue pops its largest element, so "less than" here means
   * "expand later".
   */
  struct OpenEntry {
    int f;     ///< t + h(loc).
    int t;     ///< Timestep, for tie-breaking.
    int node;  ///< Index into nodes_.
    /**
     * @brief Orders entries for std::priority_queue.
     * @param other Entry to compare with.
     * @return true if this entry should be expanded after `other`.
     */
    bool operator<(const OpenEntry &other) const {
      if (f != other.f) return f > other.f;  // min-heap on f
      return t < other.t;  // tie-break: prefer larger g (smaller h)
    }
  };

  /** @brief Discards the current search; the next step begins a new one. */
  void ClearSearch();

  /** @brief Starts a new search from start_, on a cleared search. */
  void BeginSearch();

  /**
   * @brief Backward BFS from the destination, ignoring all constraints and all
   * other agents.
   *
   * This is the perfect distance on the static graph: admissible, consistent,
   * and far stronger than Manhattan distance on maps with obstacles.
   *
   * @return Each cell's distance to the destination, indexed [x][y]; kInf
   * where the destination cannot be reached, and everywhere if it is occupied.
   */
  [[nodiscard]] std::vector<std::vector<int>> BackwardBfs() const;

  /**
   * @brief Checks whether a cell is free.
   * @param coordinate A cell inside the map.
   * @return true if the cell is not occupied.
   */
  [[nodiscard]] bool IsFree(const Coordinate &coordinate) const;

  /**
   * @brief Returns the free 4-connected neighbours of a cell.
   *
   * The wait action is not included; the search adds it itself.
   *
   * @param coordinate A cell inside the map.
   * @return The free cells one step away, in motion_constraint_ order.
   */
  [[nodiscard]] std::vector<Coordinate> GetNeighbours(
      const Coordinate &coordinate) const;

  /**
   * @brief Returns the heuristic value of a cell.
   * @param c A cell inside the map.
   * @return Its distance to the destination ignoring constraints, or kInf if
   * the destination cannot be reached from it.
   */
  [[nodiscard]] int Heuristic(const Coordinate &c) const;

  /// Cardinal moves in the order +x, -x, +y, -y. The order decides which of
  /// several equally short paths is returned, so it is not the order of
  /// GetMotionConstraint(MotionConstraintType::CARDINAL_MOTION).
  MotionConstraint motion_constraint_{{1, -1, 0, 0}, {0, 0, 1, -1}};
  std::unique_ptr<MapStorage> map_storage_;  ///< Occupied grid and bounds.
  std::vector<std::vector<int>>
      heuristic_;  ///< Indexed [x][y]; empty until the next search needs it.

  std::vector<Constraint> constraints_;  ///< See SetConstraints().
  int max_timestep_{kNoTimestepLimit};   ///< See SetMaxTimestep().
  Coordinate start_{};                   ///< Start cell, at timestep 0.
  Coordinate dest_{};                    ///< Destination cell.
  bool start_and_end_set_{false};        ///< Start and destination are set.

  // State of the current search, cleared by ClearSearch().
  std::optional<ConstraintTable>
      constraint_table_;     ///< Constraints of the search; set once it begins.
  std::vector<Node> nodes_;  ///< Every node generated so far.
  std::unordered_set<State, StateHash>
      generated_;                        ///< (loc, t) already generated.
  std::priority_queue<OpenEntry> open_;  ///< Open list.
  int goal_node_{-1};                    ///< nodes_ index of goal, else -1.
};

}  // namespace pllee4::graph
