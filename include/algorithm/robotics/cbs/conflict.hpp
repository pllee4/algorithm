/*
 * conflict.hpp
 *
 * Created on: Oct 03, 2026 20:36
 * Description: Conflict detection for CBS. Finds where agents collide,
 * classifies collisions (standard / target), and generates the per-child
 * constraint lists the high level branches on.
 *
 * Key design choices matching CBSH2-RTC:
 *  - Constraints are per-child, not per-agent: a target conflict constrains
 *    the SAME agent in both children (LEQLENGTH vs GLENGTH).
 *  - Each child gets a vector of constraints, not a single one: corridor
 *    and rectangle splits emit multiple constraints per child.
 *  - operator< selects by priority (cardinal > semi > non) then by type
 *    (mutex > target > corridor > rectangle > standard).
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <optional>
#include <vector>

#include "algorithm/robotics/shared_type/constraint.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
/** @brief One agent's path: the cell it occupies at each timestep. */
using Path = std::vector<Coordinate>;

/** @brief A solution: one path per agent, indexed by agent index. */
using Solution = std::vector<Path>;

/**
 * @brief A collision between two agents and the split it induces.
 *
 * `constraints1` goes to child 1, `constraints2` to child 2. They are
 * per-child, not per-agent: a target conflict puts both constraints on
 * the parked agent (`agent1`), never on the passer-by.
 *
 * Selection: `operator<` returns true when `rhs` has strictly higher
 * priority, so a max-finding loop yields the best conflict to branch on.
 */
struct Conflict {
  /// Conflict types, ordered by selection priority (smaller = better).
  enum class Type {
    kMutex,      ///< Mutex reasoning: proven cardinal.
    kTarget,     ///< Parked agent at goal: length split.
    kCorridor,   ///< Corridor symmetry: range split.
    kRectangle,  ///< Rectangle symmetry: barrier split.
    kStandard,   ///< Plain vertex or edge.
  };

  /// Cardinality, decided by MDD analysis (smaller = better).
  enum class Priority {
    kCardinal,  ///< Both children cost >= parent + 1.
    kSemi,      ///< One child costs >= parent + 1.
    kNon,       ///< Neither child necessarily costs more.
    kUnknown,   ///< Not yet classified.
  };

  Type type{Type::kStandard};
  Priority priority{Priority::kUnknown};
  int agent1{};    ///< First agent (for target: parked agent).
  int agent2{};    ///< Second agent (for target: passer-by).
  int timestep{};  ///< When the collision occurs.
  std::vector<Constraint> constraints1;  ///< Constraints for child 1.
  std::vector<Constraint> constraints2;  ///< Constraints for child 2.

  /** @brief Standard vertex conflict: both agents at `cell` at `t`. */
  static Conflict Vertex(int a1, int a2, const Coordinate &cell, int t);

  /** @brief Standard edge conflict: agents swap `v1` <-> `v2` at `t`. */
  static Conflict Edge(int a1, int a2, const Coordinate &v1,
                       const Coordinate &v2, int t);

  /**
   * @brief Target conflict: `parked` sits on `goal` at `t`.
   *
   * Both constraints name `parked`:
   *   child 1 = LEQLENGTH(parked, goal, t)  ->  path <= t
   *   child 2 = GLENGTH(parked, goal, t)    ->  path >= t + 1
   * The passer-by is never directly constrained; instead, the LEQLENGTH
   * child blocks the goal cell for all non-owners via ConstraintTable,
   * and generateChild replans every agent whose path crosses it.
   */
  static Conflict Target(int parked, int passer, const Coordinate &goal,
                         int t);
};

/**
 * @brief Conflict selection: true when `rhs` is strictly better.
 *
 * Orders by priority (CARDINAL > SEMI > NON > UNKNOWN), then by type
 * (MUTEX > TARGET > CORRIDOR > RECTANGLE > STANDARD). Equal conflicts
 * compare as false — the caller may break ties randomly.
 */
bool operator<(const Conflict &lhs, const Conflict &rhs);

/**
 * @brief Where a path is at timestep `t`, parked at its goal past the end.
 * @return The cell, or {-1, -1} if the path is empty.
 */
[[nodiscard]] Coordinate PositionAt(const Path &path, int t);

/**
 * @brief Finds the earliest collision between two paths.
 * @return The earliest conflict, or std::nullopt if none.
 */
[[nodiscard]] std::optional<Conflict> FindFirstConflict(const Path &path1,
                                                        const Path &path2,
                                                        int agent1,
                                                        int agent2);

/**
 * @brief Finds the earliest collision in a solution (timestep-outermost).
 * @return The earliest conflict, or std::nullopt if valid.
 */
[[nodiscard]] std::optional<Conflict> FindFirstConflict(
    const Solution &solution);

/**
 * @brief Finds every collision in a solution, ordered by timestep.
 * @return All conflicts. Feed to operator< for selection.
 */
[[nodiscard]] std::vector<Conflict> FindAllConflicts(const Solution &solution);

/** @brief True if the solution has no collisions. */
[[nodiscard]] bool IsConflictFree(const Solution &solution);

/** @brief Sum of costs: the CBS objective. */
[[nodiscard]] int GetSumOfCosts(const Solution &solution);

/** @brief Makespan: timestep the last agent arrives. */
[[nodiscard]] int GetMakespan(const Solution &solution);

}  // namespace pllee4::graph
