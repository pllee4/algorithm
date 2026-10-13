/*
 * cbs.hpp
 *
 * Created on: Oct 04, 2026 10:00
 * Description: Conflict-Based Search (CBS) — the high-level solver. Builds
 * a constraint tree: each node holds a full solution, the best node is
 * expanded by detecting a conflict and branching into two children, each
 * with one side of the split added to its constraint set. The low-level
 * solver (Space-Time A*) replans the affected agents.
 *
 * Key design choices matching CBSH2-RTC:
 *  - Conflict selection: priority (cardinal > semi > non) then type
 *    (mutex > target > corridor > rectangle > standard).
 *  - LEQLENGTH child triggers a replanning cascade: every agent whose
 *    path crosses the blocked cell at or after t is replanned.
 *  - GLENGTH child replans only the named agent.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <vector>

#include "algorithm/robotics/astar/space_time_astar.hpp"
#include "algorithm/robotics/cbs/conflict.hpp"
#include "algorithm/robotics/shared_type/constraint.hpp"
#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {

struct Agent {
  Coordinate start;
  Coordinate goal;
};

/**
 * @brief CBS high-level search.
 *
 * Construct with the grid dimensions, obstacles and agents; then call
 * Solve(). The result contains the optimal (minimum sum-of-costs)
 * conflict-free solution if one exists within the time limit.
 */
class CBS {
 public:
  struct Result {
    Solution solution;
    int cost{};
    int makespan{};
    int expanded{};
    int generated{};
    bool solved{false};
  };

  CBS(int width, int height, const std::vector<Coordinate> &obstacles,
      const std::vector<Agent> &agents);

  Result Solve(int time_limit_ms = 60000);

 private:
  struct CTNode {
    Solution paths;
    std::vector<Constraint> constraints;
    int g{};
    int depth{};
  };

  Path PlanAgent(int agent, const std::vector<Constraint> &constraints);

  std::vector<int> AgentsToReplan(
      const CTNode &parent,
      const std::vector<Constraint> &new_constraints) const;

  std::vector<Agent> agents_;
  std::vector<SpaceTimeAstar> planners_;
};

}  // namespace pllee4::graph
