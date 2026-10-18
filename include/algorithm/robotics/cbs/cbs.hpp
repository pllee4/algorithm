/*
 * cbs.hpp
 *
 * Created on: Oct 04, 2026 10:00
 * Description: Conflict-Based Search (CBS) — the high-level solver. Builds
 * a constraint tree: each node stores only the delta from its parent
 * (new constraints and replanned paths). The full solution and constraint
 * set are reconstructed by walking the parent chain to the root.
 *
 * Key design choices matching CBSH2-RTC:
 *  - Delta storage: each CT node holds only new constraints and replanned
 *    paths, not a full copy. Memory scales with tree depth, not breadth.
 *  - Conflict selection: priority (cardinal > semi > non) then type
 *    (mutex > target > corridor > rectangle > standard).
 *  - LEQLENGTH child triggers a replanning cascade: every agent whose
 *    path crosses the blocked cell at or after t is replanned.
 *  - GLENGTH child replans only the named agent.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "algorithm/robotics/astar/space_time_astar.hpp"
#include "algorithm/robotics/cbs/conflict.hpp"
#include "algorithm/robotics/cbs/mdd.hpp"
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
    std::shared_ptr<CTNode> parent;
    std::vector<Constraint> new_constraints;
    std::vector<std::pair<int, Path>> new_paths;
    int g{};
    int depth{};
  };

  Path PlanAgent(int agent, const std::vector<Constraint> &constraints);

  Solution GetSolution(const CTNode &node) const;

  std::vector<Constraint> GetAllConstraints(const CTNode &node) const;

  void ClassifyConflicts(std::vector<Conflict> &conflicts,
                         const Solution &solution,
                         const std::vector<Constraint> &all_constraints);

  std::vector<int> AgentsToReplan(
      const Solution &parent_solution,
      const std::vector<Constraint> &new_constraints) const;

  int width_;
  int height_;
  std::vector<Coordinate> obstacles_;
  std::vector<Agent> agents_;
  std::vector<SpaceTimeAstar> planners_;
};

}  // namespace pllee4::graph
