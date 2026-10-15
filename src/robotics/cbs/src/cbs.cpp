/*
 * cbs.cpp
 *
 * Created on: Oct 04, 2026 10:00
 * Description: Implementation of CBS high-level search, see cbs.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/cbs.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <queue>

namespace pllee4::graph {

CBS::CBS(int width, int height, const std::vector<Coordinate> &obstacles,
         const std::vector<Agent> &agents)
    : agents_(agents), planners_(agents.size()) {
  for (int i = 0; i < static_cast<int>(agents.size()); ++i) {
    planners_[i].SetMapStorageSize(width, height);
    planners_[i].SetOccupiedGrid(obstacles);
    planners_[i].SetStartAndDestination(agents[i].start, agents[i].goal);
    planners_[i].SetAgent(i);
  }
}

Path CBS::PlanAgent(int agent, const std::vector<Constraint> &constraints) {
  planners_[agent].SetConstraints(constraints);
  if (!planners_[agent].FindPath()) return {};
  return planners_[agent].GetPath().value_or(Path{});
}

Solution CBS::GetSolution(const CTNode &node) const {
  auto solution = Solution(agents_.size());
  auto found = std::vector<bool>(agents_.size(), false);
  const CTNode *current = &node;
  while (current != nullptr) {
    for (const auto &[agent, path] : current->new_paths) {
      if (!found[agent]) {
        solution[agent] = path;
        found[agent] = true;
      }
    }
    current = current->parent.get();
  }
  return solution;
}

std::vector<Constraint> CBS::GetAllConstraints(const CTNode &node) const {
  auto constraints = std::vector<Constraint>{};
  const CTNode *current = &node;
  while (current != nullptr) {
    constraints.insert(constraints.end(), current->new_constraints.begin(),
                       current->new_constraints.end());
    current = current->parent.get();
  }
  return constraints;
}

std::vector<int> CBS::AgentsToReplan(
    const Solution &parent_solution,
    const std::vector<Constraint> &new_constraints) const {
  if (new_constraints.empty()) return {};
  const auto &front = new_constraints.front();
  auto agents = std::vector<int>{};

  if (const auto *leq = std::get_if<LeqLengthConstraint>(&front)) {
    for (int agent = 0; agent < static_cast<int>(agents_.size()); ++agent) {
      if (agent == leq->agent) continue;
      const auto &path = parent_solution[agent];
      for (int t = leq->t; t < static_cast<int>(path.size()); ++t) {
        if (path[t] == leq->goal) {
          agents.push_back(agent);
          break;
        }
      }
      if (!path.empty() && (static_cast<int>(path.size()) - 1 < leq->t) &&
          (path.back() == leq->goal)) {
        agents.push_back(agent);
      }
    }
  } else {
    agents.push_back(GetAgent(front));
  }
  return agents;
}

CBS::Result CBS::Solve(int time_limit_ms) {
  auto result = Result{};
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(time_limit_ms);

  // Root: plan each agent independently.
  auto root = std::make_shared<CTNode>();
  root->new_paths.reserve(agents_.size());
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    auto path = PlanAgent(i, {});
    if (path.empty()) return result;
    root->new_paths.emplace_back(i, std::move(path));
  }
  root->g = GetSumOfCosts(GetSolution(*root));

  auto cmp = [](const std::shared_ptr<CTNode> &a,
                const std::shared_ptr<CTNode> &b) {
    if (a->g != b->g) return (a->g > b->g);
    return (a->depth < b->depth);
  };
  auto open =
      std::priority_queue<std::shared_ptr<CTNode>,
                          std::vector<std::shared_ptr<CTNode>>, decltype(cmp)>{
          cmp};
  open.push(root);
  result.generated = 1;

  while (!open.empty()) {
    if (std::chrono::steady_clock::now() >= deadline) return result;

    auto node = open.top();
    open.pop();
    ++result.expanded;

    auto solution = GetSolution(*node);
    auto conflicts = FindAllConflicts(solution);
    if (conflicts.empty()) {
      result.solution = std::move(solution);
      result.cost = node->g;
      result.makespan = GetMakespan(result.solution);
      result.solved = true;
      return result;
    }

    // Choose the best conflict to branch on.
    auto best = conflicts.begin();
    for (auto it = std::next(best); it != conflicts.end(); ++it) {
      if (*best < *it) best = it;
    }

    for (int side = 0; side < 2; ++side) {
      const auto &new_constraints =
          (side == 0) ? best->constraints1 : best->constraints2;
      auto child = std::make_shared<CTNode>();
      child->parent = node;
      child->new_constraints = new_constraints;
      child->depth = node->depth + 1;

      auto all_constraints = GetAllConstraints(*child);
      auto to_replan = AgentsToReplan(solution, new_constraints);
      auto feasible = true;
      for (const auto agent : to_replan) {
        auto path = PlanAgent(agent, all_constraints);
        if (path.empty()) {
          feasible = false;
          break;
        }
        child->new_paths.emplace_back(agent, std::move(path));
      }
      if (feasible) {
        auto child_solution = solution;
        for (const auto &[agent, path] : child->new_paths) {
          child_solution[agent] = path;
        }
        child->g = GetSumOfCosts(child_solution);
        open.push(std::move(child));
        ++result.generated;
      }
    }
  }
  return result;
}

}  // namespace pllee4::graph
