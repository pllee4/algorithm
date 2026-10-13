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

std::vector<int> CBS::AgentsToReplan(
    const CTNode &parent,
    const std::vector<Constraint> &new_constraints) const {
  if (new_constraints.empty()) return {};
  const auto &front = new_constraints.front();
  auto agents = std::vector<int>{};

  if (const auto *leq = std::get_if<LeqLengthConstraint>(&front)) {
    for (int agent = 0; agent < static_cast<int>(agents_.size()); ++agent) {
      if (agent == leq->agent) continue;
      const auto &path = parent.paths[agent];
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
  root->paths.resize(agents_.size());
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    root->paths[i] = PlanAgent(i, {});
    if (root->paths[i].empty()) return result;
  }
  root->g = GetSumOfCosts(root->paths);

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

    auto conflicts = FindAllConflicts(node->paths);
    if (conflicts.empty()) {
      result.solution = node->paths;
      result.cost = node->g;
      result.makespan = GetMakespan(node->paths);
      result.solved = true;
      return result;
    }

    // Choose the best conflict to branch on.
    auto best = conflicts.begin();
    for (auto it = std::next(best); it != conflicts.end(); ++it) {
      if (*best < *it) best = it;
    }

    for (int side = 0; side < 2; ++side) {
      const auto &new_cs =
          (side == 0) ? best->constraints1 : best->constraints2;
      auto child = std::make_shared<CTNode>();
      child->constraints = node->constraints;
      child->constraints.insert(child->constraints.end(), new_cs.begin(),
                                new_cs.end());
      child->paths = node->paths;
      child->depth = node->depth + 1;

      auto to_replan = AgentsToReplan(*node, new_cs);
      auto feasible = true;
      for (const auto ag : to_replan) {
        auto path = PlanAgent(ag, child->constraints);
        if (path.empty()) {
          feasible = false;
          break;
        }
        child->paths[ag] = std::move(path);
      }
      if (feasible) {
        child->g = GetSumOfCosts(child->paths);
        open.push(std::move(child));
        ++result.generated;
      }
    }
  }
  return result;
}

}  // namespace pllee4::graph
