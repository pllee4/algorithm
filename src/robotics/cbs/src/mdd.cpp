/*
 * mdd.cpp
 *
 * Created on: Oct 15, 2026 21:00
 * Description: Implementation of the MDD builder, see mdd.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/mdd.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <queue>
#include <unordered_set>

#include "algorithm/robotics/shared_type/constraint_table.hpp"

namespace pllee4::graph {
namespace {
constexpr int kUnreachable = std::numeric_limits<int>::max();
constexpr std::array<int, 4> kDx = {1, -1, 0, 0};
constexpr std::array<int, 4> kDy = {0, 0, 1, -1};

[[nodiscard]] bool InBounds(int x, int y, int width, int height) {
  return x >= 0 && x < width && y >= 0 && y < height;
}

[[nodiscard]] std::vector<Coordinate> FreeNeighbors(
    const Coordinate &location, int width, int height,
    const std::vector<std::vector<bool>> &free) {
  auto neighbors = std::vector<Coordinate>{};
  for (int k = 0; k < 4; ++k) {
    const int nx = location.x + kDx[k];
    const int ny = location.y + kDy[k];
    if (InBounds(nx, ny, width, height) && free[nx][ny])
      neighbors.push_back(Coordinate{nx, ny});
  }
  return neighbors;
}

[[nodiscard]] std::vector<std::vector<int>> ComputeHeuristic(
    int width, int height, const std::vector<std::vector<bool>> &free,
    const Coordinate &goal) {
  auto distance = std::vector<std::vector<int>>(
      width, std::vector<int>(height, kUnreachable));
  if (!free[goal.x][goal.y]) return distance;
  distance[goal.x][goal.y] = 0;
  auto frontier = std::queue<Coordinate>{};
  frontier.push(goal);
  while (!frontier.empty()) {
    const auto current = frontier.front();
    frontier.pop();
    for (const auto &neighbor : FreeNeighbors(current, width, height, free)) {
      if (distance[neighbor.x][neighbor.y] != kUnreachable) continue;
      distance[neighbor.x][neighbor.y] = distance[current.x][current.y] + 1;
      frontier.push(neighbor);
    }
  }
  return distance;
}
}  // namespace

MDD::MDD(int width, int height, const std::vector<Coordinate> &obstacles,
         const Coordinate &start, const Coordinate &goal,
         const std::vector<Constraint> &constraints, int agent,
         int optimal_cost) {
  auto free =
      std::vector<std::vector<bool>>(width, std::vector<bool>(height, true));
  for (const auto &obstacle : obstacles) free[obstacle.x][obstacle.y] = false;

  const auto constraint_table = ConstraintTable(constraints, goal, agent);
  const auto heuristic = ComputeHeuristic(width, height, free, goal);

  // Forward pass: generate all reachable cells at each timestep.
  levels_.resize(optimal_cost + 1);
  levels_[0] = {start};

  for (int t = 0; t < optimal_cost; ++t) {
    auto next_locations = std::unordered_set<Coordinate, CoordinateHash>{};
    for (const auto &location : levels_[t]) {
      auto successors = FreeNeighbors(location, width, height, free);
      successors.push_back(location);
      for (const auto &next : successors) {
        const int next_t = t + 1;
        if (heuristic[next.x][next.y] == kUnreachable) continue;
        if (next_t + heuristic[next.x][next.y] > optimal_cost) continue;
        if (constraint_table.IsBlocked(next, next_t)) continue;
        if (constraint_table.IsBlocked(location, next, next_t)) continue;
        next_locations.insert(next);
      }
    }
    levels_[t + 1].assign(next_locations.begin(), next_locations.end());
  }

  // Last level: only the goal can survive.
  auto &last_level = levels_[optimal_cost];
  last_level.erase(
      std::remove_if(last_level.begin(), last_level.end(),
                     [&goal](const Coordinate &c) { return !(c == goal); }),
      last_level.end());

  // Backward pass: prune cells that cannot reach a surviving next level.
  for (int t = optimal_cost - 1; t >= 0; --t) {
    const auto surviving = std::unordered_set<Coordinate, CoordinateHash>(
        levels_[t + 1].begin(), levels_[t + 1].end());
    auto pruned = std::vector<Coordinate>{};
    for (const auto &location : levels_[t]) {
      auto successors = FreeNeighbors(location, width, height, free);
      successors.push_back(location);
      bool has_child = false;
      for (const auto &next : successors) {
        if (surviving.count(next) > 0 &&
            !constraint_table.IsBlocked(location, next, t + 1)) {
          has_child = true;
          break;
        }
      }
      if (has_child) pruned.push_back(location);
    }
    levels_[t] = std::move(pruned);
  }
}

int MDD::GetWidth(int timestep) const {
  if (timestep < 0 || timestep >= static_cast<int>(levels_.size())) return 0;
  return static_cast<int>(levels_[timestep].size());
}

bool MDD::IsSingleton(int timestep) const { return GetWidth(timestep) == 1; }

int MDD::GetNumLevels() const { return static_cast<int>(levels_.size()); }

const std::vector<Coordinate> &MDD::GetLevel(int timestep) const {
  return levels_.at(timestep);
}

}  // namespace pllee4::graph
