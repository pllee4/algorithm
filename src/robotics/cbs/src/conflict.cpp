/*
 * conflict.cpp
 *
 * Created on: Oct 03, 2026 20:36
 * Description: Implementation of CBS conflict detection, see conflict.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/conflict.hpp"

#include <algorithm>

namespace pllee4::graph {
namespace {
constexpr Coordinate kNowhere{-1, -1};

[[nodiscard]] int GetHorizon(const Solution &solution) {
  std::size_t longest = 0;
  for (const auto &path : solution) longest = std::max(longest, path.size());
  return static_cast<int>(longest);
}

/// True when the agent has arrived at its final cell and is parked there.
[[nodiscard]] bool IsParked(const Path &path, int t) {
  return (!path.empty() && (static_cast<size_t>(t + 1) >= path.size()));
}

/**
 * @brief Checks two paths for a collision at one timestep.
 *
 * Vertex conflicts at a parked agent's goal are classified as target
 * conflicts, which generate LEQLENGTH / GLENGTH constraints instead of
 * plain vertex constraints. This matches CBSH2-RTC's findConflicts.
 */
[[nodiscard]] std::optional<Conflict> ConflictAt(const Path &path1,
                                                 const Path &path2, int agent1,
                                                 int agent2, int t) {
  if (path1.empty() || path2.empty()) return std::nullopt;

  const auto at1 = PositionAt(path1, t);
  const auto at2 = PositionAt(path2, t);

  // Vertex conflict: same cell, same time.
  if (at1 == at2) {
    const auto parked1 = IsParked(path1, t);
    const auto parked2 = IsParked(path2, t);
    if (parked1 && parked2) {
      // Both arrived: the shorter path finished first and is the truly
      // parked agent. Same-length means same arrival time (degenerate).
      if (path1.size() < path2.size())
        return Conflict::Target(agent1, agent2, at1, t);
      if (path2.size() < path1.size())
        return Conflict::Target(agent2, agent1, at2, t);
      return Conflict::Vertex(agent1, agent2, at1, t);
    }
    if (parked1) return Conflict::Target(agent1, agent2, at1, t);
    if (parked2) return Conflict::Target(agent2, agent1, at2, t);
    return Conflict::Vertex(agent1, agent2, at1, t);
  }

  // Edge (swap) conflict: agents trade places.
  if (t == 0) return std::nullopt;
  const Coordinate from1 = PositionAt(path1, t - 1);
  const Coordinate from2 = PositionAt(path2, t - 1);
  if (at1 == from2 && at2 == from1)
    return Conflict::Edge(agent1, agent2, from1, at1, t);

  return std::nullopt;
}
}  // namespace

// --- Conflict factories ------------------------------------------------

Conflict Conflict::Vertex(int a1, int a2, const Coordinate &cell, int t) {
  return {Type::kStandard,
          Priority::kUnknown,
          a1,
          a2,
          t,
          {VertexConstraint{a1, cell, t}},
          {VertexConstraint{a2, cell, t}}};
}

Conflict Conflict::Edge(int a1, int a2, const Coordinate &v1,
                        const Coordinate &v2, int t) {
  return {Type::kStandard,
          Priority::kUnknown,
          a1,
          a2,
          t,
          {EdgeConstraint{a1, v1, v2, t}},
          {EdgeConstraint{a2, v2, v1, t}}};
}

Conflict Conflict::Target(int parked, int passer, const Coordinate &goal,
                          int t) {
  return {Type::kTarget,
          Priority::kUnknown,
          parked,
          passer,
          t,
          {LeqLengthConstraint{parked, goal, t}},
          {GLengthConstraint{parked, goal, t}}};
}

// --- Conflict selection ------------------------------------------------

bool operator<(const Conflict &lhs, const Conflict &rhs) {
  if (lhs.priority != rhs.priority)
    return static_cast<int>(lhs.priority) > static_cast<int>(rhs.priority);
  if (lhs.type != rhs.type)
    return static_cast<int>(lhs.type) > static_cast<int>(rhs.type);
  return false;  // ties unbroken: caller may randomise
}

// --- Free functions ----------------------------------------------------

Coordinate PositionAt(const Path &path, int t) {
  if (path.empty()) return kNowhere;
  if (t >= static_cast<int>(path.size())) return path.back();
  return path.at(t);
}

std::optional<Conflict> FindFirstConflict(const Path &path1, const Path &path2,
                                          int agent1, int agent2) {
  if (path1.empty() || path2.empty()) return std::nullopt;
  const auto horizon = static_cast<int>(std::max(path1.size(), path2.size()));
  for (auto t = 0; t < horizon; ++t) {
    if (auto conflict = ConflictAt(path1, path2, agent1, agent2, t)) return conflict;
  }
  return std::nullopt;
}

std::optional<Conflict> FindFirstConflict(const Solution &solution) {
  const int horizon = GetHorizon(solution);
  const int total_agent = static_cast<int>(solution.size());
  for (auto t = 0; t < horizon; ++t) {
    for (auto i = 0; i < total_agent; ++i) {
      for (auto j = i + 1; j < total_agent; ++j) {
        if (auto conflict = ConflictAt(solution[i], solution[j], i, j, t)) return conflict;
      }
    }
  }
  return std::nullopt;
}

std::vector<Conflict> FindAllConflicts(const Solution &solution) {
  const int horizon = GetHorizon(solution);
  const int total_agent = static_cast<int>(solution.size());
  std::vector<Conflict> conflicts;
  for (int t = 0; t < horizon; ++t) {
    for (int i = 0; i < total_agent; ++i) {
      for (int j = i + 1; j < total_agent; ++j) {
        if (auto c = ConflictAt(solution[i], solution[j], i, j, t))
          conflicts.push_back(*c);
      }
    }
  }
  return conflicts;
}

bool IsConflictFree(const Solution &solution) {
  return (!FindFirstConflict(solution).has_value());
}

int GetSumOfCosts(const Solution &solution) {
  int total = 0;
  for (const auto &path : solution) {
    if (path.empty()) continue;
    total += static_cast<int>(path.size()) - 1;
  }
  return total;
}

int GetMakespan(const Solution &solution) {
  int makespan = 0;
  for (const auto &path : solution) {
    if (path.empty()) continue;
    makespan = std::max(makespan, static_cast<int>(path.size()) - 1);
  }
  return makespan;
}

}  // namespace pllee4::graph
