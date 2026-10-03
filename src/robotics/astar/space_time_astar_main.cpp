/*
 * space_time_astar_main.cpp
 *
 * Created on: Sep 13, 2026 15:05
 * Description: Demo of the Space-Time A* solver in space_time_astar.hpp.
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include <cstddef>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "algorithm/robotics/astar/space_time_astar.hpp"

using namespace pllee4::graph;

int main(int argc, char **argv) {
  //  . . . . . .
  //  . # # # # .
  //  S . . . . G      (y = 2)
  //  . # # # # .
  //  . . . . . .
  const auto kWidth = 6, kHeight = 5;
  std::vector<Coordinate> occupied_grid;
  for (auto x = 1; x <= 4; ++x) {
    occupied_grid.push_back({x, 1});
    occupied_grid.push_back({x, 3});
  }

  const auto start = Coordinate{0, 2};
  const auto goal = Coordinate{5, 2};

  SpaceTimeAstar planner;
  planner.SetMapStorageSize(kWidth, kHeight);
  if (!planner.SetOccupiedGrid(occupied_grid)) {
    std::cout << "Invalid occupancy grid!" << std::endl;
  }
  planner.SetStartAndDestination(start, goal);
  planner.SetMaxTimestep(64);

  const auto show = [](const std::string &label,
                       const std::optional<std::vector<Coordinate>> &p) {
    if (!p) {
      std::cout << label << ": no path\n\n";
      return;
    }
    std::cout << label << ": cost " << p->size() - 1 << " (timesteps 0.."
              << p->size() - 1 << ")\n  ";
    for (auto t = std::size_t{0}; t < p->size(); ++t) {
      const auto &c = (*p)[t];
      std::cout << "t" << t << "=(" << c.x << "," << c.y << ") ";
    }
    std::cout << "\n\n";
  };

  // 1. No constraints: straight corridor run.
  planner.FindPath();
  show("unconstrained", planner.GetPath());

  // 2. Another agent (from CBS) blocks cell (3,2) at t=3 and the swap 2->3 at
  //    t=3. The agent must wait one timestep instead.
  planner.SetConstraints({
      Constraint::Vertex({3, 2}, 3),
      Constraint::Edge({2, 2}, {3, 2}, 3),
  });
  planner.FindPath();
  show("blocked at (3,2) t=3", planner.GetPath());

  // 3. The goal itself is constrained at t=5, so arriving at t=5 is not enough:
  //    the holding time pushes the arrival later.
  planner.SetConstraints(
      {Constraint::Vertex(goal, 5), Constraint::Vertex(goal, 6)});
  planner.FindPath();
  show("goal blocked until t=6", planner.GetPath());

  // 4. The corridor cell (4,2) is sealed for the whole horizon, so the only
  //    option left is the long detour through the y = 0 row.
  std::vector<Constraint> corridor_sealed;
  for (auto t = 0; t <= 64; ++t)
    corridor_sealed.push_back(Constraint::Vertex({4, 2}, t));
  planner.SetConstraints(corridor_sealed);
  planner.FindPath();
  show("corridor sealed", planner.GetPath());

  // 5. The unconstrained search again, one expansion at a time.
  planner.SetConstraints({});
  auto step = 0;
  while (const auto new_states = planner.StepOverSpaceTimePathFinding()) {
    std::cout << "step " << ++step << ":";
    for (const auto &state : *new_states)
      std::cout << " (" << state.at.x << "," << state.at.y << ")@t" << state.t;
    std::cout << "\n";
  }
  show("stepped", planner.GetPath());

  return 0;
}
