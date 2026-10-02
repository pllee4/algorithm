/*
 * space_time_astar_test.cpp
 *
 * Created on: Sep 13, 2026 15:05
 * Description:
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/astar/space_time_astar.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

namespace {
/**
 * s = start, e = end, x = occupied
 *  |   |   |   |   |   |   |
 *  |   | x | x | x | x |   |
 *  | s |   |   |   |   | e |   (y = 2)
 *  |   | x | x | x | x |   |
 *  |   |   |   |   |   |   |
 */
void SetUpCorridor(SpaceTimeAstar &planner) {
  planner.SetMapStorageSize(6, 5);
  planner.SetOccupiedGrid({{1, 1}, {2, 1}, {3, 1}, {4, 1},
                           {1, 3}, {2, 3}, {3, 3}, {4, 3}});
  planner.SetStartAndDestination({0, 2}, {5, 2});
}

const std::vector<Coordinate> kCorridorPath = {{0, 2}, {1, 2}, {2, 2},
                                               {3, 2}, {4, 2}, {5, 2}};
}  // namespace

TEST(SpaceTimeAstar, InvalidSetOccupiedGrid) {
  SpaceTimeAstar planner;
  EXPECT_FALSE(planner.SetOccupiedGrid({{1, 6}}));
  planner.SetMapStorageSize(3, 3);
  EXPECT_FALSE(planner.SetOccupiedGrid({{1, 6}}));
  EXPECT_TRUE(planner.SetOccupiedGrid({{1, 2}}));
}

TEST(SpaceTimeAstar, InvalidSetStartAndDestination) {
  SpaceTimeAstar planner;
  EXPECT_FALSE(planner.SetStartAndDestination({0, 0}, {2, 2}));
  planner.SetMapStorageSize(3, 3);
  EXPECT_FALSE(planner.SetStartAndDestination({0, 0}, {4, 4}));
  EXPECT_FALSE(planner.FindPath());
  EXPECT_FALSE(planner.StepOverPathFinding().has_value());
  EXPECT_FALSE(planner.GetPath().has_value());
}

TEST(SpaceTimeAstar, FindPathWithoutConstraints) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  EXPECT_TRUE(planner.FindPath());
  EXPECT_EQ(planner.GetPath().value(), kCorridorPath);
}

TEST(SpaceTimeAstar, WaitForVertexAndEdgeConstraint) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  planner.SetConstraints(
      {Constraint::Vertex({3, 2}, 3), Constraint::Edge({2, 2}, {3, 2}, 3)});
  EXPECT_TRUE(planner.FindPath());
  const std::vector<Coordinate> expected = {{0, 2}, {1, 2}, {2, 2}, {2, 2},
                                            {3, 2}, {4, 2}, {5, 2}};
  EXPECT_EQ(planner.GetPath().value(), expected);
}

TEST(SpaceTimeAstar, WaitForHoldingTime) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  planner.SetConstraints(
      {Constraint::Vertex({5, 2}, 5), Constraint::Vertex({5, 2}, 6)});
  EXPECT_TRUE(planner.FindPath());
  const std::vector<Coordinate> expected = {{0, 2}, {1, 2}, {2, 2}, {3, 2},
                                            {4, 2}, {4, 2}, {4, 2}, {5, 2}};
  EXPECT_EQ(planner.GetPath().value(), expected);
}

TEST(SpaceTimeAstar, DetourAroundSealedCorridor) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  std::vector<Constraint> constraints;
  for (auto t = 0; t <= 64; ++t)
    constraints.push_back(Constraint::Vertex({4, 2}, t));
  planner.SetConstraints(constraints);
  planner.SetMaxTimestep(64);
  EXPECT_TRUE(planner.FindPath());
  const std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                            {2, 0}, {3, 0}, {4, 0}, {5, 0},
                                            {5, 1}, {5, 2}};
  EXPECT_EQ(planner.GetPath().value(), expected);

  // the detour needs 9 timesteps
  planner.SetMaxTimestep(8);
  EXPECT_FALSE(planner.FindPath());
  EXPECT_FALSE(planner.GetPath().has_value());
}

TEST(SpaceTimeAstar, FailedToFindPath) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);

  // destination occupied
  planner.SetOccupiedGrid({{5, 2}});
  EXPECT_FALSE(planner.FindPath());
  EXPECT_FALSE(planner.GetPath().has_value());

  // nowhere to be at t = 1
  planner.SetMapStorageSize(2, 1);
  planner.SetStartAndDestination({0, 0}, {1, 0});
  planner.SetConstraints(
      {Constraint::Vertex({0, 0}, 1), Constraint::Vertex({1, 0}, 1)});
  EXPECT_FALSE(planner.FindPath());
  EXPECT_FALSE(planner.GetPath().has_value());
}

TEST(SpaceTimeAstar, StepOverPathFinding) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);

  const std::vector<Coordinate> first_step = {{1, 2}, {0, 3}, {0, 1}, {0, 2}};
  EXPECT_EQ(planner.StepOverPathFinding().value(), first_step);

  const auto second_step = planner.StepOverSpaceTimePathFinding().value();
  const std::vector<State> expected = {{{2, 2}, 2}, {{0, 2}, 2}, {{1, 2}, 2}};
  EXPECT_EQ(second_step, expected);

  EXPECT_TRUE(planner.FindPath());
  EXPECT_FALSE(planner.StepOverPathFinding().has_value());
  EXPECT_EQ(planner.GetPath().value(), kCorridorPath);
}

TEST(SpaceTimeAstar, GetPathWithSettingOfSameStartAndDestination) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  EXPECT_TRUE(planner.FindPath());

  // same start and destination, can get back previous path without finding
  planner.SetStartAndDestination({0, 2}, {5, 2});
  EXPECT_TRUE(planner.GetPath().has_value());

  // different start, can't get path without finding
  planner.SetStartAndDestination({1, 2}, {5, 2});
  EXPECT_FALSE(planner.GetPath().has_value());
  EXPECT_TRUE(planner.FindPath());
  EXPECT_EQ(planner.GetPath().value().size(), 5u);

  // new constraints, can't get path without finding
  planner.SetConstraints({Constraint::Vertex({2, 2}, 1)});
  EXPECT_FALSE(planner.GetPath().has_value());
  EXPECT_TRUE(planner.FindPath());
  EXPECT_EQ(planner.GetPath().value().size(), 6u);
}

TEST(SpaceTimeAstar, HeuristicFollowsOccupiedGrid) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  EXPECT_TRUE(planner.FindPath());

  // sealing the corridor after a search must not reuse the old heuristic
  planner.SetOccupiedGrid({{4, 2}});
  EXPECT_TRUE(planner.FindPath());
  EXPECT_EQ(planner.GetPath().value().size(), 10u);
}

TEST(SpaceTimeAstar, GetPathAfterReset) {
  SpaceTimeAstar planner;
  SetUpCorridor(planner);
  planner.SetConstraints({Constraint::Vertex({5, 2}, 5)});
  planner.SetMaxTimestep(5);
  EXPECT_FALSE(planner.FindPath());

  // internally clear path, occupied grid, constraints and max timestep
  planner.Reset();
  EXPECT_FALSE(planner.FindPath());
  EXPECT_FALSE(planner.GetPath().has_value());

  // found path without occupied grid, and without the holding time and max
  // timestep that ruled it out before
  planner.SetStartAndDestination({0, 2}, {5, 2});
  EXPECT_FALSE(planner.GetPath().has_value());
  EXPECT_TRUE(planner.FindPath());
  EXPECT_EQ(planner.GetPath().value().size(), 6u);
}
