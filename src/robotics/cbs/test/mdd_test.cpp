/*
 * mdd_test.cpp
 *
 * Created on: Oct 15, 2026 12:00
 * Description:
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/mdd.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

// ---- basic structure ----

TEST(MDD, StraightPathAllSingletons) {
  // 5x1 corridor: (0,0) -> (4,0), cost 4.
  // Only one optimal path exists, so every level is a singleton.
  MDD mdd(5, 1, {}, Coordinate{0, 0}, Coordinate{4, 0}, {}, 0, 4);
  EXPECT_EQ(mdd.GetNumLevels(), 5);
  for (int t = 0; t <= 4; ++t) {
    EXPECT_TRUE(mdd.IsSingleton(t)) << "level " << t;
  }
}

TEST(MDD, OpenGridHasMultiplePaths) {
  // 3x3 open grid: (0,0) -> (2,2), cost 4.
  // Multiple optimal paths exist (go right-right-down-down, right-down-right-down, etc.).
  // Middle levels should have width > 1.
  MDD mdd(3, 3, {}, Coordinate{0, 0}, Coordinate{2, 2}, {}, 0, 4);
  EXPECT_EQ(mdd.GetNumLevels(), 5);
  EXPECT_TRUE(mdd.IsSingleton(0));   // start
  EXPECT_TRUE(mdd.IsSingleton(4));   // goal
  EXPECT_GT(mdd.GetWidth(2), 1);     // middle has choices
}

TEST(MDD, StartEqualsGoal) {
  // Cost 0: agent is already at goal. One level, one node.
  MDD mdd(3, 3, {}, Coordinate{1, 1}, Coordinate{1, 1}, {}, 0, 0);
  EXPECT_EQ(mdd.GetNumLevels(), 1);
  EXPECT_TRUE(mdd.IsSingleton(0));
}

TEST(MDD, ObstacleForcesSingleton) {
  // 3x1 corridor with no obstacle: (0,0) -> (2,0), cost 2.
  // Only one path. All singletons.
  MDD mdd(3, 1, {}, Coordinate{0, 0}, Coordinate{2, 0}, {}, 0, 2);
  EXPECT_EQ(mdd.GetNumLevels(), 3);
  EXPECT_TRUE(mdd.IsSingleton(0));
  EXPECT_TRUE(mdd.IsSingleton(1));
  EXPECT_TRUE(mdd.IsSingleton(2));
}

// ---- constraint effects ----

TEST(MDD, VertexConstraintReducesWidth) {
  // 3x2 grid: (0,0) -> (2,0), cost 2.
  // Paths: right-right via (1,0), or down-right-right-up is cost 4 (too long).
  // So only one path. Width at t=1 is 1.
  MDD mdd_no_constraint(3, 2, {}, Coordinate{0, 0}, Coordinate{2, 0}, {}, 0, 2);
  EXPECT_TRUE(mdd_no_constraint.IsSingleton(1));

  // Now block (1,0) at t=1: no path of cost 2 exists.
  auto constraints = std::vector<Constraint>{VertexConstraint{0, Coordinate{1, 0}, 1}};
  MDD mdd_constrained(3, 2, {}, Coordinate{0, 0}, Coordinate{2, 0}, constraints, 0, 2);
  EXPECT_EQ(mdd_constrained.GetWidth(1), 0);
}

// ---- out-of-range access ----

TEST(MDD, OutOfRangeReturnsZeroWidth) {
  MDD mdd(3, 1, {}, Coordinate{0, 0}, Coordinate{2, 0}, {}, 0, 2);
  EXPECT_EQ(mdd.GetWidth(-1), 0);
  EXPECT_EQ(mdd.GetWidth(10), 0);
}

// ---- backward pruning ----

TEST(MDD, BackwardPassPrunesDeadEnds) {
  // 4x4 grid, (0,0) -> (3,3), cost 6 (= Manhattan distance, zero slack).
  // Constraint: agent 0 blocked from (3,2) at t=5.
  //
  // Without the constraint, the forward pass adds (3,1) at t=4 and (3,0)
  // at t=3 because they lie on paths through (3,2) at t=5.
  // With the constraint, (3,2) is absent from level 5, so:
  //   - (3,1) at t=4 has no h-feasible successor → backward pass prunes it
  //   - (3,0) at t=3 only reached t=4 via (3,1) → cascading prune
  //
  // Paths through (2,3) at t=5 survive.

  auto constraints =
      std::vector<Constraint>{VertexConstraint{0, Coordinate{3, 2}, 5}};

  const auto obstacles = std::vector<Coordinate>{Coordinate{1, 1}};
  const auto start = Coordinate{0, 0};
  const auto goal = Coordinate{3, 3};
  MDD mdd(4, 4, obstacles, start, goal, constraints, 0, 6);

  EXPECT_EQ(mdd.GetNumLevels(), 7);

  // Level 4: without backward pruning would have {(3,1),(2,2),(1,3)}.
  // After pruning (3,1), width should be 2.
  EXPECT_EQ(mdd.GetWidth(4), 2);
  const auto &level4 = mdd.GetLevel(4);
  for (const auto &cell : level4) {
    EXPECT_FALSE(cell.x == 3 && cell.y == 1)
        << "(3,1) should be pruned at t=4";
  }

  // Level 3: without backward pruning would have {(3,0),(2,1),(0,3),(1,2)}.
  // After pruning (3,0), width should be 3.
  EXPECT_EQ(mdd.GetWidth(3), 3);
  const auto &level3 = mdd.GetLevel(3);
  for (const auto &cell : level3) {
    EXPECT_FALSE(cell.x == 3 && cell.y == 0)
        << "(3,0) should be pruned at t=3";
  }

  // Level 5: only (2,3) survives.
  EXPECT_EQ(mdd.GetWidth(5), 1);

  // Start and goal are always present.
  EXPECT_TRUE(mdd.IsSingleton(0));
  EXPECT_TRUE(mdd.IsSingleton(6));
}

// ---- cardinality-relevant scenarios ----

TEST(MDD, CrossingAgentsOneCardinal) {
  // 5x5 grid. Agent crosses horizontally: (0,2) -> (4,2), cost 4.
  // At t=2, agent must be at (2,2) — the midpoint. All singletons along the row
  // if h-value pruning eliminates detours.
  MDD mdd(5, 5, {}, Coordinate{0, 2}, Coordinate{4, 2}, {}, 0, 4);
  // Level 2 should be singleton: (2,2) is the only cell at distance 2 from
  // start and distance 2 from goal on a straight line.
  EXPECT_TRUE(mdd.IsSingleton(2));
}

TEST(MDD, WideGridNonSingleton) {
  // 5x5 grid. (0,0) -> (4,4), cost 8.
  // Many optimal paths. Middle levels should be wide.
  MDD mdd(5, 5, {}, Coordinate{0, 0}, Coordinate{4, 4}, {}, 0, 8);
  EXPECT_TRUE(mdd.IsSingleton(0));
  EXPECT_TRUE(mdd.IsSingleton(8));
  EXPECT_GT(mdd.GetWidth(4), 1);
}
