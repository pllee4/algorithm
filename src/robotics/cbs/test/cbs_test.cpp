/*
 * cbs_test.cpp
 *
 * Created on: Oct 04, 2026 10:00
 * Description:
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/cbs.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

// ---- helpers ----

namespace {
void ExpectOptimal(const CBS::Result& r, int expected_soc) {
  ASSERT_TRUE(r.solved);
  EXPECT_EQ(r.cost, expected_soc);
  EXPECT_TRUE(IsConflictFree(r.solution));
}
}  // namespace

// ---- single agent ----

TEST(CBS, SingleAgent) {
  const auto r = CBS(3, 1, {}, {{Coordinate{0, 0}, Coordinate{2, 0}}}).Solve();
  ExpectOptimal(r, 2);
  EXPECT_EQ(r.makespan, 2);
}

// ---- two agents, no conflict ----

TEST(CBS, DisjointPaths) {
  CBS cbs(3, 3, {},
          {{Coordinate{0, 0}, Coordinate{2, 0}},
           {Coordinate{0, 2}, Coordinate{2, 2}}});
  const auto r = cbs.Solve();
  ExpectOptimal(r, 4);
}

// ---- two agents, head-on swap ----

TEST(CBS, HeadOnSwap) {
  // 3x2 grid. Agents must swap ends of row y=0. Every same-row path
  // hits a vertex or edge conflict, so one agent must detour through
  // row y=1 (cost +2). SOC = 2 + 4 = 6.
  CBS cbs(3, 2, {},
          {{Coordinate{0, 0}, Coordinate{2, 0}},
           {Coordinate{2, 0}, Coordinate{0, 0}}});
  const auto r = cbs.Solve();
  ExpectOptimal(r, 6);
}

// ---- target conflict ----

TEST(CBS, TargetConflict) {
  // 3x2 grid.
  // Agent 0: (0,0) -> (1,0)  parks at t=1.
  // Agent 1: (2,0) -> (0,0)  direct path crosses (1,0) at t=1.
  // SOC = 5: agent 0 cost 1 (direct), agent 1 detours through y=1
  // to avoid the parked cell: (2,0)->(2,1)->(1,1)->(0,1)->(0,0) = 4.
  CBS cbs(3, 2, {},
          {{Coordinate{0, 0}, Coordinate{1, 0}},
           {Coordinate{2, 0}, Coordinate{0, 0}}});
  const auto r = cbs.Solve();
  ExpectOptimal(r, 5);
}

// ---- three agents ----

TEST(CBS, ThreeAgentsOneDisjoint) {
  // 3x3 grid. Agents 0 and 1 swap on row y=0 (SOC 6), agent 2 is
  // independent on row y=2 (cost 2). Total SOC = 8.
  CBS cbs(3, 3, {},
          {{Coordinate{0, 0}, Coordinate{2, 0}},
           {Coordinate{2, 0}, Coordinate{0, 0}},
           {Coordinate{0, 2}, Coordinate{2, 2}}});
  const auto r = cbs.Solve();
  ExpectOptimal(r, 8);
}

// ---- obstacles ----

TEST(CBS, ObstacleForcesDetour) {
  // 3x2 grid with obstacle at (1,0).
  // Shortest: (0,0)->(0,1)->(1,1)->(2,1)->(2,0), cost 4.
  CBS cbs(3, 2, {Coordinate{1, 0}}, {{Coordinate{0, 0}, Coordinate{2, 0}}});
  const auto r = cbs.Solve();
  ExpectOptimal(r, 4);
}

// ---- unsolvable ----

TEST(CBS, UnsolvableReturnsNotSolved) {
  CBS cbs(3, 1, {Coordinate{1, 0}}, {{Coordinate{0, 0}, Coordinate{2, 0}}});
  const auto r = cbs.Solve();
  EXPECT_FALSE(r.solved);
}

// ---- infeasible replan (target conflict on tiny grid) ----

TEST(CBS, InfeasibleReplanOnTinyGrid) {
  // 2x1 grid. Agent 0: (0,0) -> (1,0). Agent 1: (1,0) -> (1,0) (trivial).
  // Target conflict at t=1: both parked at (1,0), agent 1 shorter.
  // LeqLength child blocks (1,0) for agent 0 from t=1 -> infeasible replan.
  // GLength child forces agent 1 to bounce, creating perpetual swap conflicts
  // on this 2-cell grid (unsolvable).
  CBS cbs(2, 1, {},
          {{Coordinate{0, 0}, Coordinate{1, 0}},
           {Coordinate{1, 0}, Coordinate{1, 0}}});
  const auto r = cbs.Solve(200);
  EXPECT_FALSE(r.solved);
}

// ---- short-path parked agent in AgentsToReplan ----

TEST(CBS, ShortPathParkedAgentInReplan) {
  // 3x2 grid. Three agents sharing goal (2,0):
  //   Agent 0: (2,0) → (2,0)  trivial, cost 0.
  //   Agent 1: (1,0) → (2,0)  cost 1.
  //   Agent 2: (2,1) → (2,0)  cost 1.
  // At root, target conflict between agent 0 (shortest) and another at t=1.
  // GLength child: agent 0 replanned with cost >= 2.
  // At depth 2: target conflict at t=2 generates LeqLength{x, (2,0), 2}.
  // The OTHER cost-1 agent has path.size()-1 = 1 < leq->t = 2 and is parked
  // at (2,0), triggering the short-path branch in AgentsToReplan.
  // Problem is unsolvable (3 agents cannot all park at one cell).
  CBS cbs(3, 2, {},
          {{Coordinate{2, 0}, Coordinate{2, 0}},
           {Coordinate{1, 0}, Coordinate{2, 0}},
           {Coordinate{2, 1}, Coordinate{2, 0}}});
  const auto r = cbs.Solve(500);
  EXPECT_FALSE(r.solved);
}

// ---- result statistics ----

TEST(CBS, DisjointPathsExpandOneNode) {
  CBS cbs(3, 3, {},
          {{Coordinate{0, 0}, Coordinate{2, 0}},
           {Coordinate{0, 2}, Coordinate{2, 2}}});
  const auto r = cbs.Solve();
  ASSERT_TRUE(r.solved);
  EXPECT_EQ(r.expanded, 1);
  EXPECT_EQ(r.generated, 1);
}

// ---- bottleneck ----

TEST(CBS, BottleneckRequiresWaiting) {
  // 3x3 grid with obstacles at (1,0) and (1,2).
  //   . X .
  //   . . .
  //   . X .
  // Both agents must pass through the bottleneck (0,1)->(1,1)->(2,1).
  // One must wait: SOC = 4 + 5 = 9.
  CBS cbs(3, 3, {Coordinate{1, 0}, Coordinate{1, 2}},
          {{Coordinate{0, 0}, Coordinate{2, 0}},
           {Coordinate{0, 2}, Coordinate{2, 2}}});
  const auto r = cbs.Solve();
  ExpectOptimal(r, 9);
}

// ---- non-cardinal conflict ----

TEST(CBS, NonCardinalConflictOnWideGrid) {
  // 5x5 open grid. Two agents with many optimal paths.
  // Agent 0: (0,0) -> (2,2), cost 4. Multiple paths exist.
  // Agent 1: (2,0) -> (0,2), cost 4. Multiple paths exist.
  // The conflict at the center is non-cardinal because both MDDs
  // have width > 1 at the conflict timestep.
  CBS cbs(5, 5, {},
          {{Coordinate{0, 0}, Coordinate{2, 2}},
           {Coordinate{2, 0}, Coordinate{0, 2}}});
  const auto r = cbs.Solve();
  ASSERT_TRUE(r.solved);
  EXPECT_TRUE(IsConflictFree(r.solution));
}

// ---- crossing agents ----

TEST(CBS, CrossingAgents) {
  // 5x5 grid. Two agents cross at the center.
  // Agent 0: (0,2) -> (4,2) cost 4.
  // Agent 1: (2,0) -> (2,4) cost 4.
  // They conflict at (2,2) at t=2. One waits, SOC = 4 + 5 = 9.
  CBS cbs(5, 5, {},
          {{Coordinate{0, 2}, Coordinate{4, 2}},
           {Coordinate{2, 0}, Coordinate{2, 4}}});
  const auto r = cbs.Solve();
  ASSERT_TRUE(r.solved);
  EXPECT_TRUE(IsConflictFree(r.solution));
  EXPECT_GE(r.cost, 9);
}
