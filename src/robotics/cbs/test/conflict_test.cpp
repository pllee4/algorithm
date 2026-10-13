/*
 * conflict_test.cpp
 *
 * Created on: Oct 03, 2026 20:36
 * Description:
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/cbs/conflict.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

namespace {
/** @brief Two agents walking into the same cell from opposite sides. */
const Solution kHeadOn = {{{0, 0}, {1, 0}, {2, 0}},
                          {{2, 0}, {1, 0}, {0, 0}}};

/** @brief Two agents trading places across one edge. */
const Solution kSwap = {{{0, 0}, {1, 0}}, {{1, 0}, {0, 0}}};
}  // namespace

// --- PositionAt ---

TEST(Conflict, PositionAtWalksThePath) {
  const Path path = {{0, 0}, {1, 0}, {1, 1}};
  EXPECT_TRUE(PositionAt(path, 0) == Coordinate({0, 0}));
  EXPECT_TRUE(PositionAt(path, 1) == Coordinate({1, 0}));
  EXPECT_TRUE(PositionAt(path, 2) == Coordinate({1, 1}));
}

TEST(Conflict, PositionAtParksOnTheGoalPastTheEnd) {
  const Path path = {{0, 0}, {1, 0}};
  EXPECT_TRUE(PositionAt(path, 2) == Coordinate({1, 0}));
  EXPECT_TRUE(PositionAt(path, 99) == Coordinate({1, 0}));
}

TEST(Conflict, PositionAtOfAnEmptyPathIsNowhere) {
  EXPECT_TRUE(PositionAt({}, 0) == Coordinate({-1, -1}));
}

// --- Standard vertex conflict ---

TEST(Conflict, DetectsVertexConflict) {
  const auto conflict = FindFirstConflict(kHeadOn);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(conflict->type, Conflict::Type::kStandard);
  EXPECT_EQ(conflict->agent1, 0);
  EXPECT_EQ(conflict->agent2, 1);
  EXPECT_EQ(conflict->timestep, 1);
}

TEST(Conflict, VertexConflictConstrainsBothAgents) {
  const auto conflict = FindFirstConflict(kHeadOn);
  ASSERT_TRUE(conflict.has_value());
  ASSERT_EQ(conflict->constraints1.size(), 1u);
  ASSERT_EQ(conflict->constraints2.size(), 1u);

  const auto *vc1 = std::get_if<VertexConstraint>(&conflict->constraints1[0]);
  const auto *vc2 = std::get_if<VertexConstraint>(&conflict->constraints2[0]);
  ASSERT_NE(vc1, nullptr);
  ASSERT_NE(vc2, nullptr);
  EXPECT_EQ(vc1->agent, 0);
  EXPECT_EQ(vc2->agent, 1);
  EXPECT_TRUE(vc1->cell == Coordinate({1, 0}));
  EXPECT_TRUE(vc2->cell == Coordinate({1, 0}));
  EXPECT_EQ(vc1->t, 1);
  EXPECT_EQ(vc2->t, 1);
}

// --- Standard edge conflict ---

TEST(Conflict, DetectsEdgeConflict) {
  const auto conflict = FindFirstConflict(kSwap);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(conflict->type, Conflict::Type::kStandard);
  EXPECT_EQ(conflict->timestep, 1);
}

TEST(Conflict, EdgeConflictConstrainsEachDirection) {
  const auto conflict = FindFirstConflict(kSwap);
  ASSERT_TRUE(conflict.has_value());
  ASSERT_EQ(conflict->constraints1.size(), 1u);
  ASSERT_EQ(conflict->constraints2.size(), 1u);

  const auto *ec1 = std::get_if<EdgeConstraint>(&conflict->constraints1[0]);
  const auto *ec2 = std::get_if<EdgeConstraint>(&conflict->constraints2[0]);
  ASSERT_NE(ec1, nullptr);
  ASSERT_NE(ec2, nullptr);
  EXPECT_EQ(ec1->agent, 0);
  EXPECT_TRUE(ec1->from == Coordinate({0, 0}));
  EXPECT_TRUE(ec1->to == Coordinate({1, 0}));
  EXPECT_EQ(ec2->agent, 1);
  EXPECT_TRUE(ec2->from == Coordinate({1, 0}));
  EXPECT_TRUE(ec2->to == Coordinate({0, 0}));
}

// --- Target conflict ---

TEST(Conflict, DetectsTargetConflictWhenParkedAgentIsBlocking) {
  // Agent 0 finishes at (1,0) at t=1 and parks. Agent 1 crosses at t=2.
  const Solution solution = {{{0, 0}, {1, 0}}, {{3, 0}, {2, 0}, {1, 0}}};
  const auto conflict = FindFirstConflict(solution);
  ASSERT_TRUE(conflict.has_value());

  EXPECT_EQ(conflict->type, Conflict::Type::kTarget);
  EXPECT_EQ(conflict->agent1, 0);  // parked
  EXPECT_EQ(conflict->agent2, 1);  // passer-by
  EXPECT_EQ(conflict->timestep, 2);
}

TEST(Conflict, TargetConflictConstrainsParkedAgentInBothChildren) {
  const Solution solution = {{{0, 0}, {1, 0}}, {{3, 0}, {2, 0}, {1, 0}}};
  const auto conflict = FindFirstConflict(solution);
  ASSERT_TRUE(conflict.has_value());
  ASSERT_EQ(conflict->constraints1.size(), 1u);
  ASSERT_EQ(conflict->constraints2.size(), 1u);

  const auto *leq =
      std::get_if<LeqLengthConstraint>(&conflict->constraints1[0]);
  const auto *gl =
      std::get_if<GLengthConstraint>(&conflict->constraints2[0]);
  ASSERT_NE(leq, nullptr);
  ASSERT_NE(gl, nullptr);
  EXPECT_EQ(leq->agent, 0);
  EXPECT_TRUE(leq->goal == Coordinate({1, 0}));
  EXPECT_EQ(leq->t, 2);
  EXPECT_EQ(gl->agent, 0);
  EXPECT_TRUE(gl->goal == Coordinate({1, 0}));
  EXPECT_EQ(gl->t, 2);
}

TEST(Conflict, TargetConflictAtExactArrivalTimestep) {
  // Agent 0 arrives at (2,0) at t=2 and parks. Agent 1 passes through
  // (2,0) at t=2 but continues (longer path, so not yet parked).
  const Solution solution = {{{0, 0}, {1, 0}, {2, 0}},
                             {{2, 1}, {2, 1}, {2, 0}, {2, 1}}};
  const auto conflict = FindFirstConflict(solution);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(conflict->type, Conflict::Type::kTarget);
  EXPECT_EQ(conflict->agent1, 0);  // parked (arrived exactly at t=2)
  EXPECT_EQ(conflict->agent2, 1);  // passer-by (still moving)
  EXPECT_EQ(conflict->timestep, 2);
}

// --- Conflict selection ---

TEST(Conflict, CardinalBeatsSemiAndNon) {
  auto cardinal = Conflict::Vertex(0, 1, {1, 0}, 5);
  cardinal.priority = Conflict::Priority::kCardinal;
  auto semi = Conflict::Vertex(0, 1, {1, 0}, 3);
  semi.priority = Conflict::Priority::kSemi;
  auto non = Conflict::Vertex(0, 1, {1, 0}, 1);
  non.priority = Conflict::Priority::kNon;

  // operator< returns true when rhs is better.
  EXPECT_TRUE(non < cardinal);
  EXPECT_TRUE(semi < cardinal);
  EXPECT_TRUE(non < semi);
  EXPECT_FALSE(cardinal < non);
  EXPECT_FALSE(cardinal < semi);
}

TEST(Conflict, TargetBeatsStandardAtSamePriority) {
  auto target = Conflict::Target(0, 1, {1, 0}, 5);
  auto standard = Conflict::Vertex(0, 1, {1, 0}, 3);
  // Both UNKNOWN priority, but target type beats standard.
  EXPECT_TRUE(standard < target);
  EXPECT_FALSE(target < standard);
}

TEST(Conflict, EqualConflictsCompareAsFalse) {
  auto a = Conflict::Vertex(0, 1, {1, 0}, 5);
  auto b = Conflict::Vertex(2, 3, {2, 0}, 7);
  // Same priority (UNKNOWN) and type (STANDARD) -> false both ways.
  EXPECT_FALSE(a < b);
  EXPECT_FALSE(b < a);
}

// --- Non-conflict cases ---

TEST(Conflict, FollowingIntoAVacatedCellIsAllowed) {
  const Solution following = {{{0, 0}, {1, 0}, {2, 0}},
                              {{1, 0}, {2, 0}, {3, 0}}};
  EXPECT_FALSE(FindFirstConflict(following).has_value());
  EXPECT_TRUE(IsConflictFree(following));
}

TEST(Conflict, ParallelPathsNeverCollide) {
  const Solution parallel = {{{0, 0}, {1, 0}, {2, 0}},
                             {{0, 2}, {1, 2}, {2, 2}}};
  EXPECT_TRUE(IsConflictFree(parallel));
}

TEST(Conflict, EmptyPathNeverCollides) {
  const Solution solution = {{{0, 0}, {1, 0}}, {}};
  EXPECT_TRUE(IsConflictFree(solution));
  EXPECT_FALSE(FindFirstConflict(solution[0], solution[1], 0, 1).has_value());
}

// --- Ordering ---

TEST(Conflict, FirstConflictIsTheEarliestInTime) {
  // Pair (0,1) collides at t=3, pair (0,2) at t=1.
  // Agent 2 has a shorter path, so the t=1 conflict is a target conflict
  // with agent 2 (parked) as agent1 and agent 0 as agent2.
  const Solution solution = {{{0, 0}, {1, 0}, {2, 0}, {3, 0}},
                             {{3, 1}, {3, 0}},
                             {{1, 1}, {1, 0}}};
  const auto conflict = FindFirstConflict(solution);
  ASSERT_TRUE(conflict.has_value());
  EXPECT_EQ(conflict->timestep, 1);
  EXPECT_EQ(conflict->type, Conflict::Type::kTarget);
  EXPECT_EQ(conflict->agent1, 2);  // parked (shorter path)
  EXPECT_EQ(conflict->agent2, 0);  // passer-by
}

// --- FindAllConflicts ---

TEST(Conflict, FindAllConflictsCountsEveryTimestep) {
  const Solution solution = {{{0, 0}, {0, 0}, {0, 0}},
                             {{0, 0}, {0, 0}, {0, 0}}};
  EXPECT_EQ(FindAllConflicts(solution).size(), 3u);
}

TEST(Conflict, FindAllConflictsIsEmptyForAValidSolution) {
  const Solution parallel = {{{0, 0}, {1, 0}}, {{0, 2}, {1, 2}}};
  EXPECT_TRUE(FindAllConflicts(parallel).empty());
}

TEST(Conflict, FindAllConflictsDetectsTargetConflicts) {
  // Agent 0 parks at (1,0). Agent 1 crosses through at t=2 and t=3.
  const Solution solution = {{{0, 0}, {1, 0}},
                             {{3, 0}, {2, 0}, {1, 0}, {1, 0}}};
  const auto conflicts = FindAllConflicts(solution);
  ASSERT_GE(conflicts.size(), 1u);
  // The conflict at the parked goal should be a target conflict.
  bool found_target = false;
  for (const auto &c : conflicts) {
    if (c.type == Conflict::Type::kTarget) found_target = true;
  }
  EXPECT_TRUE(found_target);
}

// --- Measures ---

TEST(Conflict, SumOfCostsAddsUpArrivalTimes) {
  const Solution solution = {{{0, 0}, {1, 0}, {2, 0}},  // cost 2
                             {{0, 2}, {1, 2}}};         // cost 1
  EXPECT_EQ(GetSumOfCosts(solution), 3);
  EXPECT_EQ(GetMakespan(solution), 2);
}

TEST(Conflict, MeasuresIgnoreUnplannedAgents) {
  const Solution solution = {{{0, 0}, {1, 0}}, {}};
  EXPECT_EQ(GetSumOfCosts(solution), 1);
  EXPECT_EQ(GetMakespan(solution), 1);
  EXPECT_EQ(GetSumOfCosts({}), 0);
  EXPECT_EQ(GetMakespan({}), 0);
}
