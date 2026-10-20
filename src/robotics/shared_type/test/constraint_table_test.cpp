/*
 * constraint_table_test.cpp
 *
 * Created on: Oct 20, 2026 21:00
 * Description:
 *
 * Copyright (c) 2026 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/shared_type/constraint_table.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

// ---- vertex constraints ----

TEST(ConstraintTable, VertexConstraintBlocksCell) {
  auto constraints =
      std::vector<Constraint>{VertexConstraint{-1, Coordinate{1, 0}, 3}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_TRUE(table.IsBlocked(Coordinate{1, 0}, 3));
  EXPECT_FALSE(table.IsBlocked(Coordinate{1, 0}, 2));
  EXPECT_FALSE(table.IsBlocked(Coordinate{0, 0}, 3));
}

TEST(ConstraintTable, VertexConstraintOnGoalUpdatesHoldingTime) {
  auto constraints =
      std::vector<Constraint>{VertexConstraint{-1, Coordinate{2, 0}, 5}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_EQ(table.GetHoldingTime(), 6);
}

// ---- edge constraints ----

TEST(ConstraintTable, EdgeConstraintBlocksMove) {
  auto constraints = std::vector<Constraint>{
      EdgeConstraint{-1, Coordinate{0, 0}, Coordinate{1, 0}, 2}};
  ConstraintTable table(constraints, Coordinate{3, 0});
  EXPECT_TRUE(table.IsBlocked(Coordinate{0, 0}, Coordinate{1, 0}, 2));
  EXPECT_FALSE(table.IsBlocked(Coordinate{1, 0}, Coordinate{0, 0}, 2));
  EXPECT_FALSE(table.IsBlocked(Coordinate{0, 0}, Coordinate{1, 0}, 3));
}

// ---- LeqLengthConstraint for the owning agent ----

TEST(ConstraintTable, LeqLengthForOwnerSetsMaxLength) {
  auto constraints =
      std::vector<Constraint>{LeqLengthConstraint{0, Coordinate{2, 0}, 5}};
  ConstraintTable table(constraints, Coordinate{2, 0}, 0);
  EXPECT_EQ(table.MaxLength(), 5);
}

// ---- LeqLengthConstraint for another agent creates range block ----

TEST(ConstraintTable, LeqLengthForOtherAgentCreatesRangeBlock) {
  auto constraints =
      std::vector<Constraint>{LeqLengthConstraint{1, Coordinate{2, 0}, 5}};
  ConstraintTable table(constraints, Coordinate{3, 0}, 0);
  EXPECT_TRUE(table.IsBlocked(Coordinate{2, 0}, 5));
  EXPECT_TRUE(table.IsBlocked(Coordinate{2, 0}, 10));
  EXPECT_FALSE(table.IsBlocked(Coordinate{2, 0}, 4));
}

// ---- LeqLengthConstraint with negative agents (fallback) ----

TEST(ConstraintTable, LeqLengthWithNegativeAgentSetsMaxLength) {
  auto constraints =
      std::vector<Constraint>{LeqLengthConstraint{-1, Coordinate{2, 0}, 4}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_EQ(table.MaxLength(), 4);
}

// ---- GLengthConstraint ----

TEST(ConstraintTable, GLengthSetsMinLength) {
  auto constraints =
      std::vector<Constraint>{GLengthConstraint{-1, Coordinate{2, 0}, 3}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_EQ(table.MinLength(), 4);
}

// ---- GetHoldingTime considers min_length ----

TEST(ConstraintTable, HoldingTimeRespectsMinLength) {
  auto constraints =
      std::vector<Constraint>{GLengthConstraint{-1, Coordinate{2, 0}, 5}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_GE(table.GetHoldingTime(), 6);
}

// ---- GetHoldingTime with range entries on goal ----

TEST(ConstraintTable, HoldingTimeConsidersRangeBlockOnGoal) {
  auto constraints =
      std::vector<Constraint>{LeqLengthConstraint{1, Coordinate{2, 0}, 5}};
  ConstraintTable table(constraints, Coordinate{2, 0}, 0);
  EXPECT_GE(table.GetHoldingTime(), ConstraintTable::kMaxTimestep);
}

// ---- GetLatestConstraint ----

TEST(ConstraintTable, GetLatestConstraintReturnsMaxTimestep) {
  auto constraints =
      std::vector<Constraint>{VertexConstraint{-1, Coordinate{0, 0}, 3},
                              VertexConstraint{-1, Coordinate{1, 0}, 7}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_EQ(table.GetLatestConstraint(), 7);
}

TEST(ConstraintTable, GetLatestConstraintRespectsMinLength) {
  auto constraints =
      std::vector<Constraint>{GLengthConstraint{-1, Coordinate{2, 0}, 10}};
  ConstraintTable table(constraints, Coordinate{2, 0});
  EXPECT_GE(table.GetLatestConstraint(), 11);
}

TEST(ConstraintTable, GetLatestConstraintRespectsMaxLength) {
  auto constraints =
      std::vector<Constraint>{LeqLengthConstraint{0, Coordinate{2, 0}, 5}};
  ConstraintTable table(constraints, Coordinate{2, 0}, 0);
  EXPECT_GE(table.GetLatestConstraint(), 5);
}

// ---- agent filtering ----

TEST(ConstraintTable, ConstraintForDifferentAgentIsIgnored) {
  auto constraints =
      std::vector<Constraint>{VertexConstraint{1, Coordinate{1, 0}, 3}};
  ConstraintTable table(constraints, Coordinate{2, 0}, 0);
  EXPECT_FALSE(table.IsBlocked(Coordinate{1, 0}, 3));
}

TEST(ConstraintTable, ConstraintForSameAgentApplies) {
  auto constraints =
      std::vector<Constraint>{VertexConstraint{0, Coordinate{1, 0}, 3}};
  ConstraintTable table(constraints, Coordinate{2, 0}, 0);
  EXPECT_TRUE(table.IsBlocked(Coordinate{1, 0}, 3));
}

// ---- empty constraints ----

TEST(ConstraintTable, EmptyConstraints) {
  ConstraintTable table({}, Coordinate{0, 0});
  EXPECT_FALSE(table.IsBlocked(Coordinate{0, 0}, 0));
  EXPECT_EQ(table.GetHoldingTime(), 0);
  EXPECT_EQ(table.GetLatestConstraint(), 0);
  EXPECT_EQ(table.MinLength(), 0);
  EXPECT_EQ(table.MaxLength(), ConstraintTable::kMaxTimestep);
}
