/*
 * dijikstra_test.cpp
 *
 * Created on: Mar 11, 2022 21:17
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/dijikstra/dijikstra.hpp"

#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace pllee4;
using namespace graph;

TEST(Dijikstra, InvalidConstructor) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0};
  try {
    Dijikstra dijikstra(motion_constraint);
  } catch (const std::invalid_argument& e) {
    EXPECT_EQ(
        e.what(),
        std::string("Incompatible size of dx and dy for motion constraint"));
  }
}

TEST(Dijikstra, ValidConstructor) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};
  EXPECT_NO_THROW(Dijikstra dijikstra{motion_constraint});
  EXPECT_NO_THROW(Dijikstra dijikstra(motion_constraint));
}

TEST(Dijikstra, InvalidSetOccupancyGrid) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};

  Dijikstra dijikstra{motion_constraint};
  EXPECT_FALSE(dijikstra.SetOccupancyGrid({{1, 6}}, 2, 6));
}

TEST(Dijikstra, ValidSetOccupancyGrid) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};

  Dijikstra dijikstra{motion_constraint};
  EXPECT_TRUE(dijikstra.SetOccupancyGrid({{1, 6}}, 2, 7));
}

TEST(Dijikstra, FailedToFindPath) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};

  Dijikstra dijikstra{motion_constraint};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  EXPECT_FALSE(dijikstra.FindPath({0, 0}, {4, 4}));
  EXPECT_FALSE(dijikstra.FindPath({4, 4}, {2, 2}));
}

TEST(Dijikstra, FindPath) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};

  Dijikstra dijikstra{motion_constraint};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  EXPECT_TRUE(dijikstra.FindPath({0, 2}, {2, 2}));
}

TEST(Dijikstra, GetPath) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};

  Dijikstra dijikstra{motion_constraint};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  dijikstra.FindPath({0, 2}, {2, 2});
  auto path = dijikstra.GetPath({2, 2});

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Dijikstra, GetPathWithRevisit) {
  struct MotionConstraint motion_constraint;
  motion_constraint.dx = {1, 0, -1, 0};
  motion_constraint.dy = {0, 1, 0, -1};

  Dijikstra dijikstra{motion_constraint};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  dijikstra.SetOccupancyGrid({{1, 2}}, 3, 4);
  dijikstra.FindPath({0, 2}, {2, 0});
  auto path = dijikstra.GetPath({2, 0});

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {1, 1}, {1, 0}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}