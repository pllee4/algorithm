/*
 * astar_test.cpp
 *
 * Created on: Mar 20, 2022 20:50
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/astar/astar.hpp"

#include "gmock/gmock.h"
#include "gtest/gtest.h"

using namespace pllee4;
using namespace graph;

TEST(AStar, InvalidSetOccupancyGrid) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_MOTION};
  EXPECT_FALSE(astar.SetOccupancyGrid({{1, 6}}, 2, 6));
}

TEST(AStar, ValidSetOccupancyGrid) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_MOTION};
  EXPECT_TRUE(astar.SetOccupancyGrid({{1, 6}}, 2, 7));
}

TEST(AStar, FailedToFindPath) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  EXPECT_FALSE(astar.FindPath({0, 0}, {4, 4}));
  EXPECT_FALSE(astar.FindPath({4, 4}, {2, 2}));
}

TEST(AStar, FindPath) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  EXPECT_TRUE(astar.FindPath({0, 2}, {2, 2}));
}

TEST(AStar, GetPath) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  astar.FindPath({0, 2}, {2, 2});
  auto path = astar.GetPath({2, 2});

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(AStar, GetPath8Dir) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetOccupancyGrid({{1, 2}, {1, 1}}, 3, 3);
  astar.FindPath({0, 2}, {2, 2});
  auto path = astar.GetPath({2, 2});

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {1, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(AStar, GetPathWithRevisit) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  astar.SetOccupancyGrid({{1, 2}}, 3, 4);
  astar.FindPath({0, 2}, {2, 0});
  auto path = astar.GetPath({2, 0});

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(AStar, GetPath8DirWithRevisit) {
  AStar astar{AStar::MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  astar.SetOccupancyGrid({{1, 2}}, 3, 4);
  astar.FindPath({0, 2}, {2, 0});
  auto path = astar.GetPath({2, 0});

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(AStar, GetPath8DirWithRevisitWithAnyMotion) {
  AStar astar{AStar::MotionConstraintType::ANY_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  astar.SetOccupancyGrid({{1, 2}}, 3, 4);
  astar.FindPath({0, 2}, {2, 0});
  auto path = astar.GetPath({2, 0});

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}