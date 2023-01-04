/* 
 * weighted_astar_test.cpp
 * 
 * Created on: Jan 02, 2023 11:49
 * Description: 
 * 
 * Copyright (c) 2023 Pin Loon Lee (pllee4)
 */ 

#include "algorithm/robotics/astar/weighted_astar.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

static constexpr uint8_t weight = 10;

TEST(WeightedAstar, InvalidSetOccupiedGrid) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};
  EXPECT_FALSE(weighted_astar.SetOccupiedGrid({{1, 6}}));
  weighted_astar.SetMapStorageSize(3, 3);
  EXPECT_FALSE(weighted_astar.SetOccupiedGrid({{1, 6}}));
}

TEST(WeightedAstar, ValidSetOccupanciedGrid) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};
  weighted_astar.SetMapStorageSize(2, 7);
  EXPECT_TRUE(weighted_astar.SetOccupiedGrid({{1, 6}}));
}

TEST(WeightedAstar, FailedToFindPath) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  weighted_astar.SetMapStorageSize(3, 3);
  weighted_astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  weighted_astar.SetStartAndDestination({0, 0}, {4, 4});
  EXPECT_FALSE(weighted_astar.FindPath());
  EXPECT_FALSE(weighted_astar.GetPath().has_value());
  weighted_astar.SetStartAndDestination({4, 4}, {2, 2});
  EXPECT_FALSE(weighted_astar.FindPath());
  EXPECT_FALSE(weighted_astar.GetPath().has_value());
}

TEST(WeightedAstar, FindPath) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */
  weighted_astar.SetMapStorageSize(3, 3);
  weighted_astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_TRUE(weighted_astar.FindPath());
}

TEST(WeightedAstar, GetPath) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  weighted_astar.SetMapStorageSize(3, 3);
  weighted_astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
  const auto path = weighted_astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(WeightedAstar, GetPathWithSettingOfSameStartAndDestination) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  weighted_astar.SetMapStorageSize(3, 3);
  weighted_astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});

  // no clearing of occupied grid
  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());

  const auto path = weighted_astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));

  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});
  // same start and destination, can get back previous path without finding
  EXPECT_TRUE(weighted_astar.GetPath().has_value());

  weighted_astar.SetStartAndDestination({0, 1}, {2, 2});
  // different start and destination, can't get path without finding
  EXPECT_FALSE(weighted_astar.GetPath().has_value());

  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
}

TEST(WeightedAstar, GetPathAfterReset) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  weighted_astar.SetMapStorageSize(3, 3);
  weighted_astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});

  // internally clear path and occupied
  weighted_astar.Reset();
  weighted_astar.FindPath();
  EXPECT_FALSE(weighted_astar.GetPath().has_value());

  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_FALSE(weighted_astar.GetPath().has_value());
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
  const auto path = weighted_astar.GetPath().value();

  // found path without occupied_grid
  std::vector<Coordinate> expected = {{0, 2}, {1, 2}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(WeightedAstar, GetPath8Dir) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  weighted_astar.SetMapStorageSize(3, 3);
  weighted_astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 2});
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
  const auto path = weighted_astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {1, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(WeightedAstar, GetPathWithRevisit) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  weighted_astar.SetMapStorageSize(3, 4);
  weighted_astar.SetOccupiedGrid({{1, 2}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 0});
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
  const auto path = weighted_astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(WeightedAstar, GetPath8DirWithRevisit) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  weighted_astar.SetMapStorageSize(3, 4);
  weighted_astar.SetOccupiedGrid({{1, 2}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 0});
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
  const auto path = weighted_astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(WeightedAstar, GetPath8DirWithRevisitWithAnyMotion) {
  WeightedAstar<weight> weighted_astar{MotionConstraintType::ANY_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  weighted_astar.SetMapStorageSize(3, 4);
  weighted_astar.SetOccupiedGrid({{1, 2}});
  weighted_astar.SetStartAndDestination({0, 2}, {2, 0});
  weighted_astar.FindPath();
  EXPECT_TRUE(weighted_astar.GetPath().has_value());
  const auto path = weighted_astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}