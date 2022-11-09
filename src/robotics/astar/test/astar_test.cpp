/*
 * astar_test.cpp
 *
 * Created on: Mar 20, 2022 20:50
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/astar/astar.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

TEST(Astar, InvalidSetOccupiedGrid) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};
  EXPECT_FALSE(astar.SetOccupiedGrid({{1, 6}}));
  astar.SetMapStorageSize(3, 3);
  EXPECT_FALSE(astar.SetOccupiedGrid({{1, 6}}));
}

TEST(Astar, ValidSetOccupanciedGrid) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};
  astar.SetMapStorageSize(2, 7);
  EXPECT_TRUE(astar.SetOccupiedGrid({{1, 6}}));
}

TEST(Astar, FailedToFindPath) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetMapStorageSize(3, 3);
  astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  astar.SetStartAndDestination({0, 0}, {4, 4});
  EXPECT_FALSE(astar.FindPath());
  EXPECT_FALSE(astar.GetPath().has_value());
  astar.SetStartAndDestination({4, 4}, {2, 2});
  EXPECT_FALSE(astar.FindPath());
  EXPECT_FALSE(astar.GetPath().has_value());
}

TEST(Astar, FindPath) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */
  astar.SetMapStorageSize(3, 3);
  astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  astar.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_TRUE(astar.FindPath());
}

TEST(Astar, GetPath) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetMapStorageSize(3, 3);
  astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  astar.SetStartAndDestination({0, 2}, {2, 2});
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
  const auto path = astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Astar, GetPathWithSettingOfSameStartAndDestination) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetMapStorageSize(3, 3);
  astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  astar.SetStartAndDestination({0, 2}, {2, 2});

  // no clearing of occupied grid
  astar.SetStartAndDestination({0, 2}, {2, 2});
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());

  const auto path = astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));

  astar.SetStartAndDestination({0, 2}, {2, 2});
  // same start and destination, can get back previous path without finding
  EXPECT_TRUE(astar.GetPath().has_value());

  astar.SetStartAndDestination({0, 1}, {2, 2});
  // different start and destination, can't get path without finding
  EXPECT_FALSE(astar.GetPath().has_value());

  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
}

TEST(Astar, GetPathAfterReset) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetMapStorageSize(3, 3);
  astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  astar.SetStartAndDestination({0, 2}, {2, 2});

  // internally clear path and occupied
  astar.Reset();
  astar.FindPath();
  EXPECT_FALSE(astar.GetPath().has_value());

  astar.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_FALSE(astar.GetPath().has_value());
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
  const auto path = astar.GetPath().value();

  // found path without occupied_grid
  std::vector<Coordinate> expected = {{0, 2}, {1, 2}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Astar, GetPath8Dir) {
  Astar astar{MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  astar.SetMapStorageSize(3, 3);
  astar.SetOccupiedGrid({{1, 2}, {1, 1}});
  astar.SetStartAndDestination({0, 2}, {2, 2});
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
  const auto path = astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {1, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Astar, GetPathWithRevisit) {
  Astar astar{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  astar.SetMapStorageSize(3, 4);
  astar.SetOccupiedGrid({{1, 2}});
  astar.SetStartAndDestination({0, 2}, {2, 0});
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
  const auto path = astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Astar, GetPath8DirWithRevisit) {
  Astar astar{MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  astar.SetMapStorageSize(3, 4);
  astar.SetOccupiedGrid({{1, 2}});
  astar.SetStartAndDestination({0, 2}, {2, 0});
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
  const auto path = astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Astar, GetPath8DirWithRevisitWithAnyMotion) {
  Astar astar{MotionConstraintType::ANY_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  astar.SetMapStorageSize(3, 4);
  astar.SetOccupiedGrid({{1, 2}});
  astar.SetStartAndDestination({0, 2}, {2, 0});
  astar.FindPath();
  EXPECT_TRUE(astar.GetPath().has_value());
  const auto path = astar.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}