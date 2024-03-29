/*
 * dijikstra_test.cpp
 *
 * Created on: Mar 11, 2022 21:17
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/robotics/dijikstra/dijikstra.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

TEST(Dijikstra, InvalidSetOccupiedGrid) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};
  EXPECT_FALSE(dijikstra.SetOccupiedGrid({{1, 6}}));
  dijikstra.SetMapStorageSize(2, 6);
  EXPECT_FALSE(dijikstra.SetOccupiedGrid({{1, 6}}));
}

TEST(Dijikstra, ValidSetOccupanciedGrid) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};
  dijikstra.SetMapStorageSize(2, 7);
  EXPECT_TRUE(dijikstra.SetOccupiedGrid({{1, 6}}));
}

TEST(Dijikstra, FailedToFindPath) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetMapStorageSize(3, 3);
  dijikstra.SetOccupiedGrid({{1, 2}, {1, 1}});
  dijikstra.SetStartAndDestination({0, 0}, {4, 4});
  EXPECT_FALSE(dijikstra.FindPath());
  EXPECT_FALSE(dijikstra.GetPath().has_value());
  dijikstra.SetStartAndDestination({4, 4}, {2, 2});
  EXPECT_FALSE(dijikstra.FindPath());
  EXPECT_FALSE(dijikstra.GetPath().has_value());
}

TEST(Dijikstra, FindPath) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetMapStorageSize(3, 3);
  dijikstra.SetOccupiedGrid({{1, 2}, {1, 1}});
  dijikstra.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_TRUE(dijikstra.FindPath());
}

TEST(Dijikstra, GetPath) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetMapStorageSize(3, 3);
  dijikstra.SetOccupiedGrid({{1, 2}, {1, 1}});
  dijikstra.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_TRUE(dijikstra.FindPath());
  EXPECT_TRUE(dijikstra.GetPath().has_value());
  const auto path = dijikstra.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Dijikstra, GetPathWithSettingOfSameStartAndDestination) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetMapStorageSize(3, 3);
  dijikstra.SetOccupiedGrid({{1, 2}, {1, 1}});
  dijikstra.SetStartAndDestination({0, 2}, {2, 2});

  // no clearing of occupied grid
  dijikstra.SetStartAndDestination({0, 2}, {2, 2});
  dijikstra.FindPath();
  EXPECT_TRUE(dijikstra.GetPath().has_value());

  const auto path = dijikstra.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));

  dijikstra.SetStartAndDestination({0, 2}, {2, 2});
  // same start and destination, can get back previous path without finding
  EXPECT_TRUE(dijikstra.GetPath().has_value());

  dijikstra.SetStartAndDestination({0, 1}, {2, 2});
  // different start and destination, can't get path without finding
  EXPECT_FALSE(dijikstra.GetPath().has_value());

  dijikstra.FindPath();
  EXPECT_TRUE(dijikstra.GetPath().has_value());
}

TEST(Dijikstra, GetPathAfterReset) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  dijikstra.SetMapStorageSize(3, 3);
  dijikstra.SetOccupiedGrid({{1, 2}, {1, 1}});
  dijikstra.SetStartAndDestination({0, 2}, {2, 2});

  // internally clear path and occupied
  dijikstra.Reset();
  dijikstra.FindPath();
  EXPECT_FALSE(dijikstra.GetPath().has_value());

  dijikstra.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_FALSE(dijikstra.GetPath().has_value());
  dijikstra.FindPath();
  EXPECT_TRUE(dijikstra.GetPath().has_value());
  const auto path = dijikstra.GetPath().value();

  // found path without occupied_grid
  std::vector<Coordinate> expected = {{0, 2}, {1, 2}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(Dijikstra, GetPathWithRevisit) {
  Dijikstra dijikstra{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  dijikstra.SetMapStorageSize(3, 4);
  dijikstra.SetOccupiedGrid({{1, 2}});
  dijikstra.SetStartAndDestination({0, 2}, {2, 0});
  dijikstra.FindPath();
  EXPECT_TRUE(dijikstra.GetPath().has_value());
  const auto path = dijikstra.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {1, 1}, {1, 0}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}