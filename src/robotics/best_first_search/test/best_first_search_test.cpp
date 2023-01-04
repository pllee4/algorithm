/* 
 * best_first_search_test.cpp
 * 
 * Created on: Jan 04, 2023 21:26
 * Description: 
 * 
 * Copyright (c) 2023 Pin Loon Lee (pllee4)
 */ 

#include "algorithm/robotics/best_first_search/best_first_search.hpp"

#include "gtest/gtest.h"

using namespace pllee4::graph;

static constexpr uint8_t weight = 10;

TEST(BestFirstSearch, InvalidSetOccupiedGrid) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};
  EXPECT_FALSE(best_first_search.SetOccupiedGrid({{1, 6}}));
  best_first_search.SetMapStorageSize(3, 3);
  EXPECT_FALSE(best_first_search.SetOccupiedGrid({{1, 6}}));
}

TEST(BestFirstSearch, ValidSetOccupanciedGrid) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};
  best_first_search.SetMapStorageSize(2, 7);
  EXPECT_TRUE(best_first_search.SetOccupiedGrid({{1, 6}}));
}

TEST(BestFirstSearch, FailedToFindPath) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  best_first_search.SetMapStorageSize(3, 3);
  best_first_search.SetOccupiedGrid({{1, 2}, {1, 1}});
  best_first_search.SetStartAndDestination({0, 0}, {4, 4});
  EXPECT_FALSE(best_first_search.FindPath());
  EXPECT_FALSE(best_first_search.GetPath().has_value());
  best_first_search.SetStartAndDestination({4, 4}, {2, 2});
  EXPECT_FALSE(best_first_search.FindPath());
  EXPECT_FALSE(best_first_search.GetPath().has_value());
}

TEST(BestFirstSearch, FindPath) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */
  best_first_search.SetMapStorageSize(3, 3);
  best_first_search.SetOccupiedGrid({{1, 2}, {1, 1}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_TRUE(best_first_search.FindPath());
}

TEST(BestFirstSearch, GetPath) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  best_first_search.SetMapStorageSize(3, 3);
  best_first_search.SetOccupiedGrid({{1, 2}, {1, 1}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 2});
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
  const auto path = best_first_search.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(BestFirstSearch, GetPathWithSettingOfSameStartAndDestination) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  best_first_search.SetMapStorageSize(3, 3);
  best_first_search.SetOccupiedGrid({{1, 2}, {1, 1}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 2});

  // no clearing of occupied grid
  best_first_search.SetStartAndDestination({0, 2}, {2, 2});
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());

  const auto path = best_first_search.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0},
                                      {2, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));

  best_first_search.SetStartAndDestination({0, 2}, {2, 2});
  // same start and destination, can get back previous path without finding
  EXPECT_TRUE(best_first_search.GetPath().has_value());

  best_first_search.SetStartAndDestination({0, 1}, {2, 2});
  // different start and destination, can't get path without finding
  EXPECT_FALSE(best_first_search.GetPath().has_value());

  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
}

TEST(BestFirstSearch, GetPathAfterReset) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  best_first_search.SetMapStorageSize(3, 3);
  best_first_search.SetOccupiedGrid({{1, 2}, {1, 1}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 2});

  // internally clear path and occupied
  best_first_search.Reset();
  best_first_search.FindPath();
  EXPECT_FALSE(best_first_search.GetPath().has_value());

  best_first_search.SetStartAndDestination({0, 2}, {2, 2});
  EXPECT_FALSE(best_first_search.GetPath().has_value());
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
  const auto path = best_first_search.GetPath().value();

  // found path without occupied_grid
  std::vector<Coordinate> expected = {{0, 2}, {1, 2}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(BestFirstSearch, GetPath8Dir) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |s  | x | e |
   *  |   | x |   |
   *  |   |   |   |
   */

  best_first_search.SetMapStorageSize(3, 3);
  best_first_search.SetOccupiedGrid({{1, 2}, {1, 1}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 2});
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
  const auto path = best_first_search.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0}, {2, 1}, {2, 2}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(BestFirstSearch, GetPathWithRevisit) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  best_first_search.SetMapStorageSize(3, 4);
  best_first_search.SetOccupiedGrid({{1, 2}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 0});
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
  const auto path = best_first_search.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {0, 1}, {0, 0}, {1, 0}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(BestFirstSearch, GetPath8DirWithRevisit) {
  BestFirstSearch best_first_search{MotionConstraintType::CARDINAL_ORDINAL_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  best_first_search.SetMapStorageSize(3, 4);
  best_first_search.SetOccupiedGrid({{1, 2}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 0});
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
  const auto path = best_first_search.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}

TEST(BestFirstSearch, GetPath8DirWithRevisitWithAnyMotion) {
  BestFirstSearch best_first_search{MotionConstraintType::ANY_MOTION};

  /**
   * s = start, e = end, x = occupied
   *  |   |   |   |
   *  | s | x |   |
   *  |   |   |   |
   *  |   |   | e |
   */

  best_first_search.SetMapStorageSize(3, 4);
  best_first_search.SetOccupiedGrid({{1, 2}});
  best_first_search.SetStartAndDestination({0, 2}, {2, 0});
  best_first_search.FindPath();
  EXPECT_TRUE(best_first_search.GetPath().has_value());
  const auto path = best_first_search.GetPath().value();

  std::vector<Coordinate> expected = {{0, 2}, {1, 1}, {2, 0}};
  EXPECT_TRUE(std::equal(std::begin(path), std::end(path), std::begin(expected),
                         std::end(expected)));
}