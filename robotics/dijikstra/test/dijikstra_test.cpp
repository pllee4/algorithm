/*
 * dijikstra_test.cpp
 *
 * Created on: Mar 11, 2022 21:17
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "dijikstra/dijikstra.hpp"

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
  } catch (const std::exception& e) {
    EXPECT_EQ(
        e.what(),
        std::string("Incompatible size of dx and dy for motion constraint"));
  }
}