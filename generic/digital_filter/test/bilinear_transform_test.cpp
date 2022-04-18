/*
 * bilinear_transform_test.cpp
 *
 * Created on: Apr 18, 2022 20:44
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "digital_filter/details/bilinear_transform.hpp"

#include "gtest/gtest.h"

using namespace pllee4::generic;

TEST(BilinearTransform, DivisionByZero) {
  float s_pole = 2.0;
  float z_pole = 0.0;
  EXPECT_THROW(BilinearTransform<float>::SToZ(1, s_pole, z_pole),
               std::invalid_argument);
  z_pole = -1.0;
  s_pole = 0.0;
  EXPECT_THROW(BilinearTransform<float>::ZToS(1, z_pole, s_pole),
               std::invalid_argument);
}