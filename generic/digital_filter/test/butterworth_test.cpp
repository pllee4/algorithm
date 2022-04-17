/*
 * butterworth_test.cpp
 *
 * Created on: Apr 17, 2022 12:35
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "digital_filter/butterworth.hpp"

#include "gtest/gtest.h"

using namespace pllee4::generic;

TEST(Butterworth, ButtOrd) {
  auto [order, value] = Butterworth<float>::ButtOrd(0.18141, 0.49828, 1, 26.0);
  EXPECT_EQ(order, 4);
  EXPECT_FLOAT_EQ(value, 0.28009653);
}

TEST(Butterworth, Butter) {
  Butterworth<float> butter(5, 10, 100,
                            Butterworth<float>::FilterType::kLowPass);

  VectorXt<float> a_coeffs = (VectorXt<float>(6) << 1, -2.97542211, 3.80601812,
                              -2.54525287, 0.88113008, -0.12543062)
                                 .finished();
  VectorXt<float> b_coeffs =
      (VectorXt<float>(6) << 0.0012825811, 0.0064129054, 0.012825811,
       0.012825811, 0.00641290539, 0.0012825811)
          .finished();

  auto a_coeff_res = butter.GetACoefficients();
  auto b_coeff_res = butter.GetBCoefficients();

  for (Eigen::Index i = 0; i < a_coeff_res.size(); ++i)
    EXPECT_NEAR(a_coeff_res(i), a_coeffs(i), 0.000001);
  for (Eigen::Index i = 0; i < b_coeff_res.size(); ++i)
    EXPECT_NEAR(b_coeff_res(i), b_coeffs(i), 0.000001);
}