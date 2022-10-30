/*
 * butterworth_test.cpp
 *
 * Created on: Apr 17, 2022 12:35
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "algorithm/generic/digital_filter/butterworth.hpp"

#include "gtest/gtest.h"

using namespace pllee4::generic;

TEST(Butterworth, ButtOrdThrow) {
  EXPECT_THROW(Butterworth<double>::ButtOrd(1.2, 0.49828, 1, 26.0),
               std::invalid_argument);
  EXPECT_THROW(Butterworth<double>::ButtOrd(0.49828, -2.0, 1, 26.0),
               std::invalid_argument);
}

TEST(Butterworth, ButtOrdLowPass) {
  auto [order, value] = Butterworth<double>::ButtOrd(0.18141, 0.49828, 1, 26.0);
  EXPECT_EQ(order, 4);
  EXPECT_FLOAT_EQ(value, 0.28009653);
}

TEST(Butterworth, ButtOrdHighPass) {
  auto [order, value] = Butterworth<double>::ButtOrd(0.49828, 0.18141, 1, 26.0);
  EXPECT_EQ(order, 4);
  EXPECT_FLOAT_EQ(value, 0.3528198);
}

TEST(Butterworth, ButterLowPass) {
  Butterworth<double> butter(5, 10, 100,
                             Butterworth<double>::FilterType::kLowPass);

  VectorXt<double> a_coeffs = (VectorXt<double>(6) << 1, -2.97542211,
                               3.80601812, -2.54525287, 0.88113008, -0.12543062)
                                  .finished();
  VectorXt<double> b_coeffs =
      (VectorXt<double>(6) << 0.0012825811, 0.0064129054, 0.012825811,
       0.012825811, 0.00641290539, 0.0012825811)
          .finished();

  auto a_coeff_res = butter.GetACoefficients();
  auto b_coeff_res = butter.GetBCoefficients();

  for (Eigen::Index i = 0; i < a_coeff_res.size(); ++i)
    EXPECT_NEAR(a_coeff_res(i), a_coeffs(i), 0.000001);
  for (Eigen::Index i = 0; i < b_coeff_res.size(); ++i)
    EXPECT_NEAR(b_coeff_res(i), b_coeffs(i), 0.000001);
}

TEST(Butterworth, ButterHighPass) {
  Butterworth<double> butter(5, 10, 100,
                             Butterworth<double>::FilterType::kHighPass);

  VectorXt<double> a_coeffs = (VectorXt<double>(6) << 1, -2.97542211,
                               3.80601812, -2.54525287, 0.88113008, -0.12543062)
                                  .finished();
  VectorXt<double> b_coeffs = (VectorXt<double>(6) << 0.35416418, -1.77082095,
                               3.54164181, -3.54164181, 1.77082091, -0.35416418)
                                  .finished();

  auto a_coeff_res = butter.GetACoefficients();
  auto b_coeff_res = butter.GetBCoefficients();

  for (Eigen::Index i = 0; i < a_coeff_res.size(); ++i)
    EXPECT_NEAR(a_coeff_res(i), a_coeffs(i), 0.000001);
  for (Eigen::Index i = 0; i < b_coeff_res.size(); ++i)
    EXPECT_NEAR(b_coeff_res(i), b_coeffs(i), 0.000001);
}

TEST(Butterworth, StepFilter) {
  Butterworth<double> butter(1, 5, 1000,
                             Butterworth<double>::FilterType::kLowPass);
  std::vector<double> expected_values = {0.00773315, 0.0229602, 0.0377163,
                                         0.052016, 0.0658733};
  auto check_filtered_signal = [&butter](const double value) {
    EXPECT_NEAR(butter.StepFilter(0.5), value, 0.001);
  };
  std::for_each(expected_values.begin(), expected_values.end(),
                check_filtered_signal);
}

TEST(Butterworth, SetCutoffFrequency) {
  Butterworth<double> butter(1, 10, 1000,
                             Butterworth<double>::FilterType::kLowPass);
  butter.SetCutoffFrequency(5);
  std::vector<double> expected_values = {0.00773315, 0.0229602, 0.0377163,
                                         0.052016, 0.0658733};
  auto check_filtered_signal = [&butter](const double value) {
    EXPECT_NEAR(butter.StepFilter(0.5), value, 0.001);
  };
  std::for_each(expected_values.begin(), expected_values.end(),
                check_filtered_signal);
}