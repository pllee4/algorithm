/*
 * polynomial_test.cpp
 *
 * Created on: Apr 10, 2022 20:14
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include "polynomial/polynomial.hpp"

#include "gtest/gtest.h"

using namespace pllee4::generic;

template <typename T>
using c_t = std::complex<T>;

using c_int_t = c_t<int>;

TEST(Polynomial, Int) {
  vectX_t<int> roots = (vectX_t<int>(3) << 2, 3, 4).finished();
  vectX_t<int> coeffs = (vectX_t<int>(4) << 1, -9, 26, -24).finished();
  auto results = Polynomial<int>::GetPolyCoeffFromRoots(roots);
  for (Eigen::Index i = 0; i < results.size(); ++i)
    EXPECT_EQ(results(i), coeffs(i));
}

TEST(Polynomial, IntComplex) {
  vectX_t<c_int_t> roots = (vectX_t<c_int_t>(4) << c_int_t{1, 1},
                            c_int_t{-1, 4}, c_int_t{12, -3}, c_int_t{5, 2})
                               .finished();
  vectX_t<c_int_t> coeffs =
      (vectX_t<c_int_t>(5) << c_int_t{1, 0}, c_int_t{-17, -4}, c_int_t{66, 97},
       c_int_t{127, -386}, c_int_t{-357, 153})
          .finished();
  auto res = Polynomial<c_int_t>::GetPolyCoeffFromRoots(roots);
  for (Eigen::Index i = 0; i < res.size(); ++i) EXPECT_EQ(res(i), coeffs(i));
}