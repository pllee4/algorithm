/*
 * polynomial.hpp
 *
 * Created on: Apr 10, 2022 18:21
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef POLYNOMIAL_HPP
#define POLYNOMIAL_HPP

#include "algorithm/generic/common/types.hpp"

namespace pllee4::generic {

template <typename T>
struct Polynomial {
  static VectorXt<T> GetPolyCoeffFromRoots(const VectorXt<T>& roots);
};

template <typename T>
VectorXt<T> Polynomial<T>::GetPolyCoeffFromRoots(const VectorXt<T>& roots) {
  VectorXt<T> coeffs = VectorXt<T>::Zero(roots.size() + 1);
  coeffs(0) = T(1);
  for (Eigen::Index i = 0; i < roots.size(); ++i) {
    for (Eigen::Index k = i + 1; k > 0; --k) {
      coeffs(k) -= roots(i) * coeffs(k - 1);
    }
  }
  return coeffs;
}

}  // namespace pllee4::generic
#endif /* POLYNOMIAL_HPP */
