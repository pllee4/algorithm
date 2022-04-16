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

#include "common/types.hpp"

namespace pllee4::generic {

template <typename T>
struct Polynomial {
  static vectX_t<T> GetPolyCoeffFromRoots(const vectX_t<T>& roots);
};

template <typename T>
vectX_t<T> Polynomial<T>::GetPolyCoeffFromRoots(const vectX_t<T>& roots) {
  vectX_t<T> coeffs = vectX_t<T>::Zero(roots.size() + 1);
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
