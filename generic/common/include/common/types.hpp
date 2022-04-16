/*
 * types.hpp
 *
 * Created on: Apr 16, 2022 11:30
 * Description: Common types
 *
 * Reference:
 * https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef TYPES_HPP
#define TYPES_HPP

#include <Eigen/Core>

namespace pllee4::generic {
template <typename T>
using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T>
using VectorXct = VectorXt<std::complex<T>>;

}  // namespace pllee4::generic

#endif /* TYPES_HPP */
