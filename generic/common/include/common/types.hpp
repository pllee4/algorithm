/*
 * types.hpp
 *
 * Created on: Apr 16, 2022 11:30
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef TYPES_HPP
#define TYPES_HPP

#include <Eigen/Core>

namespace pllee4::generic {
template <typename T>
using vectX_t = Eigen::Matrix<T, Eigen::Dynamic, 1>; /*!< Eigen column-vector */
}

#endif /* TYPES_HPP */
