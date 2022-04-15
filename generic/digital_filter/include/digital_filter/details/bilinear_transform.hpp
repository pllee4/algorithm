/*
 * bilinear_transform.hpp
 *
 * Created on: Apr 12, 2022 22:47
 * Description: Bilinear transform
 *
 * Reference: https://www.dsprelated.com/showarticle/1119.php
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */
#ifndef BILINEAR_TRANSFORM_HPP
#define BILINEAR_TRANSFORM_HPP

#include <complex>

namespace pllee4::generic {

// Reference: https://stackoverflow.com/a/41449096/9774052
/**
 * @brief Check whether is the type desired
 *
 * @tparam T type
 * @return std::true_type
 */
template <typename T>
std::true_type is_type(std::complex<T>);
std::false_type is_type(...);

/**
 * @brief Check whether is complex type
 *
 * @tparam T type
 */
template <typename T>
using is_complex = decltype(is_type(std::declval<T>()));

/**
 * @brief Sub type of non container
 *
 * @tparam T type
 * @tparam bool Expect dispatch for false value
 */
template <typename T, bool>
struct sub_type {
  using type = T;
};

/**
 * @brief Sub type of container
 *
 * @tparam T type
 * @tparam bool Expect dispatch for true value
 */
template <typename T>
struct sub_type<T, true> {
  using type = typename T::value_type;
};

/**
 * @brief Get sub type
 *
 * If is complex type, equals to its value_type, else just its type
 *
 * @tparam T type
 */
template <typename T>
using sub_type_t = typename sub_type<T, is_complex<T>::value>::type;

template <typename T>
struct BilinearTransform {
  using SubType = sub_type_t<T>;
  static_assert(std::is_floating_point<SubType>::value,
                "Only floating point types (real and complex) is accepted for "
                "BilinearTransform.");

  static void SToZ(SubType fs, const T& s_pole, T& z_pole);
  static void ZToS(SubType fs, const T& z_pole, T& s_pole);
};

template <typename T>
void BilinearTransform<T>::SToZ(SubType fs, const T& s_pole, T& z_pole) {
  if (std::abs(2 * fs - s_pole) <= std::numeric_limits<SubType>::epsilon())
    throw std::invalid_argument{"Division by zero for SToZ"};
  T scale_pole = s_pole / (2 * fs);
  z_pole = (T(1) + scale_pole) / (T(1) - scale_pole);
}

template <typename T>
void BilinearTransform<T>::ZToS(SubType fs, const T& z_pole, T& s_pole) {
  if (std::abs(z_pole + T(1)) <= std::numeric_limits<SubType>::epsilon())
    throw std::invalid_argument{"Division by zero for ZToS"};
  T z_pole_inv = T(1) / z_pole;
  s_pole = 2 * fs * (T(1) - z_pole_inv) / (T(1) + z_pole_inv);
}

}  // namespace pllee4::generic

#endif /* BILINEAR_TRANSFORM_HPP */
