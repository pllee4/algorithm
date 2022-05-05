/*
 * butterworth.hpp
 *
 * Created on: Apr 15, 2022 15:38
 * Description:
 *
 * Reference: https://www.mathworks.com/help/signal/ref/buttord.html
 *            https://github.com/g2e/seizmo/blob/master/misc/buttord2.m
 *            https://octave.sourceforge.io/signal/function/buttord.html
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef BUTTERWORTH_HPP
#define BUTTERWORTH_HPP

#include <complex>
#include <utility>

#include "common/types.hpp"

namespace pllee4::generic {
template <typename T>
constexpr T pi =
    T(3.14159265358979323846264338327950288419716939937510582097494459230781L);

template <typename T>
class Butterworth {
 public:
  // currently only support low and high
  enum class FilterType {
    kLowPass,
    kHighPass,
    // kBandStop,
    // kBandPass,
  };

  /**
   * @brief Find Butterworth Order
   *        Currently only support low and high pass
   * @tparam T
   * @param wpass upper passband edge frequency
   * @param wstop lower stopband edge frequency
   * @param apass maximum pass band attenuation
   * @param astop minimum stop band attenuation
   * @return std::pair<int, T>
   */
  static std::pair<int, T> ButtOrd(T wpass, T wstop, T apass, T astop);

  /**
   * @brief Construct a new Butterworth object
   *
   * @param order order of filter
   * @param fc cutoff frequency
   * @param fs sampling frequency
   * @param filter_type filter type
   */
  Butterworth(int order, T fc, T fs,
              FilterType filter_type = FilterType::kLowPass);

  /**
   * @brief Get coefficients of a
   * 
   * @return VectorXt<T> 
   */
  VectorXt<T> GetACoefficients() const { return a_coeff_; }

  /**
   * @brief Get coefficients of b
   * 
   * @return VectorXt<T> 
   */
  VectorXt<T> GetBCoefficients() const { return b_coeff_; }

 private:
  int order_;
  T fs_;
  FilterType filter_type_;
  VectorXt<T> a_coeff_;  // poles
  VectorXt<T> b_coeff_;  // zeros

  /**
   * @brief Get analog zeros
   *
   * @param analog_fc analog cutoff frequency
   * @return VectorXct<T>
   */
  VectorXct<T> GetAnalogZeros(T analog_fc);

  /**
   * @brief Get the analog poles
   *
   * @param k k
   * @param analog_fc analog cutoff frequency
   * @return std::complex<T>
   */
  std::complex<T> GetAnalogPoles(int k, T analog_fc);

  /**
   * @brief Compute gain and coefficients for a and b
   *
   * @param a_coeff vector of a coefficients
   * @param b_coeff vector of b coefficients
   */
  void ComputeGainAndNormalize(VectorXt<T>& a_coeff, VectorXt<T>& b_coeff);

  /**
   * @brief Compute digital filter
   *
   * @param fc cutoff frequency
   */
  void ComputeDigitalFilter(T fc);
};

}  // namespace pllee4::generic
#endif /* BUTTERWORTH_HPP */

#include "butterworth.tpp"
