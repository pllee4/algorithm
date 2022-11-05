/*
 * butterworth.tpp
 *
 * Created on: Apr 16, 2022 11:56
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef BUTTERWORTH_TPP
#define BUTTERWORTH_TPP

#include <cmath>
#include <stdexcept>

#include "algorithm/generic/common/types.hpp"
#include "algorithm/generic/digital_filter/details/bilinear_transform.hpp"
#include "algorithm/generic/polynomial/polynomial.hpp"

namespace pllee4::generic {
template <typename T>
std::pair<int, T> Butterworth<T>::ButtOrd(T wpass, T wstop, T apass, T astop) {
  if (wpass <= T(0) || wpass >= T(1))
    throw std::invalid_argument{"wpass out of range (0-1)"};
  if (wstop <= T(0) || wstop >= T(1))
    throw std::invalid_argument{"wstop out of range (0-1)"};

  T num = std::log10((std::pow(T(10), T(0.1) * std::abs(astop)) - 1) /
                     (std::pow(T(10), T(0.1) * std::abs(apass)) - 1));
  // pre-warp
  T w_pass = std::tan(T(0.5) * pi<T> * wpass);
  T w_stop = std::tan(T(0.5) * pi<T> * wstop);

  T w;

  if (w_pass < w_stop)  // low pass
    w = std::abs(w_stop / w_pass);
  else  // high pass
    w = std::abs(w_pass / w_stop);
  //   // band stop
  //   w = (w_stop * (w_pass(1) - w_pass(2))) /
  //       (std::pow(w_stop, 2) - w_pass(1) * w_pass(2));
  //   // band pass
  //   w = (std::pow(w_stop, 2) - w_pass(1) * w_pass(2)) /
  //       (w_stop * (w_pass(1) - w_pass(2)));

  T denum = T(2) * std::log10(w);

  int order = static_cast<int>(std::ceil(num / denum));

  // natural frequency
  T w0 = w / std::pow(std::pow(T(10), T(0.1) * std::abs(astop)) - 1,
                      T(1) / T(2 * order));
  T wn;
  // low pass
  if (w_pass < w_stop)
    wn = w0 * w_pass;
  else
    wn = w_pass / w0;
  // // band stop
  // wn(1) = ((w_pass(2) - w_pass(1)) +
  //          sqrt(std::pow(w_pass(2) - w_pass(1), 2) +
  //               4 * std::pow(w0, 2) * w_pass(1) * w_pass(2))) /
  //         (2 * w0);
  // wn(2) = ((w_pass(2) - w_pass(1)) -
  //          sqrt(std::pow(w_pass(2) - w_pass(1), 2) +
  //               4 * std::pow(w0, 2) * w_pass(1) * w_pass(2))) /
  //         (2 * w0);

  // wn = sort(abs(wn));
  // // band pass
  // w0 = [-w0 w0];  // need both left and right 3dB frequencies
  // wn = -w0 * (w_pass(2) - w_pass(1)) / 2 +
  //      sqrt(std::pow(w0, 2) / 4 * std::pow(w_pass(2) - w_pass(1), 2) +
  //           w_pass(1) * w_pass(2));
  // wn = sort(abs(wn));

  return std::pair<int, T>(order, T(2) * std::atan(wn) / pi<T>);
}

template <typename T>
Butterworth<T>::Butterworth(int order, T fc, T fs, FilterType filter_type)
    : order_(order), fs_(fs), filter_type_(filter_type) {
  ComputeDigitalFilter(fc);
  input_.resize(order_ + 1);
  filtered_output_.resize(order_ + 1);
  input_(0) = T();
  filtered_output_(0) = T();
}

template <typename T>
T Butterworth<T>::StepFilter(const T &data) {
  for (Eigen::Index i = input_.size() - 1; i > 0; --i)
    input_(i) = input_(i - 1);

  for (Eigen::Index i = filtered_output_.size() - 1; i > 0; --i)
    filtered_output_(i) = filtered_output_(i - 1);

  input_(0) = data;
  // to avoid normalized coefficient for y[n] to be included
  filtered_output_(0) = 0;
  filtered_output_(0) = b_coeff_.dot(input_) - a_coeff_.dot(filtered_output_);
  return filtered_output_(0);
}

template <typename T>
void Butterworth<T>::SetCutoffFrequency(const T &fc) {
  ComputeDigitalFilter(fc);
}

template <typename T>
VectorXct<T> Butterworth<T>::GetAnalogZeros(T analog_fc) {
  switch (filter_type_) {
    case FilterType::kHighPass:
      return VectorXct<T>::Constant(order_, std::complex<T>(1));
    // case FilterType::kBandStop: {
    //   T w0 = T(2) * std::atan(pi<T> * analog_fc / fs_);
    //   return (VectorXct<T>(2 * order_) << VectorXct<T>::Constant(
    //               order_, std::exp(std::complex<T>(0, w0))),
    //           VectorXct<T>::Constant(order_, std::exp(std::complex<T>(0, -w0))))
    //       .finished();
    // }
    // case FilterType::kBandPass:
    //   return (VectorXct<T>(2 * order_)
    //               << VectorXct<T>::Constant(order_, std::complex<T>(-1)),
    //           VectorXct<T>::Constant(order_, std::complex<T>(1)))
    //       .finished();
    case FilterType::kLowPass:
    default:
      return VectorXct<T>::Constant(order_, std::complex<T>(-1));
  }
}

template <typename T>
std::complex<T> Butterworth<T>::GetAnalogPoles(int k, T analog_fc) {
  T theta = static_cast<T>(2 * k - 1) * pi<T> / static_cast<T>(2 * order_);

  std::complex<T> analog_pole(-std::sin(theta), std::cos(theta));

  // scale s-plane
  switch (filter_type_) {
    case FilterType::kHighPass:
      return T(2) * pi<T> * analog_fc / analog_pole;
    case FilterType::kLowPass:
    default:
      return T(2) * pi<T> * analog_fc * analog_pole;
  }
}

template <typename T>
void Butterworth<T>::ComputeGainAndNormalize(VectorXt<T> &a_coeff,
                                             VectorXt<T> &b_coeff) {
  T sum_a = 0;
  T sum_b = 0;

  switch (filter_type_) {
    case FilterType::kHighPass:
      for (int i = 0; i < order_ + 1; ++i) {
        sum_a += a_coeff(i) * std::pow(-1, i);
        sum_b += b_coeff(i) * std::pow(-1, i);
      }
      break;
      // case FilterType::kBandPass: {
      // std::complex<T> complex_a(a_coeff(0));
      // std::complex<T> complex_b(b_coeff(0));

      // for (int i = 1; i < 2 * order_ + 1; ++i) {
      //   complex_a = complex_a * bpS + a_coeff(i);
      //   complex_b = complex_b * bpS + b_coeff(i);
      // }
      // sum_a = std::abs(complex_a);
      // sum_b = std::abs(complex_b);
    // } break;
    // case FilterType::kBandStop:
    case FilterType::kLowPass:
    default:
      sum_a = a_coeff.sum();
      sum_b = b_coeff.sum();
      break;
  }

  T K = sum_a / sum_b;
  b_coeff *= K;

  T a0 = a_coeff(0);
  if (std::abs(a0 - T(1)) < std::numeric_limits<T>::epsilon()) {
  } else {
    a_coeff /= a0;
    b_coeff /= a0;
  }
  a_coeff_ = a_coeff;
  b_coeff_ = b_coeff;
}

template <typename T>
void Butterworth<T>::ComputeDigitalFilter(T fc) {
  T analog_fc = fs_ / pi<T> * std::tan(pi<T> * fc / fs_);
  VectorXct<T> poles(order_);
  std::complex<T> analog_pole;

  for (int k = 0; k < order_; k++) {
    analog_pole = GetAnalogPoles(k + 1, analog_fc);  // k = 1:N
    BilinearTransform<std::complex<T>>::SToZ(fs_, analog_pole, poles(k));
  }

  VectorXct<T> zeros = GetAnalogZeros(analog_fc);
  VectorXct<T> a = Polynomial<std::complex<T>>::GetPolyCoeffFromRoots(poles);
  VectorXct<T> b = Polynomial<std::complex<T>>::GetPolyCoeffFromRoots(zeros);

  VectorXt<T> a_coeff(order_ + 1);
  VectorXt<T> b_coeff(order_ + 1);
  for (int i = 0; i < order_ + 1; ++i) {
    a_coeff(i) = a(i).real();
    b_coeff(i) = b(i).real();
  }

  ComputeGainAndNormalize(a_coeff, b_coeff);
}
}  // namespace pllee4::generic
#endif /* BUTTERWORTH_TPP */
