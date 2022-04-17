/*
 * main.cpp
 *
 * Created on: Apr 17, 2022 17:18
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "digital_filter/butterworth.hpp"
#include "digital_filter/details/bilinear_transform.hpp"

using namespace pllee4::generic;

int main(int argc, char **argv) {
  Butterworth<double> butter(5, 10, 100,
                             Butterworth<double>::FilterType::kLowPass);

  std::cout << "a coefficients" << std::endl;
  auto a_coeff = butter.GetACoefficients();
  for (int i = 0; i < a_coeff.size(); i++) {
    std::cout << a_coeff(i) << " " << std::endl;
  }
  std::cout << "\nb coefficients" << std::endl;
  auto b_coeff = butter.GetBCoefficients();
  for (int i = 0; i < b_coeff.size(); i++) {
    std::cout << b_coeff(i) << " " << std::endl;
  }
}