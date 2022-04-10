/*
 * main.cpp
 *
 * Created on: Apr 10, 2022 20:23
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "polynomial/polynomial.hpp"

using namespace pllee4::generic;

int main(int argc, char** argv) {
  vectX_t<float> roots = (vectX_t<float>(2) << 2, 3).finished();
  vectX_t<float> polycoeff = Polynomial<float>::GetPolyCoeffFromRoots(roots);
  for (int i = 0; i < polycoeff.size(); ++i) {
    std::cout << polycoeff(i) << std::endl;
  }
}