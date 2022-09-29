/*
 * main.cpp
 *
 * Created on: Sep 29, 2022 22:21
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#include <iostream>

#include "algorithm/robotics/bresenham/bresenham.hpp"

using namespace pllee4::graph;

int main(int argc, char **argv) {
  Coordinate start{100, 110};
  Coordinate end{125, 120};
  LineGenerator line(start, end);
  for (auto const &coord : line.GetPoints())
    std::cout << "x " << coord.x << ", y " << coord.y << std::endl;
}