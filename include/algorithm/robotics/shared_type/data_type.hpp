/*
 * data_type.hpp
 *
 * Created on: Sep 27, 2022 20:59
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef DATA_TYPE_HPP
#define DATA_TYPE_HPP

namespace pllee4::graph {
/**
 * @brief Coordinate in int
 *
 */
struct Coordinate {
  int x;
  int y;
  bool operator==(const Coordinate &other) const {
    return (x == other.x && y == other.y);
  }
};
}  // namespace pllee4::graph
#endif /* DATA_TYPE_HPP */
