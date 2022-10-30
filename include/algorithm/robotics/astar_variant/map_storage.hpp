/*
 * map_storage.hpp
 *
 * Created on: Oct 25, 2022 22:07
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef MAP_STORAGE_HPP
#define MAP_STORAGE_HPP

#include <cstddef>
#include <limits>
#include <vector>

#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
class MapStorage {
 public:
  using CostDataType = double;

 private:
  /**
   * @brief
   * f - travel cost
   * g - movement cost
   * h - estimated movement cost, heuristic
   */
  struct Cell {
    Coordinate parent_coordinate;
    CostDataType f{std::numeric_limits<CostDataType>::max()};
    CostDataType g{std::numeric_limits<CostDataType>::max()};
    CostDataType h{std::numeric_limits<CostDataType>::max()};
    bool occupied{false};
    bool operator==(const Cell &other) const {
      return (parent_coordinate == other.parent_coordinate);
    }
  };

 public:
  /*
  y
  ^
  |
  |
  |
  |_________>x
  */
  MapStorage(const size_t x_size, const size_t y_size)
      : map_x_size_(x_size), map_y_size_(y_size) {
    std::vector<std::vector<Cell>> map(map_x_size_,
                                       std::vector<Cell>(map_y_size_));
    map_ = map;
  }

  bool Contains(const Coordinate &coordinate) const {
    return (coordinate.x >= 0 && coordinate.x < static_cast<int>(map_x_size_) &&
            coordinate.y >= 0 && coordinate.y < static_cast<int>(map_y_size_));
  }

  std::vector<std::vector<Cell>> &GetMap() { return map_; }

  void ResetCost() {
    for (auto &row : map_) {
      for (auto &cell : row) {
        cell.parent_coordinate = {0, 0};
        cell.f = std::numeric_limits<CostDataType>::max();
        cell.g = std::numeric_limits<CostDataType>::max();
        cell.h = std::numeric_limits<CostDataType>::max();
      }
    }
  }

 private:
  std::vector<std::vector<Cell>> map_;

  size_t map_x_size_;
  size_t map_y_size_;
};
}  // namespace pllee4::graph
#endif /* MAP_STORAGE_HPP */
