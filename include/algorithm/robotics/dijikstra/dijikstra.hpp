/**
 * @file dijikstra.hpp
 * @author Pin Loon Lee (pinloon_0428@hotmail.com)
 * @brief
 * @date 2022-03-10
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef DIJIKSTRA_HPP
#define DIJIKSTRA_HPP

#include <limits>
#include <vector>

#include "algorithm/robotics/shared_type/data_type.hpp"
#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {
/**
 * @brief
 *
 */

/*
y
^
|
|
|
|_________>x
*/

class Dijikstra {
 public:
  struct Cell {
    Coordinate parent_coordinate;
    float cost{std::numeric_limits<float>::max()};
    bool occupied{false};
    bool operator==(const Cell& other) const {
      return (parent_coordinate == other.parent_coordinate);
    }
  };

  explicit Dijikstra(const struct MotionConstraint& motion_constraint);
  ~Dijikstra() = default;

  bool SetOccupancyGrid(const std::vector<Coordinate>& occupancy_grid,
                        int x_size, int y_size);
  std::vector<Coordinate> GetPath(const Coordinate& dest);
  bool FindPath(const Coordinate& src, const Coordinate& dest);

 private:
  struct MotionConstraint motion_constraint_;
  std::vector<std::vector<Cell>> map_;

  int map_x_size_;
  int map_y_size_;

  bool IsValidCoordinate(const Coordinate& coordinate) const {
    return (coordinate.x >= 0 && coordinate.x < map_x_size_ &&
            coordinate.y >= 0 && coordinate.y < map_y_size_);
  }
};
}  // namespace pllee4::graph
#endif /* DIJIKSTRA_HPP */
