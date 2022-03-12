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

#include <climits>
#include <vector>

namespace pllee4::graph {

/**
 * @brief Coordinate
 *
 */
struct Coordinate {
  int x;
  int y;
  bool operator==(const Coordinate& other) const {
    return (x == other.x && y == other.y);
  }
};

/**
 * @brief
 *
 */
struct Cell {
  Coordinate parent_coordinate;
  int cost{INT_MAX};
  bool occupied{false};
  bool operator==(const Cell& other) const {
    return (parent_coordinate == other.parent_coordinate);
  }
};

struct MotionConstraint {
  std::vector<int> dx;
  std::vector<int> dy;
  size_t size() const { return dx.size(); }
};

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
