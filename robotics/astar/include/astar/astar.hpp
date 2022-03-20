/**
 * @file astar.hpp
 * @author Pin Loon Lee (pinloon_0428@hotmail.com)
 * @brief
 * @date 2022-03-18
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include <limits>
#include <vector>

namespace pllee4::graph {

/**
 * @brief Coordinate
 *
 */
struct Coordinate {
  int x;
  int y;
  bool operator==(const Coordinate &other) const {
    return (x == other.x && y == other.y);
  }
};

struct MotionConstraint {
  std::vector<int> dx;
  std::vector<int> dy;
  size_t size() const { return dx.size(); }
};

class AStar {
 public:
  /**
   * @brief
   * f - travel cost
   * g - movement cost
   * h - estimated movement cost, heuristic
   */
  struct Cell {
    Coordinate parent_coordinate;
    float f{std::numeric_limits<float>::max()};
    float g{std::numeric_limits<float>::max()};
    float h{std::numeric_limits<float>::max()};
    bool occupied{false};
    bool operator==(const Cell &other) const {
      return (parent_coordinate == other.parent_coordinate);
    }
  };

  enum class MotionConstraintType {
    CARDINAL_MOTION,
    CARDINAL_ORDINAL_MOTION,
    ANY_MOTION,
  };

  explicit AStar(const MotionConstraintType &motion_constraint_type);
  ~AStar() = default;

  bool SetOccupancyGrid(const std::vector<Coordinate> &occupancy_grid,
                        int x_size, int y_size);
  std::vector<Coordinate> GetPath(const Coordinate &dest);
  bool FindPath(const Coordinate &src, const Coordinate &dest);

 private:
  MotionConstraintType motion_constraint_type_;
  MotionConstraint motion_constraint_;
  std::vector<std::vector<Cell>> map_;

  int map_x_size_;
  int map_y_size_;

  bool IsValidCoordinate(const Coordinate &coordinate) const {
    return (coordinate.x >= 0 && coordinate.x < map_x_size_ &&
            coordinate.y >= 0 && coordinate.y < map_y_size_);
  }

  float ComputeH(const Coordinate &curr, const Coordinate &dest);
};

}  // namespace pllee4::graph

#endif /* ASTAR_HPP */
