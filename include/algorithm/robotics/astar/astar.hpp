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

#include "algorithm/robotics/shared_type/data_type.hpp"
#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {
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
    double f{std::numeric_limits<double>::max()};
    double g{std::numeric_limits<double>::max()};
    double h{std::numeric_limits<double>::max()};
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

  double ComputeH(const Coordinate &curr, const Coordinate &dest) const;
};

}  // namespace pllee4::graph

#endif /* ASTAR_HPP */
