/*
 * path_finder_interface.hpp
 *
 * Created on: Oct 24, 2022 18:11
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef PATH_FINDER_INTERFACE_HPP
#define PATH_FINDER_INTERFACE_HPP

#include <optional>

#include "algorithm/robotics/shared_type/data_type.hpp"

namespace pllee4::graph {
struct PathFinderInterface {
  virtual ~PathFinderInterface() = default;

  virtual bool SetOccupiedGrid(const std::vector<Coordinate> &occupied_grid,
                               int x_size, int y_size) = 0;
  virtual bool SetStartAndDestination(const Coordinate &src,
                                      const Coordinate &dest) = 0;

  virtual std::optional<std::vector<Coordinate>> StepOverPathFinding() = 0;
  virtual bool FindPath() = 0;

  virtual std::optional<std::vector<Coordinate>> GetPath() = 0;
};
}  // namespace pllee4::graph

#endif /* PATH_FINDER_INTERFACE_HPP */
