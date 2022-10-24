/*
 * astar_variant_base.hpp
 *
 * Created on: Oct 24, 2022 18:12
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef ASTAR_VARIANT_BASE_HPP
#define ASTAR_VARIANT_BASE_HPP

#include "algorithm/robotics/path_finder/path_finder_interface.hpp"
#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {
class AstarVariantBase : public PathFinderInterface {
 public:
  virtual ~AstarVariantBase() = default;

  // Interface inherited
  bool SetOccupiedGrid(const std::vector<Coordinate> &occupied_grid, int x_size,
                       int y_size) override;

  bool SetStartAndDestination(const Coordinate &src,
                              const Coordinate &dest) override;

  std::optional<std::vector<Coordinate>> StepOverPathFinding() override;
  bool FindPath() override;

  std::optional<std::vector<Coordinate>> GetPath() override;

 protected:
  void SetMotionConstraint(MotionConstraint motion_constraint) {
    motion_constraint_ = motion_constraint;
  }

  MotionConstraint motion_constraint_;
};
}  // namespace pllee4::graph
#endif /* ASTAR_VARIANT_BASE_HPP */
