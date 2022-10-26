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

#include <memory>
#include <set>

#include "algorithm/robotics/astar_variant/heuristic.hpp"
#include "algorithm/robotics/astar_variant/map_storage.hpp"
#include "algorithm/robotics/path_finder/path_finder_interface.hpp"
#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {
class AstarVariantBase : public PathFinderInterface {
 public:
  virtual ~AstarVariantBase() = default;

  void SetMapStorageSize(const size_t x_size, const size_t y_size);

  // Interface inherited
  bool SetOccupiedGrid(const std::vector<Coordinate> &occupied_grid) override;

  bool SetStartAndDestination(const Coordinate &start,
                              const Coordinate &dest) override;

  std::optional<std::vector<Coordinate>> StepOverPathFinding() override;
  bool FindPath() override;

  std::optional<std::vector<Coordinate>> GetPath() override;

  // To be overridden by derived classes
  virtual HeuristicFunc GetHeuristicFunc() = 0;

 protected:
  void SetMotionConstraint(MotionConstraint motion_constraint);

 private:
  MotionConstraint motion_constraint_;
  std::unique_ptr<MapStorage> map_storage_;

  std::vector<std::vector<bool>> visited_map_;
  std::set<std::pair<int, std::pair<int, int>>> traverse_path_;

  Coordinate start_;
  Coordinate dest_;
  bool start_and_end_set_{false};
  bool found_path_{false};
};
}  // namespace pllee4::graph
#endif /* ASTAR_VARIANT_BASE_HPP */
