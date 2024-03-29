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
/**
 * @brief Specification for variants of A*
 * f = w1 * g + w2 * h
 */
struct AstarVariantSpecification {
  uint8_t w1;
  uint8_t w2;
  HeuristicFunc heuristic_func{[](int, int) {
    return 0;
  }};  // default return 0 to avoid possibly segmentation fault
};
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

  void Reset() override;

  // To be overridden by derived classes
  virtual AstarVariantSpecification GetAstarVariantSpec() = 0;

 protected:
  void SetMotionConstraint(const MotionConstraint &motion_constraint);

 private:
  void ResetFunc(bool reset_cost_only = false);

  MotionConstraint motion_constraint_;
  std::unique_ptr<MapStorage> map_storage_;

  std::vector<std::vector<bool>> visited_map_;
  std::set<std::pair<int, std::pair<int, int>>> traverse_path_;

  Coordinate start_;
  Coordinate dest_;
  bool start_and_end_set_{false};
  bool found_path_{false};
  bool reset_called_{false};
};
}  // namespace pllee4::graph
#endif /* ASTAR_VARIANT_BASE_HPP */
