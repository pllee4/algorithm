/*
 * astar_variant.hpp
 *
 * Created on: Oct 23, 2022 22:26
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef ASTAR_VARIANT_HPP
#define ASTAR_VARIANT_HPP

#include <functional>

#include "algorithm/robotics/astar_variant/astar_variant_base.hpp"
#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {
/**
 * @brief Get specification of variants of A*
 *
 * @tparam w1
 * @tparam w2
 * @tparam HeuristicFunc
 * @param heuristic_func
 * @return constexpr AstarVariantSpecification
 */
template <uint8_t w1, uint8_t w2, typename HeuristicFunc>
constexpr AstarVariantSpecification GetAstarVariantSpecification(
    HeuristicFunc heuristic_func) {
  return AstarVariantSpecification{w1, w2, heuristic_func};
}

/**
 * @brief Template class for variants of A*
 *
 * @tparam w1
 * @tparam w2
 */
template <uint8_t w1, uint8_t w2>
class AstarVariant : public AstarVariantBase {
 public:
  explicit AstarVariant(const MotionConstraintType motion_constraint_type)
      : motion_constraint_type_(motion_constraint_type),
        heuristic_func_(GetHeuristic(motion_constraint_type)) {
    SetMotionConstraint(GetMotionConstraint(motion_constraint_type));
  }

  AstarVariant(const MotionConstraintType motion_constraint_type,
               const HeuristicFunc& heuristic_func)
      : motion_constraint_type_(motion_constraint_type),
        heuristic_func_(heuristic_func) {
    SetMotionConstraint(GetMotionConstraint(motion_constraint_type));
  }

  AstarVariantSpecification GetAstarVariantSpec() override {
    return specification_;
  }

 private:
  MotionConstraintType motion_constraint_type_;
  HeuristicFunc heuristic_func_;
  AstarVariantSpecification specification_{
      GetAstarVariantSpecification<w1, w2>(heuristic_func_)};
};
}  // namespace pllee4::graph
#endif /* ASTAR_VARIANT_HPP */
