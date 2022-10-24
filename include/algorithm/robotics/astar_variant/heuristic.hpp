/*
 * heuristic.hpp
 *
 * Created on: Oct 24, 2022 11:35
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef HEURISTIC_HPP
#define HEURISTIC_HPP

#include <cmath>

#include "algorithm/robotics/shared_type/motion_constraint.hpp"

namespace pllee4::graph {

using HeuristicFunc = std::function<double(int, int)>;

/**
 * @brief Details of heuristic function
 *
 */
struct HeuristicFuncDetails {
  enum class Type {
    HEURISTIC_DIAGONAL,
    HEURISTIC_EUCLIDEAN,
    HEURISTIC_MANHATTAN,
  };
};

/**
 * @brief Get the heuristic function based on heuristic type
 *
 * @tparam type
 * @return constexpr HeuristicFunc
 */
template <HeuristicFuncDetails::Type type>
constexpr HeuristicFunc GetHeuristic() {
  HeuristicFunc heuristic_func;
  switch (type) {
    case HeuristicFuncDetails::Type::HEURISTIC_DIAGONAL:
      heuristic_func = [](int dx, int dy) {
        return std::max(abs(dx), abs(dy));
      };
      break;
    case HeuristicFuncDetails::Type::HEURISTIC_EUCLIDEAN:
      heuristic_func = [](int dx, int dy) { return sqrt(dx * dx + dy * dy); };
      break;
    case HeuristicFuncDetails::Type::HEURISTIC_MANHATTAN:
    default:
      heuristic_func = [](int dx, int dy) {
        return static_cast<double>(abs(dx) + abs(dy));
      };
  }
  return heuristic_func;
}

/**
 * @brief Get the heuristic function based on motion type
 *
 * @param motion_type
 * @return HeuristicFunc
 */
HeuristicFunc GetHeuristic(const MotionConstraintType motion_type) {
  HeuristicFunc heuristic_func;
  switch (motion_type) {
    case MotionConstraintType::CARDINAL_ORDINAL_MOTION:
      return GetHeuristic<HeuristicFuncDetails::Type::HEURISTIC_DIAGONAL>();
      break;
    case MotionConstraintType::ANY_MOTION:
      return GetHeuristic<HeuristicFuncDetails::Type::HEURISTIC_EUCLIDEAN>();
      break;
    case MotionConstraintType::CARDINAL_MOTION:
    default:
      return GetHeuristic<HeuristicFuncDetails::Type::HEURISTIC_MANHATTAN>();
      break;
  }
  return heuristic_func;
}
}  // namespace pllee4::graph

#endif /* HEURISTIC_HPP */
