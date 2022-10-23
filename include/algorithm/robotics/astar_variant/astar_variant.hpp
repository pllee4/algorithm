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

#include <cmath>
#include <functional>

namespace pllee4::graph {
using HeuristicFunc = std::function<double(int, int)>;

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
class AstarVariant {
 public:
  // without heuristic_func
  AstarVariant() = default;

  // with heuristic_func
  explicit AstarVariant(HeuristicFunc heuristic_func)
      : heuristic_func_(heuristic_func),
        specification_(GetAstarVariantSpecification<w1, w2>(heuristic_func)) {}

 public:
  HeuristicFunc heuristic_func_{[](int, int) { return 0; }};
  AstarVariantSpecification specification_{
      GetAstarVariantSpecification<w1, w2>(heuristic_func_)};
};

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
 * @brief Get the heuristic function
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
      heuristic_func = [](int dx, int dy) {
        return static_cast<double>(abs(dx) + abs(dy));
      };
  }
  return heuristic_func;
}

using Dijikstra = AstarVariant<1, 0>;
using Astar = AstarVariant<1, 1>;
using BestFirstSearch = AstarVariant<0, 1>;
using WeightedAstar = AstarVariant<1, 10>;

}  // namespace pllee4::graph
#endif /* ASTAR_VARIANT_HPP */
