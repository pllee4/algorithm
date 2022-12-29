/*
 * weighted_astar.hpp
 *
 * Created on: Dec 29, 2022 09:20
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef WEIGHTED_ASTAR_HPP
#define WEIGHTED_ASTAR_HPP

#include "algorithm/robotics/astar_variant/astar_variant.hpp"

namespace pllee4::graph {
template <uint8_t weight>
using WeightedAstar = AstarVariant<1, weight>;
}  // namespace pllee4::graph
#endif /* WEIGHTED_ASTAR_HPP */
