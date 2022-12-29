/* 
 * best_first_search.hpp
 * 
 * Created on: Dec 29, 2022 09:23
 * Description: 
 * 
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */ 

#ifndef BEST_FIRST_SEARCH_HPP
#define BEST_FIRST_SEARCH_HPP

#include "algorithm/robotics/astar_variant/astar_variant.hpp"

namespace pllee4::graph {
using BestFirstSearch = AstarVariant<0, 1>;
}  // namespace pllee4::graph
#endif /* BEST_FIRST_SEARCH_HPP */
