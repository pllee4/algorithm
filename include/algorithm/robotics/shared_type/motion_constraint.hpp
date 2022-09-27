/*
 * motion_constraint.hpp
 *
 * Created on: Sep 27, 2022 21:11
 * Description:
 *
 * Copyright (c) 2022 Pin Loon Lee (pllee4)
 */

#ifndef MOTION_CONSTRAINT_HPP
#define MOTION_CONSTRAINT_HPP

namespace pllee4::graph {
/**
 * @brief MotionConstraint for x and y
 *
 */
struct MotionConstraint {
  std::vector<int> dx;
  std::vector<int> dy;
  size_t size() const { return dx.size(); }
};
}  // namespace pllee4::graph

#endif /* MOTION_CONSTRAINT_HPP */
