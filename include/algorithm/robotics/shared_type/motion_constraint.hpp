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
 * @brief Type of motion constraint
 *
 */
enum class MotionConstraintType {
  CARDINAL_MOTION,
  CARDINAL_ORDINAL_MOTION,
  ANY_MOTION,
};

/**
 * @brief MotionConstraint for x and y
 *
 */
struct MotionConstraint {
  std::vector<int> dx;
  std::vector<int> dy;
  size_t size() const { return dx.size(); }
};

/**
 * @brief Get the MotionConstraint based on motion constraint_type
 *        Marked as inline as this is compiled as interface
 *
 * @param motion_constraint_type
 * @return MotionConstraint
 */
inline MotionConstraint GetMotionConstraint(
    const MotionConstraintType motion_constraint_type) {
  MotionConstraint motion_constraint;
  if (motion_constraint_type == MotionConstraintType::CARDINAL_MOTION) {
    motion_constraint.dx = {1, 0, -1, 0};
    motion_constraint.dy = {0, 1, 0, -1};
  } else {
    motion_constraint.dx = {1, 0, -1, 0, 1, -1, 1, -1};
    motion_constraint.dy = {0, 1, 0, -1, 1, -1, -1, 1};
  }
  return motion_constraint;
}

}  // namespace pllee4::graph

#endif /* MOTION_CONSTRAINT_HPP */
