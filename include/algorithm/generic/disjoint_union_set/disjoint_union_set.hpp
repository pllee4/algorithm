/*
 * @file disjoint_union_set.hpp
 *
 * Created on: Dec 08, 2025 22:41
 * @brief Implementation of the Disjoint Set Union (DSU) or Union-Find data structure.
 *
 * @details This data structure keeps track of a set of elements partitioned into a
 * number of disjoint (non-overlapping) subsets. It provides two main
 * operations: finding the set to which an element belongs and merging two sets.
 *
 * Copyright (c) 2025 Pin Loon Lee (pllee4)
 */

#pragma once

#include <vector>

namespace pllee4::generic {
/**
 * @brief A class representing the Disjoint Set Union (DSU) data structure.
 * @details Also known as Union-Find, this data structure manages a collection of
 * disjoint sets. It is highly optimized for finding which set an element
 * belongs to and for merging two sets, using path compression and union by
 * rank.
 */
class DisjointUnionSets {
 public:
  /**
   * @brief Constructs a new Disjoint Union Sets object.
   * @param n The initial number of elements, each in its own set.
   */
  explicit DisjointUnionSets(int n);

  /**
   * @brief Finds the representative (or root) of the set containing element i.
   * @details This method uses path compression to flatten the tree structure,
   * speeding up future find operations.
   * @param i The element to find.
   * @return The representative of the set containing i.
   */
  int Find(int i);

  /**
   * @brief Merges the sets containing elements x and y.
   * @details This method uses union by rank to keep the tree structures flat.
   * If the two elements are already in the same set, no action is taken.
   * @param x An element in the first set.
   * @param y An element in the second set.
   */
  void UnionSets(int x, int y);

 private:
  std::vector<int> parent_;  // parent_[i] stores the parent of element i.
  std::vector<int> rank_;    // rank_[i] stores the rank (an upper bound on the
                             // height) of the tree rooted at i.
};
}  // namespace pllee4::generic