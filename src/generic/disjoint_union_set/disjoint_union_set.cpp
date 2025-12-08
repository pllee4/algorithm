/*
 * disjoint_union_set.cpp
 *
 * Created on: Dec 08, 2025 22:43
 * Description:
 *
 * Copyright (c) 2025 Pin Loon Lee (pllee4)
 */

#include "algorithm/generic/disjoint_union_set/disjoint_union_set.hpp"

#include <numeric>

namespace pllee4::generic {
DisjointUnionSets::DisjointUnionSets(int n) {
  rank_.resize(n, 0);
  parent_.resize(n);
  std::iota(parent_.begin(), parent_.end(), 0);
}

int DisjointUnionSets::Find(int i) {
  if (parent_[i] != i) {
    parent_[i] = Find(parent_[i]);
  }
  return parent_[i];
}

void DisjointUnionSets::UnionSets(int x, int y) {
  int xRoot = Find(x);
  int yRoot = Find(y);
  if (xRoot == yRoot) {
    return;
  }
  if (rank_[xRoot] < rank_[yRoot]) {
    parent_[xRoot] = yRoot;
  } else if (rank_[yRoot] < rank_[xRoot]) {
    parent_[yRoot] = xRoot;
  } else {
    parent_[yRoot] = xRoot;
    ++rank_[xRoot];
  }
}
}  // namespace pllee4::generic
