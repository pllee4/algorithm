/*
 * disjoint_union_set_test.cpp
 *
 * Created on: Dec 09, 2025 22:08
 * Description:
 *
 * Copyright (c) 2025 Pin Loon Lee (pllee4)
 */

#include "algorithm/generic/disjoint_union_set/disjoint_union_set.hpp"

#include "gtest/gtest.h"

using namespace pllee4::generic;

TEST(DisjointUnionSetsTest, InitialState) {
  DisjointUnionSets dsu(5);

  for (auto i = 0; i < 5; ++i) {
    EXPECT_EQ(dsu.Find(i), i)
        << "Element " << i << " should be its own parent initially";
  }
}

TEST(DisjointUnionSetsTest, UnionTwoElements) {
  DisjointUnionSets dsu(5);

  dsu.UnionSets(0, 1);

  EXPECT_EQ(dsu.Find(0), dsu.Find(1))
      << "Elements 0 and 1 should have the same root";
}

TEST(DisjointUnionSetsTest, UnionChain) {
  DisjointUnionSets dsu(5);

  dsu.UnionSets(0, 1);
  dsu.UnionSets(1, 2);
  dsu.UnionSets(2, 3);

  const auto root = dsu.Find(0);
  EXPECT_EQ(dsu.Find(1), root);
  EXPECT_EQ(dsu.Find(2), root);
  EXPECT_EQ(dsu.Find(3), root);
  EXPECT_NE(dsu.Find(4), root) << "Element 4 should not be in the same set";
}

TEST(DisjointUnionSetsTest, PathCompression) {
  DisjointUnionSets dsu(5);

  dsu.UnionSets(0, 1);
  dsu.UnionSets(1, 2);
  dsu.UnionSets(2, 3);

  // After first Find, path should be compressed
  const auto root = dsu.Find(3);
  EXPECT_EQ(dsu.Find(0), root);

  // All elements should now point directly to root (path compression)
  EXPECT_EQ(dsu.Find(1), root);
  EXPECT_EQ(dsu.Find(2), root);
}

TEST(DisjointUnionSetsTest, UnionByRank) {
  DisjointUnionSets dsu(10);

  // Create two trees of different ranks
  dsu.UnionSets(0, 1);
  dsu.UnionSets(2, 3);
  dsu.UnionSets(3, 4);

  EXPECT_NE(dsu.Find(0), dsu.Find(2));

  // Union the two trees
  dsu.UnionSets(0, 2);

  // The tree with higher rank should become the root
  EXPECT_EQ(dsu.Find(4), dsu.Find(0));
}

TEST(DisjointUnionSetsTest, UnionLowerRankToHigherRank) {
  DisjointUnionSets dsu(6);

  dsu.UnionSets(0, 1);
  dsu.UnionSets(2, 3);
  dsu.UnionSets(0, 2);
  
  dsu.UnionSets(4, 0);
  
  EXPECT_EQ(dsu.Find(4), dsu.Find(0));
}

TEST(DisjointUnionSetsTest, SelfUnion) {
  DisjointUnionSets dsu(5);

  dsu.UnionSets(2, 2);

  EXPECT_EQ(dsu.Find(2), 2) << "Self-union should not change the parent";
}

// Test: Multiple disjoint sets
TEST(DisjointUnionSetsTest, MultipleDisjointSets) {
  DisjointUnionSets dsu(10);

  dsu.UnionSets(0, 1);
  dsu.UnionSets(2, 3);
  dsu.UnionSets(4, 5);

  // Check that different sets have different roots
  EXPECT_EQ(dsu.Find(0), dsu.Find(1));
  EXPECT_EQ(dsu.Find(2), dsu.Find(3));
  EXPECT_EQ(dsu.Find(4), dsu.Find(5));

  EXPECT_NE(dsu.Find(0), dsu.Find(2));
  EXPECT_NE(dsu.Find(2), dsu.Find(4));
  EXPECT_NE(dsu.Find(0), dsu.Find(4));
}

TEST(DisjointUnionSetsTest, IdempotentUnions) {
  DisjointUnionSets dsu(5);

  dsu.UnionSets(0, 1);
  const auto root1 = dsu.Find(0);

  dsu.UnionSets(0, 1);
  dsu.UnionSets(1, 0);

  EXPECT_EQ(dsu.Find(0), root1)
      << "Repeated unions should not change the structure";
}

TEST(DisjointUnionSetsTest, LargeSet) {
  DisjointUnionSets dsu(1000);

  // Union all elements into one set
  for (auto i = 1; i < 1000; ++i) {
    dsu.UnionSets(0, i);
  }

  const auto root = dsu.Find(0);
  for (int i = 1; i < 1000; ++i) {
    EXPECT_EQ(dsu.Find(i), root) << "All elements should be in the same set";
  }
}

TEST(DisjointUnionSetsTest, AlternatingUnions) {
  DisjointUnionSets dsu(8);

  dsu.UnionSets(0, 2);
  dsu.UnionSets(1, 3);
  dsu.UnionSets(4, 6);
  dsu.UnionSets(5, 7);

  dsu.UnionSets(0, 4);
  dsu.UnionSets(1, 5);

  dsu.UnionSets(0, 1);

  // All should be in the same set now
  const auto root = dsu.Find(0);
  for (int i = 1; i < 8; ++i) {
    EXPECT_EQ(dsu.Find(i), root);
  }
}