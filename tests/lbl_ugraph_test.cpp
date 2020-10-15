///////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief Testing module for EdgeLblUGraph class.
///
/// © Sergey Shershakov 2020.
///
/// This code is for educational purposes of the course "Algorithms and Data 
/// Structures" provided by the School of Software Engineering of the Faculty 
/// of Computer Science at the Higher School of Economics.
///
/// When altering code, a copyright line must be preserved.
///////////////////////////////////////////////////////////////////////////////


#include <gtest/gtest.h>

#include "ugraph/lbl_ugraph.hpp"


TEST(EdgeLblUGraph, simplest)
{
}


// Graph with integers as node ids and edge labels.
typedef EdgeLblUGraph<int, int> IntIntGraph;

TEST(EdgeLblUGraph, simpleCreation)
{
    IntIntGraph g;
}

// Tests iterating edges w/ self-loops using cutom EdgeIterators.
TEST(EdgeLblUGraph, getEdgeLabel)
{
    IntIntGraph g;
    g.addLblEdge(1, 2, 10);
    g.addLblEdge(1, 3, 20);
    g.addEdge(1, 4);
    g.addLblEdge(2, 4, 40);

    EXPECT_EQ(4, g.getVerticesNum());
    EXPECT_EQ(4, g.getEdgesNum());

    int lbl;
    EXPECT_TRUE(g.getLabel(1, 2, lbl));
    EXPECT_EQ(10, lbl);

    EXPECT_TRUE(g.getLabel(1, 3, lbl));
    EXPECT_EQ(20, lbl);

    EXPECT_FALSE(g.getLabel(1, 4, lbl));

    EXPECT_TRUE(g.getLabel(2, 4, lbl));
    EXPECT_EQ(40, lbl);

    EXPECT_TRUE(g.getLabel(4, 2, lbl)); // same as {2, 4}
    EXPECT_EQ(40, lbl);
}


