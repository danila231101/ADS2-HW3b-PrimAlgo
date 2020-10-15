///////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief Testing module for algoritms for undirected graphs.
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

#include "ugraph/ugraph_algos.hpp"
#include "grviz/ugraph_dotwriter.hpp"

// TODO: set the GV_OUT_DIR macros to the path in your local environment!
#define GV_OUT_DIR "f:/temp/2020/20200922/gv/"


TEST(UgraphAlgos, simplest)
{
}


// Graph with integers as node ids and edge labels.
typedef EdgeLblUGraph<int, int> IntIntGraph;
typedef EdgeLblUGraph<char, int> CharIntGraph;
typedef std::set<typename CharIntGraph::Edge> CharIntGraphEdgesSet;


typedef EdgeLblUGraphDotWriter<char, int>::Type CharIntGraphDW;


CharIntGraph makeGraphFromEdges(const CharIntGraph& origG,
                                const CharIntGraphEdgesSet& edges)
{
    CharIntGraph mst;
    for(auto edge : edges)
    {
        int lbl;
        origG.getLabel(edge.first, edge.second, lbl);
        mst.addLblEdge(edge.first, edge.second, lbl);
    }

    return mst;
}

TEST(UgraphAlgos, mstPrim1)
{
    // Creates a graph
    CharIntGraph g;
    g.addLblEdge('a', 'b', 4);
    g.addLblEdge('b', 'c', 8);
    g.addLblEdge('b', 'h', 11);
    g.addLblEdge('c', 'd', 7);
    g.addLblEdge('c', 'i', 2);
    g.addLblEdge('c', 'f', 4);
    g.addLblEdge('d', 'e', 9);
    g.addLblEdge('d', 'f', 14);
    g.addLblEdge('e', 'f', 10);
    g.addLblEdge('f', 'g', 2);
    g.addLblEdge('g', 'h', 1);
    g.addLblEdge('g', 'i', 6);
    g.addLblEdge('h', 'a', 8);
    g.addLblEdge('h', 'i', 7);


    // output it first as a pic
    CharIntGraphDW dw;   // dotwriter
    dw.write(GV_OUT_DIR "clrs_graph.gv", g, "CLRS Graph");


    // Prim
    CharIntGraphEdgesSet mstEdges = findMSTPrim(g);
    CharIntGraph mst = makeGraphFromEdges(g, mstEdges);
    dw.write(GV_OUT_DIR "clrs_graph_mst.gv", mst, "MST for CLRS Graph (Prim)");

}

TEST(UgraphAlgos, mstKruskal1)
{
    // TODO:
}

