////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief      Testing module for algoritms for DOT-writers for labeled graphs.
/// \author     Sergey Shershakov
/// \version    0.1.0
/// \date       23.09.2020
/// \copyright  © Sergey Shershakov 2020.
///             This code is for educational purposes of the course "Algorithms
///             and Data Structures" provided by the Faculty of Computer Science
///             at the Higher School of Economics.
///
/// When altering code, a copyright line must be preserved.
///
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>

#include "ugraph/lbl_ugraph.hpp"
#include "grviz/ugraph_dotwriter.hpp"

#define GV_OUT_DIR "f:/temp/2020/20200922/gv/"

TEST(UGraphDotWriter, simplest)
{
}

typedef EdgeLblUGraph<int, int> IntIntGraph;
typedef EdgeLblUGraphDotWriter<int, int>::Type IntIntGraphDW;

TEST(UGraphDotWriter, simpleGraph)
{
    IntIntGraph g;      // graph
    g.addLblEdge(1, 2, 10);
    g.addLblEdge(1, 3, 20);
    g.addEdge(1, 4);
    g.addLblEdge(2, 4, 40);

    IntIntGraphDW dw;   // dotwriter
    dw.write(GV_OUT_DIR "test1.gv", g, "Test Graph");
}
