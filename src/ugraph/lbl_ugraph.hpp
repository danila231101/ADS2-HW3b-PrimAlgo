////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief      Contains declarations of the types for labeled undirected graphs.
/// \author     Sergey Shershakov
/// \version    0.1.0
/// \date       21.09.2020
/// \copyright  © Sergey Shershakov 2020.
///             This code is for educational purposes of the course "Algorithms
///             and Data Structures" provided by the Faculty of Computer Science
///             at the Higher School of Economics.
///
/// When altering code, a copyright line must be preserved.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef LBL_UGRAPH_HPP
#define LBL_UGRAPH_HPP

#include "ugraph.hpp"

#include <map>

/*! ****************************************************************************
 *  \brief The EdgeLblUGraph class represents a undirected graph with labels on
 *  edges..
 *
 *  \tparam Vertex represents a type for vertices. See requirements for UGraph.
 *  \tparam EdgeLbl represents a type for edge labeling.
 ******************************************************************************/
template <typename Vertex, typename EdgeLbl>
class EdgeLblUGraph
        : public UGraph<Vertex>
{
public:
    // Aliases
    typedef UGraph<Vertex> Base;
    typedef typename Base::Edge Edge;

    // Local datatype definitions

    /// Labeling function type for graph edges.
    typedef std::map<typename Base::Edge, EdgeLbl> EdgeLabeling;
    typedef typename EdgeLabeling::const_iterator EdgeLabelingCIter;

public:
    // Graph structure modifying methods.

    /// \brief Adds into this graph a new edge made of two vertices and label it.
    /// \return An object of type Edge with normalized positions of vertices.
    ///
    /// If a correponding edge {s, d} or equivalent {d, s} has been added earlier,
    /// the current call of the function UPDATES the associated label.
    Edge addLblEdge(Vertex s, Vertex d, EdgeLbl lbl)
    {
        Edge e = Base::addEdge(s, d);
        _edgeLabeling.insert({e, lbl});

        return e;
    }

    /// For a given edge \a e tries to find an associated label and returns it
    /// if so.
    ///
    /// \return true if a label for the given edge is associated and \a lbl is
    /// assigned to its value; false otherwise (in this case the value of lbl is
    /// undefined).
    //bool getLabel(const Edge& e, EdgeLbl& lbl) const
    bool getLabel(Vertex s, Vertex d, EdgeLbl& lbl) const
    {
        Edge e = Base::makeNormalizedEdge(s, d);
        EdgeLabelingCIter it = _edgeLabeling.find(e);
        if(it != _edgeLabeling.end())
        {
            lbl = it->second;
            return true;
        }

        return false;
    }

protected:
    EdgeLabeling _edgeLabeling;
};

#endif // UGRAPH_HPP
