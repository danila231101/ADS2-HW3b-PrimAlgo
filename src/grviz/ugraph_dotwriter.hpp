////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief      DOT-writer (1) for labeled graphs.
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

#ifndef UGRAPH_HPP_
#define UGRAPH_HPP_

#include <sstream>

#include "gen_dot_writer.hpp"
#include "../ugraph/lbl_ugraph.hpp"


/** \brief DOT-writer for EvLogTSWithFreqs models. */
template <typename Vertex, typename EdgeLbl>
struct EdgeLblUGraphDotVisitor :
    public xi::ldopa::graph::DefaultDotVisitor < EdgeLblUGraph<Vertex, EdgeLbl> >
{
    typedef EdgeLblUGraph<Vertex, EdgeLbl> Graph;
    typedef xi::ldopa::graph::DefaultDotVisitor
        < EdgeLblUGraph<Vertex, EdgeLbl> > Base;


    EdgeLblUGraphDotVisitor() : Base(Base::Sort::graph) {}

    void outputBody(std::ostream& str, const Graph& g)
    {
        // enumerates all vertices
        typename Graph::VertexIterPair vs = g.getVertices();
        for(typename Graph::VertexIter it = vs.first; it != vs.second; ++it)
        {
            str << *it << "\n";     // see code below for example how to add params
        }

        // enumerates all edges
        typename Graph::EdgeIterPair es = g.getEdges();
        for(typename Graph::EdgeIter it = es.first; it != es.second; ++it)
        {
            typename Base::ParamValueList pars;    // edge attributes
            EdgeLbl lbl;
            if(g.getLabel(it->first, it->second, lbl))
            {
                // use string stream to convert an arbitrary type EdgeLbl to a string
                std::stringstream ss;
                ss << lbl;
                pars.append("label", Base::makeEscapedString(ss.str()));
            }

            str << it->first << " -- " << it->second
                << " " << Base::makeParamValueStr(pars)  << "\n";
        }
    }
};



/*! ****************************************************************************
 *  \brief Metafunction for typedefing templates.
 *
 *  Templates param are the same as for EdgeLblUGraph datatype.
 *  \see https://stackoverflow.com/questions/26151/template-typedefs-whats-your-work-around
 *
 *  Usage: EdgeLblUGraphDotWriter<T1, T2>::Type...
 ******************************************************************************/
template <typename Vertex, typename EdgeLbl>
struct EdgeLblUGraphDotWriter
{
    typedef xi::ldopa::graph::GenDotWriter < EdgeLblUGraph<Vertex, EdgeLbl>,
                                             EdgeLblUGraphDotVisitor<Vertex, EdgeLbl> >
            Type;
};


#endif //
