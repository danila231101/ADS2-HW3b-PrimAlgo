////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief      Contains declarations of the types for undirected graphs.
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


#ifndef UGRAPH_HPP
#define UGRAPH_HPP


#include <set>
#include <map>
//#include <cstddef> // size_t



/*! ****************************************************************************
 *  \brief The UGraph class represents a undirected graph.
 *
 *  \tparam Vertex represents a type for vertices. Will be used as a node ID by
 *  copy, so choose it cleverly. Must be comparable.
 ******************************************************************************/
template <typename Vertex>
class UGraph {
public:
    // type definitions

    typedef std::pair<Vertex, Vertex> Edge;

    /// Set of vertices.
    typedef std::set<Vertex> VerticesSet;

    /// Iterator type for vertices.
    typedef typename VerticesSet::iterator VertexIter;

    /// Pair of vertex iterators.
    typedef std::pair<VertexIter, VertexIter> VertexIterPair;

    // TODO: there need to define const iterator types.

    // Edge Iterators must be defined customly!


    /// \brief Adjacency list datatype, for storing adjacent vertices.
    ///
    /// Consists of exactly twice more elements than the number of edges in a
    /// graph (think of why).
    typedef std::multimap<Vertex, Vertex> AdjList;
    typedef typename AdjList::iterator AdjListIter;
    typedef typename AdjList::const_iterator AdjListCIter;
    typedef std::pair<AdjListCIter, AdjListCIter> AdjListCIterPair;


    /// \brief Custom definition of Edge Iterators.
    ///
    /// Iterator is an any object that behaves like an iterator. So, we need
    /// to implement all necessary features specific to the forward iterator.
    ///
    /// This class iterates a given range of edges in a multimap, considering
    /// only non-repeating edges.
    class EdgeIter {
    public:
        // Typically expected types
//        typedef Edge                        value_type;
//        typedef Edge&                       reference;
//        typedef Edge*                       pointer;
        typedef typename AdjListCIter::value_type  value_type;
        typedef typename AdjListCIter::reference   reference;
        typedef typename AdjListCIter::pointer     pointer;


        typedef std::forward_iterator_tag   iterator_category;
        typedef long                        difference_type;

        typedef EdgeIter Self;              ///< For convenience.
    public:
        // Minimum set of expected operations
        EdgeIter(AdjListCIter cur, AdjListCIter end)
            : _cur(cur), _end(end)
        {
            goUntilNextValid();
        }

        /// Prefix version of ++: iterates first until the end.
        Self operator++(int junk)
        {
            ++_cur;
            goUntilNextValid();

            return *this;
        }

        // Postfix version of ++: creates a copy of this.
        Self operator++()
        {
            Self curCopy = *this;

            ++_cur;
            goUntilNextValid();

            return curCopy;
        }

        // this works perfectly and  could be cool, but breaks a bit general
        // (expected!) logic of iterators
        //Edge operator*() { return Edge(_cur->first, _cur->second); }

        reference operator*() { return *_cur; }
        pointer operator->() { return &*_cur; } // seems a bit weird, need to clarify


        bool operator==(const Self& rhv)
        {
            return (_cur == rhv._cur) && (_end == rhv._end);
        }

        bool operator!=(const Self& rhv)
        {
            return !(*this == rhv);
        }

    protected:
        /// Iterates the underlying mmap until finds a valid pair or reaches
        /// the end.
        void goUntilNextValid()
        {
            bool duplicate = false;             // indicates duplicates existance
            while (_cur != _end)
            {
                // self-loop case
                if(_cur->first == _cur->second)
                {
                    if(duplicate)
                    {
                        duplicate = false;
                        return;                 // valid second instance of s SL
                    }
                    duplicate = true;
                    ++_cur;
                    continue;
                }

                // “normal” case
                if(_cur->first < _cur->second)
                    return;                     // valid first part of edge

                // “collinear” case
                // _cur->first > _cur->second
                ++_cur;
                continue;
            }

            return;                             // endge empty
        }

    protected:
        AdjListCIter _cur;                   ///< Current iterator in the mmap.
        AdjListCIter _end;                   ///< End iterator in the mmap.
    }; // class EdgeIter



    /// Pair of edge iterators.
    typedef std::pair<EdgeIter, EdgeIter> EdgeIterPair;


public:
    // Helpers

    /// Creates an edge as a pair of provided vertices s.t. the “smaller” node
    /// goes first and the “greater” node goes second.
    static Edge makeNormalizedEdge(Vertex s, Vertex d)
    {
        if(s < d)
            return {s, d};

        return {d, s};
    }


    // Graph structure modifying methods.

    /// Adds into this graph a new vertex \a v and returns it by value.
    Vertex addVertex(Vertex v)
    {
        _vertices.insert(v);
        return v;
    }

    /// \brief Adds into this graph a new edge made of two vertices.
    /// \return An object of type Edge with normalized positions of vertices.
    ///
    /// If a correponding edge {s, d} or equivalent {d, s} has been added earlier,
    /// do nothing else as just return an edge object.
    Edge addEdge(Vertex s, Vertex d)
    {
        if(!isEdgeExists(s, d))      // need to add
        {
            // add two collinear edges
            _edges.insert({s, d});
            _edges.insert({d, s});

            // add edges vertices too
            addVertex(s);
            addVertex(d);
        }
        //Edge e(s, d);
        Edge e = makeNormalizedEdge(s, d);

        return e;
    }

    /// Method determines whether an edge {s, d} exists in this graph.
    ///
    /// \return true if the edge exists, false otherwise.
    ///
    /// Graph guarantees that is a vertex {a, b} exists then its counterpart
    /// {b, a} exists too.
    bool isEdgeExists(Vertex s, Vertex d) const
    {
        auto itlow = _edges.lower_bound(s);
        auto itup = _edges.upper_bound(s);
        for (auto it = itlow; it != itup; ++it)
        {
            if(it->second == d)
                return true;
        }

        return false;
    }

    bool isVertexExists(Vertex v) const
    {
        return (_vertices.find(v) != _vertices.end());
    }



public:
    // setters/getters
    size_t getVerticesNum() const { return _vertices.size(); }
    size_t getEdgesNum() const { return _edges.size() / 2; }


    /// Provides a collection of vertices as a semirange (pair of iterators).
    VertexIterPair getVertices() const
    {
        return {_vertices.begin(), _vertices.end()};
    }

    EdgeIterPair getEdges() const
    {
        EdgeIter beg(_edges.begin(), _edges.end());
        EdgeIter end(_edges.end(), _edges.end());

        return {beg, end};
    }

    /// Return a range of edges that are direct neighbours of the given
    /// vertex \a v.
    AdjListCIterPair getAdjEdges(Vertex v) const
    {
        return { _edges.lower_bound(v), _edges.upper_bound(v) };
    }


protected:
    VerticesSet _vertices;      ///< Set of vertices.
    AdjList _edges;             ///< Adjacency list for representing edges.
}; // class UGraph




#endif // UGRAPH_HPP
