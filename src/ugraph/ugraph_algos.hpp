////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief      Contains implementations of some algorithms for undirected graph.
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


#ifndef UGRAPH_ALGOS_HPP
#define UGRAPH_ALGOS_HPP

#include <set>
#include <climits>
#include <map>

#include "lbl_ugraph.hpp"

template <typename Vertex, typename EdgeLbl>
class PriorityQueue {
public:
    PriorityQueue(){}

    void insert(const Vertex& vert, const EdgeLbl& cost) //that inserts a fresh vertex together with an associated cost
    {
        _queueL.insert(std::make_pair(cost, vert));
        _queueV.insert(std::make_pair(vert, cost));
    }

    void remove(const Vertex& vert) //that removes a given vertex from the set of unvisited vertices
    {
        auto it = _queueV.find(vert);
        if (it != _queueV.end())
        {
            _queueL.erase({it->second, vert});
            _queueV.erase(it);
        }
    }

    void set(const Vertex& vert, const EdgeLbl& cost) //that changes the cost for a given vertex to the value of a passed parameter
    {
        insert(vert, cost);
        remove(vert);
    }

    Vertex getMin() const //that returns a vertex with the current minimal cost among all the vertices
    {
        return (_queueL.begin()->second);
    }

    EdgeLbl getCost(const Vertex& vert) const//that returns an associated cost for a given vertex.
    {
        auto it =_queueV.find(vert);
        return (it->second);
    }

    bool isEmpty() //that returns true when the setOfUnvisitedNodes is empty.
    {
        return (_queueV.empty() && _queueL.empty());
    }

    ~PriorityQueue(){}
private:
    std::multimap<Vertex, EdgeLbl> _queueV;
    std::set<std::pair<EdgeLbl, Vertex>> _queueL;
};

/// Finds a MST for the given graph \a g using Prim's algorithm.
template<typename Vertex, typename EdgeLbl>
std::set<typename EdgeLblUGraph<Vertex, EdgeLbl>::Edge>
findMSTPrim(const EdgeLblUGraph<Vertex, EdgeLbl>& g)
{
    std::set<typename EdgeLblUGraph<Vertex, EdgeLbl>::Edge> finalResult;
    std::map<Vertex,Vertex> MST;
    typename std::map<Vertex,Vertex>::iterator iterMST;
    std::set<Vertex> setOfVisitedNodes;
    std::map<Vertex,EdgeLbl> cost;
    typename EdgeLblUGraph<Vertex, EdgeLbl>::VertexIterPair vert = g.getVertices();
    PriorityQueue<Vertex,EdgeLbl> queueOfUnvisitedNodes;

    for(typename EdgeLblUGraph<Vertex, EdgeLbl>::VertexIter itVert = vert.first; itVert != vert.second; ++itVert)
    {
        cost.insert(std::make_pair(*itVert, INT_MAX));
    }

    typename EdgeLblUGraph<Vertex,EdgeLbl>::EdgeIter iterOfEdge = g.getEdges().first;
    typename std::map<Vertex,EdgeLbl>::iterator iterCost = cost.begin();
    iterCost = cost.find(iterOfEdge->first);
    iterCost->second = 0;
    queueOfUnvisitedNodes.insert(iterOfEdge->first, 0);

    while(!queueOfUnvisitedNodes.isEmpty())
    {
        Vertex minVert = queueOfUnvisitedNodes.getMin();
        setOfVisitedNodes.insert(minVert);
        typename EdgeLblUGraph<Vertex,EdgeLbl>::AdjListCIterPair range = g.getAdjEdges(minVert);
        typename EdgeLblUGraph<Vertex,EdgeLbl>::AdjListCIter iterAdjVert;
        queueOfUnvisitedNodes.remove(minVert);

        for(iterAdjVert = range.first; iterAdjVert != range.second; ++iterAdjVert)
        {
            Vertex adjVert = iterAdjVert->second;
            EdgeLbl AdjCost;
            g.getLabel(minVert, adjVert, AdjCost);

            if((setOfVisitedNodes.find(adjVert) == setOfVisitedNodes.end()) && (cost.find(adjVert)->second > AdjCost))
            {
                cost.find(adjVert)->second = AdjCost;
                queueOfUnvisitedNodes.insert(adjVert, cost.find(adjVert)->second);
                iterMST = MST.find(adjVert);
                if(iterMST == MST.end())
                    MST.insert(std::make_pair(adjVert, minVert));
                else
                    iterMST->second = minVert;
            }
        }
    }
    for(iterMST = MST.begin(); iterMST != MST.end(); ++iterMST)
    {
        finalResult.insert({iterMST->first, iterMST->second});
    }
    return finalResult;
}


#endif // UGRAPH_ALGOS_HPP
