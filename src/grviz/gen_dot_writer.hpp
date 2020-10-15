////////////////////////////////////////////////////////////////////////////////
/// \file
/// \brief     LDOPA Graph Library
/// \author    Sergey Shershakov
/// \version   0.1.0
/// \date      31.07.2018
/// \copyright (c) xidv.ru 2014—2020.
///            This source is for internal use only — Restricted Distribution.
///            All rights reserved.
///
/// Generic class template for output graph-based models as a DOT-files.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef XI_LDOPA_GRAPHS_GRVIZ_GEN_DOT_WRITER_H_
#define XI_LDOPA_GRAPHS_GRVIZ_GEN_DOT_WRITER_H_

// std
#include <list>
#include <string>
#include <fstream>

namespace xi { namespace ldopa { namespace graph {

/*! ****************************************************************************
 *  \brief Default visitor class.
 *
 *  \tparam TGraph typename for a graph-based model to output.
 ******************************************************************************/
template <typename TGraph>
struct DefaultDotVisitor
{
    //----<Types>----
    typedef std::pair<std::string, std::string> StrStrPair;
    
    /** \brief List extension. */
    class ParamValueList : public std::list < StrStrPair >
    {
    public:
        /** \brief Appends a pair of param-value to the end of the list. */
        inline void append(const std::string& par, const std::string& val)
        {
            push_back(std::make_pair(par, val));
        }
    }; // class ParamValueList
    
    typedef typename ParamValueList::const_iterator ParamValueListCIter;

    /** \brief Define a sort of a graph. */
    enum class Sort { graph, digraph };

    //----<Constructors>----
    DefaultDotVisitor(Sort tSort = Sort::digraph)
        : sort(tSort)
    {
    }

    //----<Helper methods>----

    /// Creates a HEX representation of the given Uint value \a v prefixed by 'x'.
    static std::string makeUintHexId(unsigned int v)
    {
        // https://stackoverflow.com/questions/1042940/writing-directly-to-stdstring-internal-buffers
        char addrBuf[11];
        sprintf(addrBuf, "x%x", v);
        
        return std::string(addrBuf);
    }

    /// Makes a formatted string correposing to the list of param-values.
    static std::string makeParamValueStr(const ParamValueList& parList)
    {
        ParamValueListCIter it = parList.begin();

        // check if no param at all
        if (it == parList.end())
            return "";

        std::string res = "[";
        bool first = true;

        for (; it != parList.end(); ++it)
        {
            if (!first)
                res += ',';
            else
                first = false;

            res += it->first;
            res += '=';
            res += it->second;            
        }

        res += ']';

        return res;
    }

    /// Escapes a strings special symbols and enclose the result into dblquotes.
    static std::string makeEscapedString(const std::string& s)
    {
        std::string res;
        res.reserve(s.length() + 2);            // в минимальной версии
            
        res += '\"';
        for (char c : s) 
        {
            if (c == '"')           // заменяем кавычку на посл. \"
            {
                res += "\\\"";      
                continue;
            }

            if (c == '\\')          // заменяем бэкслеш на два бекслеша
            {
                res += "\\\\";      
                continue;
            }

            // все остальное просто копируем
            res += c;
        }
        res += '\"';
        
        return res;
    }

    //----<Concept methods>----
    void outputHeader(std::ostream& str, const TGraph& gr, const char* grLbl)
    {
        if(sort == Sort::digraph)
            str << "digraph G {\n";
        else
            str << "graph G {\n";

        // если есть метка графа, добавим:
        if (grLbl)
            str << "    label=\"" << grLbl << "\";\n";
            //str << "    node [width=0.5,fontcolor=white,style=filled];\n";

        str << "    node [width=0.5];\n";
    }

    void outputTail(std::ostream& str, const TGraph& gr)
    {
        str << "}\n";
    }


    //----<Fields>----
    Sort sort;
}; // class DefaultDotVisitor

//=============================================================================


/*! ****************************************************************************
 *  \brief Generic DOT-writer.
 *
 *  \tparam TGraph typename for a graph-based model to output.
 *  \tparam TGraphVisitor traits class for individual visitors of graph elements.
 *
 ******************************************************************************/
template <typename TGraph, typename TGraphVisitor = DefaultDotVisitor<TGraph> >
class GenDotWriter {
public:
    
    // Constructor.
    GenDotWriter(const TGraphVisitor& gv = TGraphVisitor())
        : _gv(gv)
    {
    }
public:

    /// Writes a dump of the given model \a gr to a file with the name \a fn,
    /// having the label \a grLbl and using the given visitor object \a gv.
    void write(const std::string& fn, const TGraph& gr, const char* grLbl = nullptr)
    {
        std::ofstream dfile(fn.c_str());
        if (!dfile.is_open())
            throw std::invalid_argument("Can't open dump file for GraphViz");

        // заголовок
        outputHeader(dfile, gr, grLbl);

        // тело
        outputBody(dfile, gr);

        // хвост
        outputTail(dfile, gr);

        dfile.flush();
    }

protected:
    /// Outputs the main part (vertices and edges) of the graph to the output.
    inline void outputBody(std::ostream& str, const TGraph& gr)
    {
        _gv.outputBody(str, gr);
    }

    /// Outputs the header of the graph to the output.
    inline void outputHeader(std::ostream& str, const TGraph& gr, const char* grLbl = nullptr)
    {
        _gv.outputHeader(str, gr, grLbl);
    }

    /// Outputs the tail of the graph to the output.
    inline void outputTail(std::ostream& str, const TGraph& gr)
    {
        _gv.outputTail(str, gr);
    }

protected:
    
    /** \brief Graph Visitor object. */
    TGraphVisitor _gv;
}; // class GenDotWriter 


//
}}} // namespace xi::ldopa::graph {

#endif // XI_LDOPA_GRAPHS_GRVIZ_GENDOTWRITER_H_
