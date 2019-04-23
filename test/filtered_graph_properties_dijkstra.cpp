// (c) Copyright Juergen Hunold 2012
// Use, modification and distribution is subject to the Boost Software
// License, Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/property_map/property_map.hpp>

namespace boost {

    enum edge_info_t { edge_info = 114 };

    BOOST_INSTALL_PROPERTY( edge, info );
}

template< typename EdgeInfo,
          typename Directed >
class Graph
{
public:
    typedef boost::property< boost::edge_info_t, EdgeInfo > tEdge_property;

    typedef boost::adjacency_list< boost::setS,
                                   boost::vecS,
                                   Directed,
                                   boost::no_property,
                                   tEdge_property > tGraph;

    typedef typename boost::graph_traits< tGraph >::vertex_descriptor tNode;
    typedef typename boost::graph_traits< tGraph >::edge_descriptor   tEdge;

protected:

    tGraph          m_Graph;
};

class DataEdge;

class UndirectedGraph
    : public Graph< DataEdge*,
                    boost::undirectedS >
{
public:

    template< class Evaluator, class Filter >
    void        dijkstra( Evaluator const&,
                          Filter const& ) const;
};

template< typename Graph, typename Derived >
struct Evaluator : public boost::put_get_helper< int, Derived >
{
    typedef int value_type;
    typedef typename Graph::tEdge key_type;
    typedef int reference;
    typedef boost::readable_property_map_tag category;

    explicit Evaluator( Graph const* pGraph );
};

template< typename Graph, typename Derived >
Evaluator< Graph, Derived >::Evaluator( Graph const* pGraph )
{
}

template< typename Graph >
class LengthEvaluator : public Evaluator< Graph, LengthEvaluator<Graph> >
{
    typedef Evaluator< Graph, LengthEvaluator<Graph> > Base;

public: 
    explicit LengthEvaluator( Graph const* pGraph );

    typedef typename Evaluator<Graph, LengthEvaluator<Graph> >::reference reference;
    typedef typename Evaluator<Graph, LengthEvaluator<Graph> >::key_type key_type;

    virtual reference operator[]( key_type const& edge ) const;
};

template< typename Graph >
LengthEvaluator< Graph >::LengthEvaluator( Graph const* pGraph ) : Base( pGraph )
{
}

int test_dummy;

template< typename Graph >
typename LengthEvaluator< Graph >::reference
LengthEvaluator< Graph >::operator[]( key_type const& edge ) const
{
    return test_dummy;
}

template< class Graph >
struct EdgeFilter
{
    typedef typename Graph::tEdge key_type;

    EdgeFilter();

    explicit EdgeFilter( Graph const*);

    bool    operator()( key_type const& ) const;

private:
    const Graph*    m_pGraph;
};

template< class Graph >
EdgeFilter< Graph >::EdgeFilter()
{
}

template< class Graph >
bool
EdgeFilter< Graph >::operator()( key_type const& ) const
{
    return true;
}

template< class Evaluator, class Filter >
void
UndirectedGraph::dijkstra( Evaluator const& rEvaluator,
                           Filter const& rFilter ) const
{
    tNode nodeSource = vertex(0, m_Graph);

    std::vector< tNode > predecessors( num_vertices(m_Graph) );

    boost::filtered_graph< tGraph, Filter > filteredGraph( m_Graph, rFilter );

    dijkstra_shortest_paths(
        filteredGraph, nodeSource,
        boost::graph::keywords::_predecessor_map = make_iterator_property_map(
            predecessors.begin(), boost::typed_identity_property_map< std::size_t >(),
            boost::graph_traits< tGraph >::null_vertex()
        ),
        boost::graph::keywords::_weight_map = rEvaluator
    );
}

// explicit instantiation
template void UndirectedGraph::dijkstra( LengthEvaluator<UndirectedGraph> const&,
                                         EdgeFilter<UndirectedGraph> const& ) const;

int main(int, char**) {return 0;} // Tests above will fail to compile if anything is broken
