// Copyright (C) 2002 Trustees of Indiana University

// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dag_shortest_paths.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/core/lightweight_test.hpp>
#include <boost/limits.hpp>
#include <functional>

using namespace boost;

int main(int, char*[])
{
    typedef adjacency_list<vecS, vecS, directedS, no_property,
        property<edge_weight_t, int> > Graph;

    Graph graph;

    (void)add_vertex(graph);
    (void)add_vertex(graph);
    (void)add_vertex(graph);
    (void)add_vertex(graph);

    Graph::edge_descriptor e;
    
    e = add_edge(0, 1, graph).first;
    put(edge_weight, graph, e, 1);

    e = add_edge(1, 2, graph).first;
    put(edge_weight, graph, e, 1);

    e = add_edge(3, 1, graph).first;
    put(edge_weight, graph, e, 5);

    vector_property_map<int> distance;

    dag_shortest_paths(
      graph, 0,
      boost::graph::keywords::_distance_map = distance,
      boost::graph::keywords::_distance_compare = std::greater<int>(),
      boost::graph::keywords::_distance_inf = (std::numeric_limits<int>::min)(),
      boost::graph::keywords::_distance_zero = 0
    );

    BOOST_TEST(distance[2] == 2);

    return boost::report_errors();
}

