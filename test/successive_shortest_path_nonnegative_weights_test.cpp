//============================================================================
// Copyright 2013 University of Warsaw.
// Authors: Piotr Wygocki 
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#include <boost/graph/successive_shortest_path_nonnegative_weights.hpp>
#include <boost/graph/find_flow_cost.hpp>
#include <boost/core/lightweight_test.hpp>
#include "min_cost_max_flow_utils.hpp"

void path_augmentation_def_test()
{
    boost::SampleGraph::vertex_descriptor s,t;
    boost::SampleGraph::Graph g; 
    boost::SampleGraph::getSampleGraph(g, s, t);
    boost::successive_shortest_path_nonnegative_weights(g, s, t);
    long cost = boost::find_flow_cost(g);
    BOOST_TEST_EQ(cost, 29);
}

void path_augmentation_def_test2()
{
    boost::SampleGraph::vertex_descriptor s,t;
    boost::SampleGraph::Graph g; 
    boost::SampleGraph::getSampleGraph2(g, s, t);
    boost::successive_shortest_path_nonnegative_weights(g, s, t);
    long cost = boost::find_flow_cost(g);
    BOOST_TEST_EQ(cost, 7);
}

void path_augmentation_test()
{
    boost::SampleGraph::vertex_descriptor s,t;
    typedef boost::SampleGraph::Graph Graph;
    Graph g;
    boost::SampleGraph::getSampleGraph(g, s, t);

    std::size_t N = boost::num_vertices(g);
    std::vector<long> dist(N);
    std::vector<long> dist_prev(N);
    typedef boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
    std::vector<edge_descriptor> pred(N);

    boost::property_map<Graph, boost::vertex_index_t>::const_type
      idx = get(boost::vertex_index, g);

    boost::successive_shortest_path_nonnegative_weights(
        g, s, t,
        boost::graph::keywords::_distance_map =
        boost::make_iterator_property_map(dist.begin(), idx),
        boost::graph::keywords::_predecessor_map =
        boost::make_iterator_property_map(pred.begin(), idx),
        boost::graph::keywords::_distance_map2 =
        boost::make_iterator_property_map(dist_prev.begin(), idx),
        boost::graph::keywords::_vertex_index_map = idx
    );

    long cost = boost::find_flow_cost(g);
    BOOST_TEST_EQ(cost, 29);
}

int main()
{
    path_augmentation_def_test();
    path_augmentation_def_test2();
    path_augmentation_test();
    return boost::report_errors();
}

