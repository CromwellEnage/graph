//============================================================================
// Copyright 2013 University of Warsaw.
// Authors: Piotr Wygocki 
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>
#include <boost/core/lightweight_test.hpp>
#include "min_cost_max_flow_utils.hpp"

typedef boost::adjacency_list_traits<boost::vecS,boost::vecS,boost::directedS> traits;
struct edge_t {
  double capacity;
  float cost;
  float residual_capacity;
  traits::edge_descriptor reversed_edge;
};
struct node_t {
  traits::edge_descriptor predecessor;
  int dist;
  int dist_prev;
  boost::vertex_index_t id;
  boost::default_color_type color;
};
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, node_t, edge_t > Graph;

int main()
{
  Graph g;
  traits::vertex_descriptor s,t;

  boost::property_map<Graph,double edge_t::* >::type capacity = get(&edge_t::capacity, g);
  boost::property_map<Graph,float edge_t::* >::type cost = get(&edge_t::cost, g);
  boost::property_map<Graph,float edge_t::* >::type residual_capacity = get(&edge_t::residual_capacity, g);
  boost::property_map<Graph,traits::edge_descriptor edge_t::* >::type rev = get(&edge_t::reversed_edge, g);
  boost::property_map<Graph,traits::edge_descriptor node_t::* >::type pred = get(&node_t::predecessor, g);
  boost::property_map<Graph,boost::default_color_type node_t::* >::type col = get(&node_t::color, g);

  boost::SampleGraph::getSampleGraph(g,s,t,capacity,residual_capacity,cost,rev);

  // The "named parameter version" (producing errors)
  // I chose to show the error with edmonds_karp_max_flow().
  int flow_value = edmonds_karp_max_flow(
    g,
    s,
    t,
    boost::graph::keywords::_capacity_map = capacity,
    boost::graph::keywords::_residual_capacity_map = residual_capacity,
    boost::graph::keywords::_reverse_edge_map = rev,
    boost::graph::keywords::_color_map = col,
    boost::graph::keywords::_predecessor_map = pred
  );

  BOOST_TEST_EQ(flow_value,4);
  return boost::report_errors();
}
