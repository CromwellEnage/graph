//=======================================================================
// Copyright 2002 Indiana University.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#ifndef BOOST_GRAPH_DAG_SHORTEST_PATHS_HPP
#define BOOST_GRAPH_DAG_SHORTEST_PATHS_HPP

#include <boost/graph/topological_sort.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

// single-source shortest paths for a Directed Acyclic Graph (DAG)

namespace boost { namespace graph {

  // Initalize distances and call depth first search
  template <class VertexListGraph, class DijkstraVisitor,
            class DistanceMap, class WeightMap, class ColorMap,
            class PredecessorMap, class Compare, class Combine,
            class DistInf, class DistZero>
  void
  dag_shortest_paths
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     DistanceMap distance, WeightMap weight, ColorMap color,
     PredecessorMap pred, DijkstraVisitor vis, Compare compare,
     Combine combine, DistInf inf, DistZero zero, typename boost::disable_if<
       parameter::are_tagged_arguments<
         DistanceMap, ColorMap, WeightMap, PredecessorMap,
         DijkstraVisitor, Compare, Combine, DistInf, DistZero
       >,
       mpl::true_
     >::type = mpl::true_()
     )
  {
    typedef typename graph_traits<VertexListGraph>::vertex_descriptor Vertex;
    std::vector<Vertex> rev_topo_order;
    rev_topo_order.reserve(num_vertices(g));

    // Call 'depth_first_visit', not 'topological_sort', because we don't
    // want to traverse the entire graph, only vertices reachable from 's',
    // and 'topological_sort' will traverse everything. The logic below
    // is the same as for 'topological_sort', only we call 'depth_first_visit'
    // and 'topological_sort' calls 'depth_first_search'.
    topo_sort_visitor<std::back_insert_iterator<std::vector<Vertex> > >
        topo_visitor(std::back_inserter(rev_topo_order));
    depth_first_visit(g, s, topo_visitor, color);

    typename graph_traits<VertexListGraph>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
      put(distance, *ui, inf);
      put(pred, *ui, *ui);
    }

    put(distance, s, zero);
    vis.discover_vertex(s, g);
    typename std::vector<Vertex>::reverse_iterator i;
    for (i = rev_topo_order.rbegin(); i != rev_topo_order.rend(); ++i) {
      Vertex u = *i;
      vis.examine_vertex(u, g);
      typename graph_traits<VertexListGraph>::out_edge_iterator e, e_end;
      for (boost::tie(e, e_end) = out_edges(u, g); e != e_end; ++e) {
        vis.discover_vertex(target(*e, g), g);
        bool decreased = relax(*e, g, weight, pred, distance, 
                               combine, compare);
        if (decreased)
          vis.edge_relaxed(*e, g);
        else
          vis.edge_not_relaxed(*e, g);
      }
      vis.finish_vertex(u, g);      
    }
  }
}} // namespace boost::graph

namespace boost { namespace graph {

    template <typename Graph, typename Args>
    void dag_shortest_paths(
        const Graph &g, typename graph_traits<Graph>::vertex_descriptor s,
        const Args& arg_pack, typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            Graph
        >::type IndexMap;
        IndexMap v_i_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
        );
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            Graph
        >::type WeightMap;
        WeightMap e_w_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_weight_map,
            g,
            edge_weight
        );
        typedef typename boost::property_traits<WeightMap>::value_type Weight;
        const Weight zero_weight = Weight();
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::distance_map,
            Weight
        > v_d_map_gen(zero_weight);
        typedef typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type DistanceMap;
        DistanceMap v_d_map = v_d_map_gen(g, arg_pack);
        typedef typename boost::property_traits<DistanceMap>::value_type D;
        const D inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<D>()
        ];
        const D zero_distance = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<D>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                dijkstra_visitor<>
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<dijkstra_visitor<> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::predecessor_map,
                dummy_property_map
            >::type
        >::type v_p_map = arg_pack[
            boost::graph::keywords::_predecessor_map ||
            boost::value_factory<dummy_property_map>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_compare,
                std::less<D>
            >::type
        >::type dist_comp = arg_pack[
            boost::graph::keywords::_distance_compare ||
            boost::value_factory<std::less<D> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_combine,
                std::plus<D>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            boost::value_factory<std::plus<D> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::color_map,
                boost::detail::two_bit_color_map_generator<
                    Graph,
                    IndexMap
                >
            >::type
        >::type v_c_map = arg_pack[
            boost::graph::keywords::_color_map ||
            boost::detail::two_bit_color_map_generator<
                Graph,
                IndexMap
            >(g, v_i_map)
        ];
        dag_shortest_paths(
            g, s, v_d_map, e_w_map, v_c_map, v_p_map, vis,
            dist_comp, dist_comb, inf, zero_distance
        );
    }
}} // namespace boost::graph

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const Graph& g, \
        typename graph_traits<Graph>::vertex_descriptor s, \
        const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, mpl::true_ \
        >::type = mpl::true_() \
    ) \
    { \
        name( \
            g, s, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 11, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, dag_shortest_paths
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

    using ::boost::graph::dag_shortest_paths;
}

namespace boost { namespace detail {

    // Defaults are the same as Dijkstra's algorithm

    // Handle Distance Compare, Combine, Inf and Zero defaults
    template <class VertexListGraph, class DijkstraVisitor, 
      class DistanceMap, class WeightMap, class ColorMap, 
      class IndexMap, class Param, class Tag, class Rest>
    inline void
    dag_sp_dispatch2
      (const VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s, 
       DistanceMap distance, WeightMap weight, ColorMap color, IndexMap /*id*/,
       DijkstraVisitor vis, const bgl_named_params<Param,Tag,Rest>& params)
    {
      typedef typename property_traits<DistanceMap>::value_type D;
      dummy_property_map p_map;
      D inf =
        choose_param(get_param(params, distance_inf_t()), 
                     (std::numeric_limits<D>::max)());
      dag_shortest_paths
        (g, s, distance, weight, color, 
         choose_param(get_param(params, vertex_predecessor), p_map),
         vis, 
         choose_param(get_param(params, distance_compare_t()), std::less<D>()),
         choose_param(get_param(params, distance_combine_t()), std::plus<D>()),
         inf,
         choose_param(get_param(params, distance_zero_t()), 
                      D()));
    }

    // Handle DistanceMap and ColorMap defaults
    template <class VertexListGraph, class DijkstraVisitor, 
              class DistanceMap, class WeightMap, class ColorMap,
              class IndexMap, class Param, class Tag, class Rest>
    inline void
    dag_sp_dispatch1
      (const VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s, 
       DistanceMap distance, WeightMap weight, ColorMap color, IndexMap id,
       DijkstraVisitor vis, const bgl_named_params<Param,Tag,Rest>& params)
    {
      typedef typename property_traits<WeightMap>::value_type T;
      typename std::vector<T>::size_type n;
      n = is_default_param(distance) ? num_vertices(g) : 1;
      std::vector<T> distance_map(n);
      n = is_default_param(color) ? num_vertices(g) : 1;
      std::vector<default_color_type> color_map(n);

      dag_sp_dispatch2
        (g, s, 
         choose_param(distance, 
                      make_iterator_property_map(distance_map.begin(), id,
                                                 distance_map[0])),
         weight, 
         choose_param(color,
                      make_iterator_property_map(color_map.begin(), id, 
                                                 color_map[0])),
         id, vis, params);
    }
}} // namespace boost::detail

namespace boost {

  template <class VertexListGraph, class Param, class Tag, class Rest>
  inline void
  dag_shortest_paths
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     const bgl_named_params<Param,Tag,Rest>& params)
  {
    // assert that the graph is directed...
    null_visitor null_vis;
    detail::dag_sp_dispatch1
      (g, s, 
       get_param(params, vertex_distance),
       choose_const_pmap(get_param(params, edge_weight), g, edge_weight),
       get_param(params, vertex_color),
       choose_const_pmap(get_param(params, vertex_index), g, vertex_index),
       choose_param(get_param(params, graph_visitor),
                    make_dijkstra_visitor(null_vis)),
       params);
  }
} // namespace boost

#endif // BOOST_GRAPH_DAG_SHORTEST_PATHS_HPP
