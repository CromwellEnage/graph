//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
#ifndef BOOST_GRAPH_MST_PRIM_HPP
#define BOOST_GRAPH_MST_PRIM_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/lambda/core.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>
#include <functional>

namespace boost { namespace graph {

    template <
        typename VertexListGraph,
        typename DijkstraVisitor,
        typename PredecessorMap,
        typename DistanceMap,
        typename WeightMap,
        typename IndexMap
    >
    inline void prim_minimum_spanning_tree(
        const VertexListGraph& g,
        typename graph_traits<VertexListGraph>::vertex_descriptor s,
        PredecessorMap predecessor,
        DistanceMap distance,
        WeightMap weight,
        IndexMap index_map,
        DijkstraVisitor vis,
        typename boost::disable_if<
            parameter::are_tagged_arguments<
                PredecessorMap,
                DistanceMap,
                WeightMap,
                IndexMap,
                DijkstraVisitor
            >,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typedef typename property_traits<WeightMap>::value_type W;
        dijkstra_shortest_paths(
            g,
            s,
            predecessor,
            distance,
            weight,
            index_map,
            std::less<W>(),
            boost::lambda::_2,
            (std::numeric_limits<W>::max)(),
            0,
            vis
        );
    }
}} // namespace boost::graph

#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/two_bit_color_map_generator.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/functional/value_factory.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename PredecessorMap, typename Args>
    void prim_minimum_spanning_tree(
        const Graph& g, PredecessorMap p_map, const Args& arg_pack,
        typename boost::enable_if<
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
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type v_d_map = v_d_map_gen(g, arg_pack);
        std::less<Weight> compare;
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                default_dijkstra_visitor
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<default_dijkstra_visitor>()
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
        dijkstra_shortest_paths(
            g,
            arg_pack[
                boost::graph::keywords::_root_vertex ||
                boost::detail::get_default_starting_vertex_t<Graph>(g)
            ],
            boost::graph::keywords::_predecessor_map = p_map,
            boost::graph::keywords::_distance_map = v_d_map,
            boost::graph::keywords::_weight_map = e_w_map,
            boost::graph::keywords::_vertex_index_map = v_i_map,
            boost::graph::keywords::_distance_compare = compare,
            boost::graph::keywords::_distance_combine = boost::lambda::_2,
            boost::graph::keywords::_distance_inf = (
                std::numeric_limits<Weight>::max
            )(),
            boost::graph::keywords::_visitor = vis,
            boost::graph::keywords::_color_map = v_c_map
        );
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename PredMap>
    inline void prim_minimum_spanning_tree(const Graph& g, PredMap p_map)
    {
        prim_minimum_spanning_tree(g, p_map, parameter::compose());
    }
}} // namespace boost::graph

#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename PredMap, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const Graph &g, PredMap p_map, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, \
            mpl::true_ \
        >::type = mpl::true_()) \
    { \
        name( \
            g, p_map, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 6, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, prim_minimum_spanning_tree
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

    using ::boost::graph::prim_minimum_spanning_tree;
} // namespace boost

namespace boost { namespace detail {

    // This is Prim's algorithm to calculate the Minimum Spanning Tree
    // for an undirected graph with weighted edges.

    template <
        typename Graph, typename P, typename T, typename R, typename Weight
    >
    inline void prim_mst_impl(
        const Graph& G, typename graph_traits<Graph>::vertex_descriptor s,
        const bgl_named_params<P,T,R>& params, Weight
    )
    {
        typedef typename property_traits<Weight>::value_type W;
        std::less<W> compare;
        dijkstra_shortest_paths(
            G, s, params.distance_compare(compare).distance_combine(
                boost::lambda::_2
            )
        );
    }
}} // namespace boost::detail

namespace boost {

  template <class VertexListGraph, class PredecessorMap,
            class P, class T, class R>
  inline void prim_minimum_spanning_tree
    (const VertexListGraph& g,
     PredecessorMap p_map,
     const bgl_named_params<P,T,R>& params)
  {
    detail::prim_mst_impl
      (g, 
       choose_param(get_param(params, root_vertex_t()), *vertices(g).first), 
       params.predecessor_map(p_map),
       choose_const_pmap(get_param(params, edge_weight), g, edge_weight));
  }
} // namespace boost

#endif // BOOST_GRAPH_MST_PRIM_HPP
