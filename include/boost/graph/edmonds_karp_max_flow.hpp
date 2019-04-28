//=======================================================================
// Copyright 2000 University of Notre Dame.
// Authors: Jeremy G. Siek, Andrew Lumsdaine, Lie-Quan Lee
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#ifndef EDMONDS_KARP_MAX_FLOW_HPP
#define EDMONDS_KARP_MAX_FLOW_HPP

#include <boost/config.hpp>
#include <vector>
#include <algorithm> // for std::min and std::max
#include <boost/config.hpp>
#include <boost/pending/queue.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/breadth_first_search.hpp>

// The "labeling" algorithm from "Network Flows" by Ahuja, Magnanti,
// Orlin.  I think this is the same as or very similar to the original
// Edmonds-Karp algorithm.  This solves the maximum flow problem.

namespace boost { namespace detail {

    template <class Graph, class ResCapMap>
    filtered_graph<Graph, is_residual_edge<ResCapMap> >
    residual_graph(Graph& g, ResCapMap residual_capacity) {
      return filtered_graph<Graph, is_residual_edge<ResCapMap> >
        (g, is_residual_edge<ResCapMap>(residual_capacity));
    }

    template <class Graph, class PredEdgeMap, class ResCapMap,
              class RevEdgeMap>
    inline void
    augment(Graph& g, 
            typename graph_traits<Graph>::vertex_descriptor src,
            typename graph_traits<Graph>::vertex_descriptor sink,
            PredEdgeMap p, 
            ResCapMap residual_capacity,
            RevEdgeMap reverse_edge)
    {
      typename graph_traits<Graph>::edge_descriptor e;
      typename graph_traits<Graph>::vertex_descriptor u;
      typedef typename property_traits<ResCapMap>::value_type FlowValue;

      // find minimum residual capacity along the augmenting path
      FlowValue delta = (std::numeric_limits<FlowValue>::max)();
      e = get(p, sink);
      do {
        BOOST_USING_STD_MIN();
        delta = min BOOST_PREVENT_MACRO_SUBSTITUTION(delta, get(residual_capacity, e));
        u = source(e, g);
        e = get(p, u);
      } while (u != src);

      // push delta units of flow along the augmenting path
      e = get(p, sink);
      do {
        put(residual_capacity, e, get(residual_capacity, e) - delta);
        put(residual_capacity, get(reverse_edge, e), get(residual_capacity, get(reverse_edge, e)) + delta);
        u = source(e, g);
        e = get(p, u);
      } while (u != src);
    }
}} // namespace boost::detail

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

  template <class Graph, 
            class CapacityEdgeMap, class ResidualCapacityEdgeMap,
            class ReverseEdgeMap, class ColorMap, class PredEdgeMap>
  typename boost::disable_if<
    parameter::are_tagged_arguments<
      CapacityEdgeMap, ResidualCapacityEdgeMap, ReverseEdgeMap, ColorMap,
      PredEdgeMap
    >,
    typename property_traits<CapacityEdgeMap>::value_type
  >::type
  edmonds_karp_max_flow
    (Graph& g, 
     typename graph_traits<Graph>::vertex_descriptor src,
     typename graph_traits<Graph>::vertex_descriptor sink,
     CapacityEdgeMap cap, 
     ResidualCapacityEdgeMap res,
     ReverseEdgeMap rev, 
     ColorMap color, 
     PredEdgeMap pred)
  {
    typedef typename graph_traits<Graph>::vertex_descriptor vertex_t;
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;

    typename graph_traits<Graph>::vertex_iterator u_iter, u_end;
    typename graph_traits<Graph>::out_edge_iterator ei, e_end;
    for (boost::tie(u_iter, u_end) = vertices(g); u_iter != u_end; ++u_iter)
      for (boost::tie(ei, e_end) = out_edges(*u_iter, g); ei != e_end; ++ei)
        put(res, *ei, get(cap, *ei));

    put(color, sink, Color::gray());
    while (get(color, sink) != Color::white()) {
      boost::queue<vertex_t> Q;
      breadth_first_search
        (boost::detail::residual_graph(g, res), src, Q,
         make_bfs_visitor(record_edge_predecessors(pred, on_tree_edge())),
         color);
      if (get(color, sink) != Color::white())
        boost::detail::augment(g, src, sink, pred, res, rev);
    } // while

    typename property_traits<CapacityEdgeMap>::value_type flow = 0;
    for (boost::tie(ei, e_end) = out_edges(src, g); ei != e_end; ++ei)
      flow += (get(cap, *ei) - get(res, *ei));
    return flow;
  } // edmonds_karp_max_flow()
}} // namespace boost::graph

#include <boost/graph/named_function_params.hpp>

namespace boost { namespace detail {

    //-------------------------------------------------------------------------
    // Handle default for color property map

    // use of class here is a VC++ workaround
    template <class ColorMap>
    struct edmonds_karp_dispatch2 {
      template <class Graph, class PredMap, class P, class T, class R>
      static typename edge_capacity_value<Graph, P, T, R>::type
      apply
      (Graph& g,
       typename graph_traits<Graph>::vertex_descriptor src,
       typename graph_traits<Graph>::vertex_descriptor sink,
       PredMap pred,
       const bgl_named_params<P, T, R>& params,
       ColorMap color)
      {
        return edmonds_karp_max_flow
          (g, src, sink, 
           choose_const_pmap(get_param(params, edge_capacity), g, edge_capacity),
           choose_pmap(get_param(params, edge_residual_capacity), 
                       g, edge_residual_capacity),
           choose_const_pmap(get_param(params, edge_reverse), g, edge_reverse),
           color, pred);
      }
    };
    template<>
    struct edmonds_karp_dispatch2<param_not_found> {
      template <class Graph, class PredMap, class P, class T, class R>
      static typename edge_capacity_value<Graph, P, T, R>::type
      apply
      (Graph& g,
       typename graph_traits<Graph>::vertex_descriptor src,
       typename graph_traits<Graph>::vertex_descriptor sink,
       PredMap pred,
       const bgl_named_params<P, T, R>& params,
       param_not_found)
      {
        typedef typename graph_traits<Graph>::vertices_size_type size_type;
        size_type n = is_default_param(get_param(params, vertex_color)) ?
          num_vertices(g) : 1;
        std::vector<default_color_type> color_vec(n);
        return edmonds_karp_max_flow
          (g, src, sink, 
           choose_const_pmap(get_param(params, edge_capacity), g, edge_capacity),
           choose_pmap(get_param(params, edge_residual_capacity), 
                       g, edge_residual_capacity),
           choose_const_pmap(get_param(params, edge_reverse), g, edge_reverse),
           make_iterator_property_map(color_vec.begin(), choose_const_pmap
                                      (get_param(params, vertex_index),
                                       g, vertex_index), color_vec[0]),
           pred);
      }
    };

    //-------------------------------------------------------------------------
    // Handle default for predecessor property map

    // use of class here is a VC++ workaround
    template <class PredMap>
    struct edmonds_karp_dispatch1 {
      template <class Graph, class P, class T, class R>
      static typename edge_capacity_value<Graph, P, T, R>::type
      apply(Graph& g,
            typename graph_traits<Graph>::vertex_descriptor src,
            typename graph_traits<Graph>::vertex_descriptor sink,
            const bgl_named_params<P, T, R>& params,
            PredMap pred)
      {
        typedef typename get_param_type< vertex_color_t, bgl_named_params<P,T,R> >::type C;
        return edmonds_karp_dispatch2<C>::apply
          (g, src, sink, pred, params, get_param(params, vertex_color));
      }
    };
    template<>
    struct edmonds_karp_dispatch1<param_not_found> {

      template <class Graph, class P, class T, class R>
      static typename edge_capacity_value<Graph, P, T, R>::type
      apply
      (Graph& g,
       typename graph_traits<Graph>::vertex_descriptor src,
       typename graph_traits<Graph>::vertex_descriptor sink,
       const bgl_named_params<P, T, R>& params,
       param_not_found)
      {
        typedef typename graph_traits<Graph>::edge_descriptor edge_descriptor;
        typedef typename graph_traits<Graph>::vertices_size_type size_type;
        size_type n = is_default_param(get_param(params, vertex_predecessor)) ?
          num_vertices(g) : 1;
        std::vector<edge_descriptor> pred_vec(n);
        
        typedef typename get_param_type< vertex_color_t, bgl_named_params<P,T,R> >::type C;
        return edmonds_karp_dispatch2<C>::apply
          (g, src, sink, 
           make_iterator_property_map(pred_vec.begin(), choose_const_pmap
                                      (get_param(params, vertex_index),
                                       g, vertex_index), pred_vec[0]),
           params, 
           get_param(params, vertex_color));
      }
    };
}} // namespace boost::detail

#include <boost/parameter/is_argument_pack.hpp>

namespace boost { namespace graph {

// Boost.Parameter-enabled argument-pack overload
template <typename Graph, typename Args>
inline typename boost::lazy_enable_if<
  parameter::is_argument_pack<Args>,
  boost::detail::arg_packed_property_map_value<
    Args, boost::graph::keywords::tag::capacity_map, edge_capacity_t, Graph
  >
>::type
edmonds_karp_max_flow(Graph& g,
                      typename graph_traits<Graph>::vertex_descriptor src,
                      typename graph_traits<Graph>::vertex_descriptor sink,
                      const Args& arg_pack)
{
    typename boost::detail::map_maker<
        Graph,
        Args,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type v_c_map = boost::detail::make_color_map_from_arg_pack(
        g,
        arg_pack
    );
    typedef typename graph_traits<Graph>::edge_descriptor Edge;
    const Edge no_edge = Edge();
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::predecessor_map,
        Edge
    > v_p_map_gen(no_edge);
    typename boost::detail::map_maker<
        Graph,
        Args,
        boost::graph::keywords::tag::predecessor_map,
        Edge
    >::map_type v_p_map = v_p_map_gen(g, arg_pack);
    return edmonds_karp_max_flow(
        g,
        src,
        sink,
        boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_capacity_map,
            g,
            edge_capacity
        ),
        boost::detail::override_property(
            arg_pack,
            boost::graph::keywords::_residual_capacity_map,
            g,
            edge_residual_capacity
        ),
        boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_reverse_edge_map,
            g,
            edge_reverse
        ),
        v_c_map,
        v_p_map
    );
}
}} // namespace boost::detail

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

// all-defaults overload
template <typename Graph>
inline typename property_traits<
    typename property_map<Graph, edge_capacity_t>::const_type
>::value_type
edmonds_karp_max_flow(Graph& g,
                      typename graph_traits<Graph>::vertex_descriptor src,
                      typename graph_traits<Graph>::vertex_descriptor sink)
{
  return edmonds_karp_max_flow(g, src, sink, parameter::compose());
}
}} // namespace boost::graph

#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

// Boost.Parameter-enabled tagged-argument overloads
#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
template <typename Graph, typename TA \
          BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA)> \
inline typename boost::lazy_enable_if< \
  parameter::are_tagged_arguments< \
    TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
  >, \
  boost::detail::tagged_property_map_value< \
    boost::graph::keywords::tag::capacity_map, edge_capacity_t, Graph \
    BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
  > \
>::type \
name(Graph& g, typename graph_traits<Graph>::vertex_descriptor src, \
     typename graph_traits<Graph>::vertex_descriptor sink, \
     const TA& ta BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta)) \
{ \
  return name(g, src, sink, parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta))); \
}

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(1, 7, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, edmonds_karp_max_flow)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

using ::boost::graph::edmonds_karp_max_flow;

// old-style named-parameter overload
template <typename Graph, typename P, typename T, typename R>
inline typename property_traits<
  typename detail::choose_impl_result<
    boost::is_const<Graph>,
    Graph,
    typename get_param_type<bgl_named_params<P, T, R>, edge_capacity_t>::type,
    edge_capacity_t
  >::type
>::value_type
edmonds_karp_max_flow(Graph& g,
                      typename graph_traits<Graph>::vertex_descriptor src,
                      typename graph_traits<Graph>::vertex_descriptor sink,
                      const bgl_named_params<P, T, R>& params)
{
  typedef typename get_param_type< vertex_predecessor_t, bgl_named_params<P,T,R> >::type Pred;
  return detail::edmonds_karp_dispatch1<Pred>::apply
    (g, src, sink, params, get_param(params, vertex_predecessor));
}
} // namespace boost

#endif // EDMONDS_KARP_MAX_FLOW_HPP

