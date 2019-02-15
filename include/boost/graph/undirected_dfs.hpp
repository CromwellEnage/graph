//
//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
#ifndef BOOST_GRAPH_UNDIRECTED_DFS_HPP
#define BOOST_GRAPH_UNDIRECTED_DFS_HPP

#include <boost/graph/depth_first_search.hpp>
#include <vector>
#include <boost/concept/assert.hpp>
#include <boost/mpl/vector.hpp>
#include <boost/mpl/placeholders.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/enum_params.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#endif

namespace boost {

  namespace detail {

// Define BOOST_RECURSIVE_DFS to use older, recursive version.
// It is retained for a while in order to perform performance
// comparison.
#ifndef BOOST_RECURSIVE_DFS

    template <typename IncidenceGraph, typename DFSVisitor, 
              typename VertexColorMap, typename EdgeColorMap>
    void undir_dfv_impl
      (const IncidenceGraph& g,
       typename graph_traits<IncidenceGraph>::vertex_descriptor u, 
       DFSVisitor& vis,
       VertexColorMap vertex_color,
       EdgeColorMap edge_color)
    {
      BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<IncidenceGraph> ));
      BOOST_CONCEPT_ASSERT(( DFSVisitorConcept<DFSVisitor, IncidenceGraph> ));
      typedef typename graph_traits<IncidenceGraph>::vertex_descriptor Vertex;
      typedef typename graph_traits<IncidenceGraph>::edge_descriptor Edge;
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<VertexColorMap,Vertex> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<EdgeColorMap,Edge> ));
      typedef typename property_traits<VertexColorMap>::value_type ColorValue;
      typedef typename property_traits<EdgeColorMap>::value_type EColorValue;
      BOOST_CONCEPT_ASSERT(( ColorValueConcept<ColorValue> ));
      BOOST_CONCEPT_ASSERT(( ColorValueConcept<EColorValue> ));
      typedef color_traits<ColorValue> Color;
      typedef color_traits<EColorValue> EColor;
      typedef typename graph_traits<IncidenceGraph>::out_edge_iterator Iter;
      typedef std::pair<Vertex, std::pair<boost::optional<Edge>, std::pair<Iter, Iter> > > VertexInfo;

      std::vector<VertexInfo> stack;

      put(vertex_color, u, Color::gray());
      vis.discover_vertex(u, g);
      stack.push_back(std::make_pair(u, std::make_pair(boost::optional<Edge>(), out_edges(u, g))));
      while (!stack.empty()) {
        VertexInfo& back = stack.back();
        u = back.first;
        boost::optional<Edge> src_e = back.second.first;
        Iter ei = back.second.second.first, ei_end = back.second.second.second;
        stack.pop_back();
        while (ei != ei_end) {
          Vertex v = target(*ei, g);
          vis.examine_edge(*ei, g);
          ColorValue v_color = get(vertex_color, v);
          EColorValue uv_color = get(edge_color, *ei);
          put(edge_color, *ei, EColor::black());
          if (v_color == Color::white()) {
            vis.tree_edge(*ei, g);
            src_e = *ei;
            stack.push_back(std::make_pair(u, std::make_pair(src_e, std::make_pair(++ei, ei_end))));
            u = v;
            put(vertex_color, u, Color::gray());
            vis.discover_vertex(u, g);
            boost::tie(ei, ei_end) = out_edges(u, g);
          } else if (v_color == Color::gray()) {
            if (uv_color == EColor::white()) vis.back_edge(*ei, g);
            call_finish_edge(vis, *ei, g);
            ++ei;
          } else { // if (v_color == Color::black())
            call_finish_edge(vis, *ei, g);
            ++ei;
          }
        }
        put(vertex_color, u, Color::black());
        vis.finish_vertex(u, g);
        if (src_e) call_finish_edge(vis, src_e.get(), g);
      }
    }

#else // BOOST_RECURSIVE_DFS

    template <typename IncidenceGraph, typename DFSVisitor, 
              typename VertexColorMap, typename EdgeColorMap>
    void undir_dfv_impl
      (const IncidenceGraph& g,
       typename graph_traits<IncidenceGraph>::vertex_descriptor u, 
       DFSVisitor& vis,  // pass-by-reference here, important!
       VertexColorMap vertex_color,
       EdgeColorMap edge_color)
    {
      BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<IncidenceGraph> ));
      BOOST_CONCEPT_ASSERT(( DFSVisitorConcept<DFSVisitor, IncidenceGraph> ));
      typedef typename graph_traits<IncidenceGraph>::vertex_descriptor Vertex;
      typedef typename graph_traits<IncidenceGraph>::edge_descriptor Edge;
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<VertexColorMap,Vertex> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<EdgeColorMap,Edge> ));
      typedef typename property_traits<VertexColorMap>::value_type ColorValue;
      typedef typename property_traits<EdgeColorMap>::value_type EColorValue;
      BOOST_CONCEPT_ASSERT(( ColorValueConcept<ColorValue> ));
      BOOST_CONCEPT_ASSERT(( ColorValueConcept<EColorValue> ));
      typedef color_traits<ColorValue> Color;
      typedef color_traits<EColorValue> EColor;
      typename graph_traits<IncidenceGraph>::out_edge_iterator ei, ei_end;

      put(vertex_color, u, Color::gray());   vis.discover_vertex(u, g);
      for (boost::tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
        Vertex v = target(*ei, g);           vis.examine_edge(*ei, g);
        ColorValue v_color = get(vertex_color, v);
        EColorValue uv_color = get(edge_color, *ei);
        put(edge_color, *ei, EColor::black());
        if (v_color == Color::white()) {     vis.tree_edge(*ei, g);
          undir_dfv_impl(g, v, vis, vertex_color, edge_color);
        } else if (v_color == Color::gray() && uv_color == EColor::white())
                                             vis.back_edge(*ei, g);
                                             call_finish_edge(vis, *ei, g);
      }
      put(vertex_color, u, Color::black());  vis.finish_vertex(u, g);
    }

#endif // ! BOOST_RECURSIVE_DFS

  } // namespace detail

  // Boost.Parameter-enabled variant
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  BOOST_PARAMETER_FUNCTION(
    (bool), undirected_dfs, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_edge_list_graph>))
    )
    (deduced
      (required
        (edge_color_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_edge_color_map_of_graph
            >
          )
        )
      )
      (optional
        (vertex_index_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_to_integer_map_of_graph
            >
          )
          ,detail::vertex_index_map_or_dummy_property_map(graph)
        )
        (color_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_color_map_of_graph
            >
          )
          ,make_shared_array_property_map(
            num_vertices(graph),
            white_color,
            vertex_index_map
          )
        )
        (visitor
          ,*(detail::dfs_visitor_predicate)
          ,default_dfs_visitor()
        )
        (root_vertex
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_of_graph
            >
          )
          ,detail::get_default_starting_vertex(graph)
        )
      )
    )
  )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  template <typename Graph, typename DFSVisitor,
            typename VertexColorMap, typename EdgeColorMap,
            typename Vertex>
  void
  undirected_dfs(
    const Graph& g, DFSVisitor vis, VertexColorMap vertex_color,
    EdgeColorMap edge_color, Vertex start_vertex, typename boost::disable_if<
      parameter::are_tagged_arguments<
        DFSVisitor, VertexColorMap, EdgeColorMap, Vertex
      >,
      mpl::true_
    >::type = mpl::true_()
  )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS
  {
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
    typedef typename boost::remove_const<
      typename boost::remove_reference<graph_type>::type
    >::type Graph;
    typedef typename boost::remove_const<
      typename boost::remove_reference<visitor_type>::type
    >::type DFSVisitor;
    typedef typename boost::remove_const<
      typename boost::remove_reference<color_map_type>::type
    >::type VertexColorMap;
#endif

    BOOST_CONCEPT_ASSERT(( DFSVisitorConcept<DFSVisitor, Graph> ));
    BOOST_CONCEPT_ASSERT(( EdgeListGraphConcept<Graph> ));

    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename property_traits<VertexColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;

#if !defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    DFSVisitor vis = visitor;
#endif
    typename graph_traits<Graph>::vertex_iterator ui, ui_end;

    for (boost::tie(ui, ui_end) = vertices(graph); ui != ui_end; ++ui) {
      put(color_map, *ui, Color::white());
#if defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
      visitor.initialize_vertex(*ui, graph);
#else
      vis.initialize_vertex(*ui, graph);
#endif
    }

    typename graph_traits<Graph>::edge_iterator ei, ei_end;

    for (boost::tie(ei, ei_end) = edges(graph); ei != ei_end; ++ei)
      put(edge_color_map, *ei, Color::white());

#if defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    if (root_vertex != detail::get_default_starting_vertex(graph)) {
      visitor.start_vertex(root_vertex, graph);
      detail::undir_dfv_impl(graph, root_vertex, visitor, color_map,
                             edge_color_map);
    }
#else
    Vertex s = root_vertex;

    if (s != detail::get_default_starting_vertex(graph)) {
      vis.start_vertex(s, graph);
      detail::undir_dfv_impl(graph, s, vis, color_map,
                             edge_color_map);
    }
#endif

    for (boost::tie(ui, ui_end) = vertices(graph); ui != ui_end; ++ui) {
      ColorValue u_color = get(color_map, *ui);
      if (u_color == Color::white()) {
#if defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
        visitor.start_vertex(*ui, graph);
        detail::undir_dfv_impl(graph, *ui, visitor, color_map,
                               edge_color_map);
#else
        vis.start_vertex(*ui, graph);
        detail::undir_dfv_impl(graph, *ui, vis, color_map, edge_color_map);
#endif
      }
    }

    return true;
  }

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  template <typename Graph, typename DFSVisitor, typename VertexColorMap,
    typename EdgeColorMap>
  void
  undirected_dfs(
    const Graph& g, DFSVisitor vis, VertexColorMap vertex_color,
    EdgeColorMap edge_color, typename boost::disable_if<
      parameter::are_tagged_arguments<
        DFSVisitor, VertexColorMap, EdgeColorMap
      >,
      mpl::true_
    >::type = mpl::true_()
  )
  {
    undirected_dfs(
      g,
      vis,
      vertex_color,
      edge_color,
      detail::get_default_starting_vertex(g)
    );
  }

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
  template <typename Graph \
            BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA)> \
  inline void name \
    (const Graph &g \
     BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
     typename boost::enable_if< \
       parameter::are_tagged_arguments<BOOST_PP_ENUM_PARAMS_Z(z, n, TA)>, \
       mpl::true_ \
     >::type = mpl::true_()) \
  { \
    name(g, parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta))); \
  }

BOOST_PP_REPEAT_FROM_TO(1, 9, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, undirected_dfs)

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

#endif  // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

  // Old-style named parameter variant
  template <typename Graph, typename P, typename T, typename R>
  void
  undirected_dfs(const Graph& g, 
                 const bgl_named_params<P, T, R>& params)
  {
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    undirected_dfs(
      g,
      boost::graph::keywords::_visitor = arg_pack[
        boost::graph::keywords::_visitor ||
        boost::value_factory<default_dfs_visitor>()
      ],
      boost::graph::keywords::_color_map = arg_pack[
        boost::graph::keywords::_color_map |
        make_shared_array_property_map(
          num_vertices(g),
          white_color,
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            get(vertex_index, g)
          ]
        )
      ],
      boost::graph::keywords::_edge_color_map = arg_pack[
        boost::graph::keywords::_edge_color_map
      ],
      boost::graph::keywords::_root_vertex = arg_pack[
        boost::graph::keywords::_root_vertex ||
        boost::detail::get_default_starting_vertex_t<Graph>(g)
      ]
    );
  }

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  BOOST_PARAMETER_FUNCTION(
    (bool), undirected_depth_first_visit, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_incidence_graph>))
    )
    (deduced
      (required
        (root_vertex
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_of_graph
            >
          )
        )
        (visitor, *(detail::dfs_visitor_predicate))
        (color_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_color_map_of_graph
            >
          )
        )
        (edge_color_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_edge_color_map_of_graph
            >
          )
        )
      )
    )
  )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  BOOST_PARAMETER_FUNCTION(
    (bool), undirected_depth_first_visit, ::boost::graph::keywords::tag,
    (required
      (graph, *)
      (root_vertex, *)
      (visitor, *)
      (color_map, *)
      (edge_color_map, *)
    )
  )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS
  {
#if defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    detail::undir_dfv_impl(graph, root_vertex, visitor, color_map,
                           edge_color_map);
#else
    typename boost::remove_const<
      typename boost::remove_reference<root_vertex_type>::type
    >::type s = root_vertex;
    typename boost::remove_const<
      typename boost::remove_reference<visitor_type>::type
    >::type vis = visitor;
    detail::undir_dfv_impl(graph, s, vis, color_map, edge_color_map);
#endif
    return true;
  }
} // namespace boost

#endif
