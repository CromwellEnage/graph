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
#ifndef BOOST_GRAPH_BREADTH_FIRST_SEARCH_HPP
#define BOOST_GRAPH_BREADTH_FIRST_SEARCH_HPP

/*
  Breadth First Search Algorithm (Cormen, Leiserson, and Rivest p. 470)
*/
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/config.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/mpl/vector.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/type_traits/add_pointer.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/declval.hpp>

#if defined(BOOST_NO_CXX11_DECLTYPE)
#include <boost/typeof/typeof.hpp>
#endif

namespace boost { namespace detail {

#if !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
    template <typename T, typename G>
    class is_bfs_visitor_impl
    {
        typedef typename boost::remove_const<T>::type _m_T;

        template <typename B, typename P>
        static graph_yes_tag
            _check_init_v(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().initialize_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().initialize_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_init_v(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_disc_v(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().discover_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().discover_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_disc_v(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_exam_v(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().examine_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().examine_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_exam_v(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_exam_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().examine_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().examine_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_exam_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_tree_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().tree_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().tree_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_tree_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_back_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().back_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().back_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_back_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_n_t_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().non_tree_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().non_tree_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_n_t_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_g_t_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().gray_target(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().gray_target(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_g_t_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_b_t_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().black_target(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().black_target(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_b_t_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_end_v(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().finish_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().finish_vertex(
                            boost::declval<
                                typename graph_traits<P>::vertex_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_end_v(...);

    public:
        typedef mpl::bool_<
            (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_init_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_disc_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_exam_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_exam_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_tree_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_n_t_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_g_t_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_b_t_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_bfs_visitor_impl<T,G>::_check_end_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            )
        > type;
    };

    template <typename T, typename G>
    struct is_bfs_visitor
      : mpl::eval_if<
        is_bgl_graph<G>,
        is_bfs_visitor_impl<T,G>,
        mpl::false_
      >::type
    { };

    typedef argument_with_graph_predicate<
      is_bfs_visitor
    > bfs_visitor_predicate;
#else   // defined(BOOST_NO_CXX11_DECLTYPE) && !defined(BOOST_TYPEOF_KEYWORD)
    typedef visitor_predicate bfs_visitor_predicate;
#endif  // !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

#include <boost/pending/queue.hpp>

namespace boost { namespace detail {

    template <typename Vertex>
    inline boost::queue<Vertex> create_empty_buffer(Vertex const&)
    {
        return boost::queue<Vertex>();
    }
}}

#include <vector>
#include <boost/functional/value_factory.hpp>
#include <boost/parameter.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/two_bit_color_map.hpp>
#include <boost/graph/detail/mpi_include.hpp>
#include <boost/concept/assert.hpp>

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/concepts.hpp>)

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/core/enable_if.hpp>
#endif

namespace boost {

  template <class Visitor, class Graph>
  struct BFSVisitorConcept {
    void constraints() {
      BOOST_CONCEPT_ASSERT(( CopyConstructibleConcept<Visitor> ));
      vis.initialize_vertex(u, g);
      vis.discover_vertex(u, g);
      vis.examine_vertex(u, g);
      vis.examine_edge(e, g);
      vis.tree_edge(e, g);
      vis.non_tree_edge(e, g);
      vis.gray_target(e, g);
      vis.black_target(e, g);
      vis.finish_vertex(u, g);
    }
    Visitor vis;
    Graph g;
    typename graph_traits<Graph>::vertex_descriptor u;
    typename graph_traits<Graph>::edge_descriptor e;
  };

  namespace graph { struct bfs_visitor_event_not_overridden {}; }

  template <class Visitors = null_visitor>
  class bfs_visitor {
  public:
    bfs_visitor() : m_vis() { }
    bfs_visitor(Visitors vis) : m_vis(vis) { }

    template <class Vertex, class Graph>
    graph::bfs_visitor_event_not_overridden
    initialize_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_initialize_vertex());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Vertex, class Graph>
    graph::bfs_visitor_event_not_overridden
    discover_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_discover_vertex());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Vertex, class Graph>
    graph::bfs_visitor_event_not_overridden
    examine_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_examine_vertex());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Edge, class Graph>
    graph::bfs_visitor_event_not_overridden
    examine_edge(Edge e, Graph& g)
    {
      invoke_visitors(m_vis, e, g, ::boost::on_examine_edge());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Edge, class Graph>
    graph::bfs_visitor_event_not_overridden
    tree_edge(Edge e, Graph& g)
    {
      invoke_visitors(m_vis, e, g, ::boost::on_tree_edge());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Edge, class Graph>
    graph::bfs_visitor_event_not_overridden
    non_tree_edge(Edge e, Graph& g)
    {
      invoke_visitors(m_vis, e, g, ::boost::on_non_tree_edge());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Edge, class Graph>
    graph::bfs_visitor_event_not_overridden
    gray_target(Edge e, Graph& g)
    {
      invoke_visitors(m_vis, e, g, ::boost::on_gray_target());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Edge, class Graph>
    graph::bfs_visitor_event_not_overridden
    black_target(Edge e, Graph& g)
    {
      invoke_visitors(m_vis, e, g, ::boost::on_black_target());
      return graph::bfs_visitor_event_not_overridden();
    }

    template <class Vertex, class Graph>
    graph::bfs_visitor_event_not_overridden
    finish_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_finish_vertex());
      return graph::bfs_visitor_event_not_overridden();
    }

    BOOST_GRAPH_EVENT_STUB(on_initialize_vertex,bfs)
    BOOST_GRAPH_EVENT_STUB(on_discover_vertex,bfs)
    BOOST_GRAPH_EVENT_STUB(on_examine_vertex,bfs)
    BOOST_GRAPH_EVENT_STUB(on_examine_edge,bfs)
    BOOST_GRAPH_EVENT_STUB(on_tree_edge,bfs)
    BOOST_GRAPH_EVENT_STUB(on_non_tree_edge,bfs)
    BOOST_GRAPH_EVENT_STUB(on_gray_target,bfs)
    BOOST_GRAPH_EVENT_STUB(on_black_target,bfs)
    BOOST_GRAPH_EVENT_STUB(on_finish_vertex,bfs)

  protected:
    Visitors m_vis;
  };
  template <class Visitors>
  bfs_visitor<Visitors>
  make_bfs_visitor(Visitors vis) {
    return bfs_visitor<Visitors>(vis);
  }
  typedef bfs_visitor<> default_bfs_visitor;

  // Multiple-source version
  template <class IncidenceGraph, class Buffer, class BFSVisitor,
            class ColorMap, class SourceIterator>
  void breadth_first_visit
    (const IncidenceGraph& g,
     SourceIterator sources_begin, SourceIterator sources_end,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<IncidenceGraph> ));
    typedef graph_traits<IncidenceGraph> GTraits;
    typedef typename GTraits::vertex_descriptor Vertex;
    BOOST_CONCEPT_ASSERT(( BFSVisitorConcept<BFSVisitor, IncidenceGraph> ));
    BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<ColorMap, Vertex> ));
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename GTraits::out_edge_iterator ei, ei_end;

    for (; sources_begin != sources_end; ++sources_begin) {
      Vertex s = *sources_begin;
      put(color, s, Color::gray());           vis.discover_vertex(s, g);
      Q.push(s);
    }
    while (! Q.empty()) {
      Vertex u = Q.top(); Q.pop();            vis.examine_vertex(u, g);
      for (boost::tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
        Vertex v = target(*ei, g);            vis.examine_edge(*ei, g);
        ColorValue v_color = get(color, v);
        if (v_color == Color::white()) {      vis.tree_edge(*ei, g);
          put(color, v, Color::gray());       vis.discover_vertex(v, g);
          Q.push(v);
        } else {                              vis.non_tree_edge(*ei, g);
          if (v_color == Color::gray())       vis.gray_target(*ei, g);
          else                                vis.black_target(*ei, g);
        }
      } // end for
      put(color, u, Color::black());          vis.finish_vertex(u, g);
    } // end while
  } // breadth_first_visit

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
#if !defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
  template <class IncidenceGraph, class Buffer, class BFSVisitor,
            class ColorMap, class SourceIterator>
  void breadth_first_visit
    (const IncidenceGraph& g,
     SourceIterator sources_begin, SourceIterator sources_end,
     Buffer const& default_buffer, BFSVisitor vis, ColorMap color)
  {
    BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<IncidenceGraph> ));
    typedef graph_traits<IncidenceGraph> GTraits;
    typedef typename GTraits::vertex_descriptor Vertex;
    BOOST_CONCEPT_ASSERT(( BFSVisitorConcept<BFSVisitor, IncidenceGraph> ));
    BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<ColorMap, Vertex> ));
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    Buffer Q(default_buffer);
    typename GTraits::out_edge_iterator ei, ei_end;

    for (; sources_begin != sources_end; ++sources_begin) {
      Vertex s = *sources_begin;
      put(color, s, Color::gray());           vis.discover_vertex(s, g);
      Q.push(s);
    }
    while (! Q.empty()) {
      Vertex u = Q.top(); Q.pop();            vis.examine_vertex(u, g);
      for (boost::tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
        Vertex v = target(*ei, g);            vis.examine_edge(*ei, g);
        ColorValue v_color = get(color, v);
        if (v_color == Color::white()) {      vis.tree_edge(*ei, g);
          put(color, v, Color::gray());       vis.discover_vertex(v, g);
          Q.push(v);
        } else {                              vis.non_tree_edge(*ei, g);
          if (v_color == Color::gray())       vis.gray_target(*ei, g);
          else                                vis.black_target(*ei, g);
        }
      } // end for
      put(color, u, Color::black());          vis.finish_vertex(u, g);
    } // end while
  } // breadth_first_visit
#endif  // !defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

  // Boost.Parameter-enabled single-source variant
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
  BOOST_PARAMETER_FUNCTION(
    (bool), breadth_first_visit, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_vertex_list_graph>))
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
      )
      (optional
        (visitor
          ,*(detail::bfs_visitor_predicate)
          ,default_bfs_visitor()
        )
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
        (buffer
          ,*(detail::argument_predicate<detail::is_buffer>)
          ,detail::create_empty_buffer(root_vertex)
        )
      )
    )
  )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
  BOOST_PARAMETER_FUNCTION(
    (
      boost::disable_if<
        detail::is_bgl_named_param_argument<
          Args,
          boost::graph::keywords::tag::buffer
        >,
        bool
      >
    ), breadth_first_visit, ::boost::graph::keywords::tag,
    (required
      (graph, *)
      (root_vertex, *)
    )
    (optional
      (buffer, *, detail::create_empty_buffer(root_vertex))
      (visitor, *, default_bfs_visitor())
      (color_map
        ,*
        ,make_shared_array_property_map(
          num_vertices(graph),
          white_color,
          detail::vertex_index_map_or_dummy_property_map(graph)
        )
      )
    )
  )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
  {
    typename graph_traits<
      typename boost::remove_const<
        typename boost::remove_reference<graph_type>::type
      >::type
    >::vertex_descriptor srcs[1] = {root_vertex};
    breadth_first_visit(graph, srcs, srcs + 1, buffer, visitor, color_map);
    return true;
  }
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
  // Single-source version
  template <class IncidenceGraph, class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_visit
    (const IncidenceGraph& g,
     typename graph_traits<IncidenceGraph>::vertex_descriptor s,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    typename graph_traits<IncidenceGraph>::vertex_descriptor sources[1] = {s};
    breadth_first_visit(g, sources, sources + 1, Q, vis, color);
  }
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

  template <class VertexListGraph, class SourceIterator,
            class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_search
    (const VertexListGraph& g,
     SourceIterator sources_begin, SourceIterator sources_end,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    // Initialization
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename boost::graph_traits<VertexListGraph>::vertex_iterator i, i_end;
    for (boost::tie(i, i_end) = vertices(g); i != i_end; ++i) {
      vis.initialize_vertex(*i, g);
      put(color, *i, Color::white());
    }
    breadth_first_visit(g, sources_begin, sources_end, Q, vis, color);
  }

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
#if !defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
  template <class VertexListGraph, class SourceIterator,
            class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_search
    (const VertexListGraph& g,
     SourceIterator sources_begin, SourceIterator sources_end,
     Buffer const& default_buffer, BFSVisitor vis, ColorMap color)
  {
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename boost::graph_traits<VertexListGraph>::vertex_iterator i, i_end;
    for (boost::tie(i, i_end) = vertices(g); i != i_end; ++i) {
      vis.initialize_vertex(*i, g);
      put(color, *i, Color::white());
    }
    Buffer Q(default_buffer);
    breadth_first_visit(g, sources_begin, sources_end, Q, vis, color);
  }
#endif  // !defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)

  // Boost.Parameter-enabled single-source variant
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
  BOOST_PARAMETER_FUNCTION(
    (bool), breadth_first_search, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_vertex_list_graph>))
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
      )
      (optional
        (visitor
          ,*(detail::bfs_visitor_predicate)
          ,default_bfs_visitor()
        )
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
        (buffer
          ,*(detail::argument_predicate<detail::is_buffer>)
          ,detail::create_empty_buffer(root_vertex)
        )
      )
    )
  )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
  BOOST_PARAMETER_FUNCTION(
    (
      boost::disable_if<
        detail::is_bgl_named_param_argument<
          Args,
          boost::graph::keywords::tag::buffer
        >,
        bool
      >
    ), breadth_first_search, ::boost::graph::keywords::tag,
    (required
      (graph, *)
      (root_vertex, *)
    )
    (optional
      (buffer, *, detail::create_empty_buffer(root_vertex))
      (visitor, *, default_bfs_visitor())
      (color_map
        ,*
        ,make_shared_array_property_map(
          num_vertices(graph),
          white_color,
          detail::vertex_index_map_or_dummy_property_map(graph)
        )
      )
    )
  )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
  {
    typename graph_traits<
      typename boost::remove_const<
        typename boost::remove_reference<graph_type>::type
      >::type
    >::vertex_descriptor srcs[1] = {root_vertex};
    breadth_first_search(graph, srcs, srcs + 1, buffer, visitor, color_map);
    return true;
  }
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
  template <class VertexListGraph, class Buffer, class BFSVisitor,
            class ColorMap>
  void breadth_first_search
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     Buffer& Q, BFSVisitor vis, ColorMap color)
  {
    typename graph_traits<VertexListGraph>::vertex_descriptor sources[1] = {s};
    breadth_first_search(g, sources, sources + 1, Q, vis, color);
  }
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

  namespace detail {

    template <class VertexListGraph, class ColorMap, class BFSVisitor,
      class P, class T, class R>
    void bfs_helper
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       ColorMap color,
       BFSVisitor vis,
       const bgl_named_params<P, T, R>& params,
       boost::mpl::false_)
    {
      typedef graph_traits<VertexListGraph> Traits;
      // Buffer default
      typedef typename Traits::vertex_descriptor Vertex;
      typedef boost::queue<Vertex> queue_t;
      queue_t Q;
      breadth_first_search
        (g, s,
         choose_param(get_param(params, buffer_param_t()), boost::ref(Q)).get(),
         vis, color);
    }

#ifdef BOOST_GRAPH_USE_MPI
    template <class DistributedGraph, class ColorMap, class BFSVisitor,
              class P, class T, class R>
    void bfs_helper
      (DistributedGraph& g,
       typename graph_traits<DistributedGraph>::vertex_descriptor s,
       ColorMap color,
       BFSVisitor vis,
       const bgl_named_params<P, T, R>& params,
       boost::mpl::true_);
#endif // BOOST_GRAPH_USE_MPI

#if !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    //-------------------------------------------------------------------------
    // Choose between default color and color parameters. Using
    // function dispatching so that we don't require vertex index if
    // the color default is not being used.

    template <class ColorMap>
    struct bfs_dispatch {
      template <class VertexListGraph, class P, class T, class R>
      static void apply
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       const bgl_named_params<P, T, R>& params,
       ColorMap color)
      {
        bfs_helper
          (g, s, color,
           choose_param(get_param(params, graph_visitor),
                        make_bfs_visitor(null_visitor())),
           params,
           boost::mpl::bool_<
             boost::is_base_and_derived<
               distributed_graph_tag,
               typename graph_traits<VertexListGraph>::traversal_category>::value>());
      }
    };

    template <>
    struct bfs_dispatch<param_not_found> {
      template <class VertexListGraph, class P, class T, class R>
      static void apply
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       const bgl_named_params<P, T, R>& params,
       param_not_found)
      {
        null_visitor null_vis;

        bfs_helper
          (g, s,
           make_two_bit_color_map
           (num_vertices(g),
            choose_const_pmap(get_param(params, vertex_index),
                              g, vertex_index)),
           choose_param(get_param(params, graph_visitor),
                        make_bfs_visitor(null_vis)),
           params,
           boost::mpl::bool_<
             boost::is_base_and_derived<
               distributed_graph_tag,
               typename graph_traits<VertexListGraph>::traversal_category>::value>());
      }
    };
#endif  // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
  } // namespace detail

  // Named Parameter Variant
  template <class VertexListGraph, class P, class T, class R>
  void breadth_first_search
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     const bgl_named_params<P, T, R>& params)
  {
    // The graph is passed by *const* reference so that graph adaptors
    // (temporaries) can be passed into this function. However, the
    // graph is not really const since we may write to property maps
    // of the graph.
    VertexListGraph& ng = const_cast<VertexListGraph&>(g);
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    breadth_first_search(
      ng,
      boost::graph::keywords::_root_vertex = s,
      boost::graph::keywords::_visitor = arg_pack[
        boost::graph::keywords::_visitor ||
        boost::value_factory<default_bfs_visitor>()
      ],
      boost::graph::keywords::_color_map = arg_pack[
        boost::graph::keywords::_color_map |
        make_shared_array_property_map(
          num_vertices(ng),
          white_color,
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_index_map_or_dummy_property_map(ng)
          ]
        )
      ],
      boost::graph::keywords::_buffer = arg_pack[
        boost::graph::keywords::_buffer ||
        boost::value_factory<
          boost::queue<
            typename graph_traits<VertexListGraph>::vertex_descriptor
          >
        >()
      ]
    );
#else
    typedef typename get_param_type< vertex_color_t, bgl_named_params<P,T,R> >::type C;
    detail::bfs_dispatch<C>::apply(ng, s, params,
                                   get_param(params, vertex_color));
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS
  }

  // This version does not initialize colors, user has to.

  template <class IncidenceGraph, class P, class T, class R>
  void breadth_first_visit
    (const IncidenceGraph& g,
     typename graph_traits<IncidenceGraph>::vertex_descriptor s,
     const bgl_named_params<P, T, R>& params)
  {
    // The graph is passed by *const* reference so that graph adaptors
    // (temporaries) can be passed into this function. However, the
    // graph is not really const since we may write to property maps
    // of the graph.
    IncidenceGraph& ng = const_cast<IncidenceGraph&>(g);
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    breadth_first_visit(
      ng,
      boost::graph::keywords::_root_vertex = s,
      boost::graph::keywords::_buffer = arg_pack[
        boost::graph::keywords::_buffer ||
        boost::value_factory<
          boost::queue<
            typename graph_traits<IncidenceGraph>::vertex_descriptor
          >
        >()
      ],
      boost::graph::keywords::_visitor = arg_pack[
        boost::graph::keywords::_visitor ||
        boost::value_factory<default_bfs_visitor>()
      ],
      boost::graph::keywords::_color_map = arg_pack[
        boost::graph::keywords::_color_map |
        make_shared_array_property_map(
          num_vertices(ng),
          white_color,
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_index_map_or_dummy_property_map(ng)
          ]
        )
      ]
    );
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    typedef graph_traits<IncidenceGraph> Traits;
    // Buffer default
    typedef typename Traits::vertex_descriptor vertex_descriptor;
    typedef boost::queue<vertex_descriptor> queue_t;
    queue_t Q;

    breadth_first_visit
      (ng, s,
       choose_param(get_param(params, buffer_param_t()), boost::ref(Q)).get(),
       choose_param(get_param(params, graph_visitor),
                    make_bfs_visitor(null_visitor())),
       choose_pmap(get_param(params, vertex_color), ng, vertex_color)
       );
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS
  }

#if !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
  namespace graph {
    namespace detail {
      template <typename Graph, typename Source>
      struct breadth_first_search_impl {
        typedef void result_type;
        template <typename ArgPack>
        void operator()(const Graph& g, const Source& source, const ArgPack& arg_pack) {
          using namespace boost::graph::keywords;
          typename boost::graph_traits<Graph>::vertex_descriptor sources[1] = {source};
          boost::queue<typename boost::graph_traits<Graph>::vertex_descriptor> Q;
          boost::breadth_first_search(g,
                                      &sources[0],
                                      &sources[1], 
                                      boost::unwrap_ref(arg_pack[_buffer | boost::ref(Q)]),
                                      arg_pack[_visitor | make_bfs_visitor(null_visitor())],
                                      boost::detail::make_color_map_from_arg_pack(g, arg_pack));
        }
      };
    }
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(breadth_first_search, 2, 4)
  }
#endif  // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
} // namespace boost

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/breadth_first_search.hpp>)

#endif // BOOST_GRAPH_BREADTH_FIRST_SEARCH_HPP

