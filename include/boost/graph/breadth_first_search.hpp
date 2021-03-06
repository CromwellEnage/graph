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

namespace boost { namespace graph {

    struct bfs_visitor_event_not_overridden
    {
    };
}} // namespace boost::graph

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/detail/mpi_include.hpp>
#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/concepts.hpp>)

namespace boost {

    template <typename Visitor, typename Graph>
    struct BFSVisitorConcept
    {
        void constraints()
        {
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
}

#include <boost/graph/detail/traits.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/parameter/config.hpp>

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
    {
    };

    typedef argument_with_graph_predicate<
        is_bfs_visitor
    > bfs_visitor_predicate;
#else   // defined(BOOST_NO_CXX11_DECLTYPE) && !defined(BOOST_TYPEOF_KEYWORD)
    typedef visitor_predicate bfs_visitor_predicate;
#endif  // !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
}}
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS

#include <vector>
#include <boost/pending/queue.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/graph/two_bit_color_map.hpp>

namespace boost { namespace graph {

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
}} // namespace boost::graph

#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

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
}} // namespace boost::graph

#include <boost/functional/value_factory.hpp>

#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
#include <boost/parameter/preprocessor.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/parameter/value_type.hpp>
#endif

namespace boost { namespace graph {

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
  // Boost.Parameter-enabled single-source variants
  BOOST_PARAMETER_BASIC_FUNCTION(
    (
      boost::disable_if<
        boost::detail::is_iterator_argument<
          Args,
          boost::graph::keywords::tag::root_vertex
        >,
        bool
      >
    ), breadth_first_visit, ::boost::graph::keywords::tag,
    (required
      (graph, *(boost::detail::argument_predicate<is_vertex_list_graph>))
    )
    (deduced
      (required
        (root_vertex
          ,*(
            boost::detail::argument_with_graph_predicate<
              boost::detail::is_vertex_of_graph
            >
          )
        )
      )
      (optional
        (buffer
          ,*(boost::detail::argument_predicate<boost::detail::is_buffer>)
        )
        (visitor, *(boost::detail::bfs_visitor_predicate))
        (color_map
          ,*(
            boost::detail::argument_with_graph_predicate<
              boost::detail::is_vertex_color_map_of_graph
            >
          )
        )
        (vertex_index_map
          ,*(
            boost::detail::argument_with_graph_predicate<
              boost::detail::is_vertex_to_integer_map_of_graph
            >
          )
        )
      )
    )
  )
  {
    typedef typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::graph
        >::type
    >::type VertexListGraph;
    typename parameter::binding<
        Args,
        boost::graph::keywords::tag::graph
    >::type g = args[boost::graph::keywords::_graph];
    typedef typename graph_traits<
        VertexListGraph
    >::vertex_descriptor Vertex;
    Vertex s = args[boost::graph::keywords::_root_vertex];
    Vertex srcs[1] = {s};
    boost::queue<Vertex> d_buf;
    typename parameter::binding<
        Args,
        boost::graph::keywords::tag::buffer,
        boost::queue<Vertex>&
    >::type Q = args[
        boost::graph::keywords::_buffer ||
        boost::detail::make_reference_generator(d_buf)
    ];
    typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::visitor,
            bfs_visitor<>
        >::type
    >::type vis = args[
        boost::graph::keywords::_visitor ||
        boost::value_factory<bfs_visitor<> >()
    ];
    typename boost::detail::map_maker<
        VertexListGraph,
        Args,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type c_map = boost::detail::make_color_map_from_arg_pack(g, args);
    breadth_first_visit(g, srcs, srcs + 1, Q, vis, c_map);
    return true;
  }

  BOOST_PARAMETER_BASIC_FUNCTION(
    (
      boost::disable_if<
        boost::detail::is_iterator_argument<
          Args,
          boost::graph::keywords::tag::root_vertex
        >,
        bool
      >
    ), breadth_first_search, ::boost::graph::keywords::tag,
    (required
      (graph, *(boost::detail::argument_predicate<is_vertex_list_graph>))
    )
    (deduced
      (required
        (root_vertex
          ,*(
            boost::detail::argument_with_graph_predicate<
              boost::detail::is_vertex_of_graph
            >
          )
        )
      )
      (optional
        (buffer
          ,*(boost::detail::argument_predicate<boost::detail::is_buffer>)
        )
        (visitor, *(boost::detail::bfs_visitor_predicate))
        (color_map
          ,*(
            boost::detail::argument_with_graph_predicate<
              boost::detail::is_vertex_color_map_of_graph
            >
          )
        )
        (vertex_index_map
          ,*(
            boost::detail::argument_with_graph_predicate<
              boost::detail::is_vertex_to_integer_map_of_graph
            >
          )
        )
      )
    )
  )
  {
    typedef typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::graph
        >::type
    >::type VertexListGraph;
    typename parameter::binding<
        Args,
        boost::graph::keywords::tag::graph
    >::type g = args[boost::graph::keywords::_graph];
    typedef typename graph_traits<
        typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::graph
            >::type
        >::type
    >::vertex_descriptor Vertex;
    Vertex s = args[boost::graph::keywords::_root_vertex];
    Vertex srcs[1] = {s};
    boost::queue<Vertex> d_buf;
    typename parameter::binding<
        Args,
        boost::graph::keywords::tag::buffer,
        boost::queue<Vertex>&
    >::type Q = args[
        boost::graph::keywords::_buffer ||
        boost::detail::make_reference_generator(d_buf)
    ];
    typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::visitor,
            bfs_visitor<>
        >::type
    >::type vis = args[
        boost::graph::keywords::_visitor ||
        boost::value_factory<bfs_visitor<> >()
    ];
    typename boost::detail::map_maker<
        VertexListGraph,
        Args,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type c_map = boost::detail::make_color_map_from_arg_pack(g, args);
    breadth_first_search(g, srcs, srcs + 1, Q, vis, c_map);
    return true;
  }
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
  // single-source variants
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
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
}} // namespace boost::graph

namespace boost { namespace detail {

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

#if ( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
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
#endif  // MSVC-14.0 w/64-bit addressing
}} // namespace boost::detail

namespace boost {

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
#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    typedef typename graph_traits<
        VertexListGraph
    >::vertex_descriptor Vertex;
    boost::queue<Vertex> d_buf;
    typename parameter::binding<
        arg_pack_type,
        boost::graph::keywords::tag::buffer,
        boost::queue<Vertex>&
    >::type Q = arg_pack[
        boost::graph::keywords::_buffer ||
        boost::detail::make_reference_generator(d_buf)
    ];
    typename boost::remove_const<
        typename parameter::value_type<
            arg_pack_type,
            boost::graph::keywords::tag::visitor,
            boost::graph::bfs_visitor<>
        >::type
    >::type vis = arg_pack[
        boost::graph::keywords::_visitor ||
        boost::value_factory<boost::graph::bfs_visitor<> >()
    ];
    typename boost::detail::map_maker<
        VertexListGraph,
        arg_pack_type,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type c_map = boost::detail::make_color_map_from_arg_pack(
        g,
        arg_pack
    );
    breadth_first_search(ng, s, Q, vis, c_map);
#else
    typedef typename get_param_type< vertex_color_t, bgl_named_params<P,T,R> >::type C;
    detail::bfs_dispatch<C>::apply(ng, s, params,
                                   get_param(params, vertex_color));
#endif  // not MSVC-14.0 w/64-bit addressing
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
#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    typedef typename graph_traits<
        IncidenceGraph
    >::vertex_descriptor Vertex;
    boost::queue<Vertex> d_buf;
    typename parameter::binding<
        arg_pack_type,
        boost::graph::keywords::tag::buffer,
        boost::queue<Vertex>&
    >::type Q = arg_pack[
        boost::graph::keywords::_buffer ||
        boost::detail::make_reference_generator(d_buf)
    ];
    typename boost::remove_const<
        typename parameter::value_type<
            arg_pack_type,
            boost::graph::keywords::tag::visitor,
            boost::graph::bfs_visitor<>
        >::type
    >::type vis = arg_pack[
        boost::graph::keywords::_visitor ||
        boost::value_factory<boost::graph::bfs_visitor<> >()
    ];
    typename boost::detail::map_maker<
        IncidenceGraph,
        arg_pack_type,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type c_map = boost::detail::make_color_map_from_arg_pack(
        g,
        arg_pack
    );
    breadth_first_visit(ng, s, Q, vis, c_map);
#else   // MSVC-14.0 w/64-bit addressing
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
#endif  // not MSVC-14.0 w/64-bit addressing
  }
} // namespace boost

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost { namespace graph { namespace detail {

    template <typename Graph, typename Source>
    struct breadth_first_visit_impl
    {
        typedef void result_type;
        typedef result_type type;

        template <typename ArgPack>
        void operator()(
            const Graph& g, const Source& source, const ArgPack& arg_pack
        )
        {
            typedef typename boost::graph_traits<Graph>::vertex_descriptor V;
            V sources[1] = {source};
            boost::queue<V> Q;
            breadth_first_visit(
                g,
                &sources[0],
                &sources[1],
                arg_pack[
                    boost::graph::keywords::_buffer ||
                    boost::detail::make_reference_generator(Q)
                ],
                arg_pack[
                    boost::graph::keywords::_visitor ||
                    boost::value_factory<bfs_visitor<> >()
                ],
                boost::detail::make_color_map_from_arg_pack(g, arg_pack)
            );
        }
    };

    template <typename Graph, typename Source>
    struct breadth_first_search_impl
    {
        typedef void result_type;
        typedef result_type type;

        template <typename ArgPack>
        void operator()(
            const Graph& g, const Source& source, const ArgPack& arg_pack
        )
        {
            typedef typename boost::graph_traits<Graph>::vertex_descriptor V;
            V sources[1] = {source};
            boost::queue<V> Q;
            breadth_first_search(
                g,
                &sources[0],
                &sources[1],
                arg_pack[
                    boost::graph::keywords::_buffer ||
                    boost::detail::make_reference_generator(Q)
                ],
                arg_pack[
                    boost::graph::keywords::_visitor ||
                    boost::value_factory<bfs_visitor<> >()
                ],
                boost::detail::make_color_map_from_arg_pack(g, arg_pack)
            );
        }
    };
}}} // namespace boost::graph::detail

namespace boost { namespace graph {

    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(breadth_first_visit, 2, 6)
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(breadth_first_search, 2, 6)
}} // namespace boost::graph

#endif  // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost {

    using ::boost::graph::breadth_first_visit;
    using ::boost::graph::breadth_first_search;
    using ::boost::graph::bfs_visitor;
    using ::boost::graph::make_bfs_visitor;
    using ::boost::graph::default_bfs_visitor;
} // namespace boost

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/breadth_first_search.hpp>)

#endif // BOOST_GRAPH_BREADTH_FIRST_SEARCH_HPP

