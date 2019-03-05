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
#ifndef BOOST_GRAPH_NEIGHBOR_BREADTH_FIRST_SEARCH_HPP
#define BOOST_GRAPH_NEIGHBOR_BREADTH_FIRST_SEARCH_HPP

/*
  Neighbor Breadth First Search
  Like BFS, but traverses in-edges as well as out-edges.
  (for directed graphs only. use normal BFS for undirected graphs)
*/
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/config.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
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
    class is_neighbor_bfs_visitor_impl
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
            _check_exam_o_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().examine_out_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().examine_out_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_exam_o_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_exam_i_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().examine_in_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().examine_in_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_exam_i_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_tree_o_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().tree_out_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().tree_out_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_tree_o_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_tree_i_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().tree_in_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().tree_in_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_tree_i_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_n_t_o_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().non_tree_out_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().non_tree_out_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_n_t_o_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_n_t_i_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().non_tree_in_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().non_tree_in_edge(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_n_t_i_e(...);

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
            _check_g_s_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().gray_source(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().gray_source(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_g_s_e(...);

        template <typename B, typename P>
        static graph_yes_tag
            _check_b_s_e(
                mpl::vector<B,P>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::detail::declref<B>().black_source(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    ))
#else
                    decltype(
                        boost::detail::declref<B>().black_source(
                            boost::declval<
                                typename graph_traits<P>::edge_descriptor
                            >(),
                            boost::detail::declcref<P>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_b_s_e(...);

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
                    is_neighbor_bfs_visitor_impl<T,G>::_check_init_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_disc_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_exam_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_exam_o_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_exam_i_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_tree_o_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_tree_i_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_n_t_o_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_n_t_i_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_g_t_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_b_t_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_g_s_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_b_s_e(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_neighbor_bfs_visitor_impl<T,G>::_check_end_v(
                        static_cast<mpl::vector<_m_T,G>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            )
        > type;
    };

    template <typename T, typename G>
    struct is_neighbor_bfs_visitor
      : mpl::eval_if<
        is_bgl_graph<G>,
        is_neighbor_bfs_visitor_impl<T,G>,
        mpl::false_
      >::type
    { };

    typedef argument_with_graph_predicate<
      is_neighbor_bfs_visitor
    > neighbor_bfs_visitor_predicate;
#else   // defined(BOOST_NO_CXX11_DECLTYPE) && !defined(BOOST_TYPEOF_KEYWORD)
    typedef visitor_predicate neighbor_bfs_visitor_predicate;
#endif  // !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
}}
#endif  // unnamed argument deduction and perfect forwarding enabled

#include <vector>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/core/ref.hpp>
#include <boost/concept/assert.hpp>

namespace boost {

  template <class Visitor, class Graph>
  struct NeighborBFSVisitorConcept {
    void constraints() {
      BOOST_CONCEPT_ASSERT(( CopyConstructibleConcept<Visitor> ));
      vis.initialize_vertex(u, g);
      vis.discover_vertex(u, g);
      vis.examine_vertex(u, g);
      vis.examine_out_edge(e, g);
      vis.examine_in_edge(e, g);
      vis.tree_out_edge(e, g);
      vis.tree_in_edge(e, g);
      vis.non_tree_out_edge(e, g);
      vis.non_tree_in_edge(e, g);
      vis.gray_target(e, g);
      vis.black_target(e, g);
      vis.gray_source(e, g);
      vis.black_source(e, g);
      vis.finish_vertex(u, g);
    }
    Visitor vis;
    Graph g;
    typename graph_traits<Graph>::vertex_descriptor u;
    typename graph_traits<Graph>::edge_descriptor e;
  };

  template <class Visitors = null_visitor>
  class neighbor_bfs_visitor {
  public:
    neighbor_bfs_visitor(Visitors vis = Visitors()) : m_vis(vis) { }

    template <class Vertex, class Graph>
    void initialize_vertex(Vertex u, Graph& g) {
      invoke_visitors(m_vis, u, g, on_initialize_vertex());      
    }
    template <class Vertex, class Graph>
    void discover_vertex(Vertex u, Graph& g) {
      invoke_visitors(m_vis, u, g, on_discover_vertex());      
    }
    template <class Vertex, class Graph>
    void examine_vertex(Vertex u, Graph& g) {
      invoke_visitors(m_vis, u, g, on_examine_vertex());
    }
    template <class Edge, class Graph>
    void examine_out_edge(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_examine_edge());
    }
    template <class Edge, class Graph>
    void tree_out_edge(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_tree_edge());      
    }
    template <class Edge, class Graph>
    void non_tree_out_edge(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_non_tree_edge());
    }
    template <class Edge, class Graph>
    void gray_target(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_gray_target());
    }
    template <class Edge, class Graph>
    void black_target(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_black_target());
    }
    template <class Edge, class Graph>
    void examine_in_edge(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_examine_edge());
    }
    template <class Edge, class Graph>
    void tree_in_edge(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_tree_edge());      
    }
    template <class Edge, class Graph>
    void non_tree_in_edge(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_non_tree_edge());
    }
    template <class Edge, class Graph>
    void gray_source(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_gray_target());
    }
    template <class Edge, class Graph>
    void black_source(Edge e, Graph& g) {
      invoke_visitors(m_vis, e, g, on_black_target());
    }
    template <class Vertex, class Graph>
    void finish_vertex(Vertex u, Graph& g) {
      invoke_visitors(m_vis, u, g, on_finish_vertex());      
    }
  protected:
    Visitors m_vis;
  };

  template <class Visitors>
  neighbor_bfs_visitor<Visitors>
  make_neighbor_bfs_visitor(Visitors vis) {
    return neighbor_bfs_visitor<Visitors>(vis);
  }
}

namespace boost { namespace detail {

    template <class BidirectionalGraph, class Buffer, class BFSVisitor, 
              class ColorMap>
    void neighbor_bfs_impl
      (const BidirectionalGraph& g, 
       typename graph_traits<BidirectionalGraph>::vertex_descriptor s, 
       Buffer& Q, BFSVisitor vis, ColorMap color)

    {
      BOOST_CONCEPT_ASSERT(( BidirectionalGraphConcept<BidirectionalGraph> ));
      typedef graph_traits<BidirectionalGraph> GTraits;
      typedef typename GTraits::vertex_descriptor Vertex;
      typedef typename GTraits::edge_descriptor Edge;
      BOOST_CONCEPT_ASSERT(( 
        NeighborBFSVisitorConcept<BFSVisitor, BidirectionalGraph> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<ColorMap, Vertex> ));
      typedef typename property_traits<ColorMap>::value_type ColorValue;
      typedef color_traits<ColorValue> Color;
      
      put(color, s, Color::gray());
      vis.discover_vertex(s, g);
      Q.push(s);
      while (! Q.empty()) {
        Vertex u = Q.top();
        Q.pop(); // pop before push to avoid problem if Q is priority_queue.
        vis.examine_vertex(u, g);

        typename GTraits::out_edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = out_edges(u, g); ei != ei_end; ++ei) {
          Edge e = *ei;
          vis.examine_out_edge(e, g);
          Vertex v = target(e, g);
          ColorValue v_color = get(color, v);
          if (v_color == Color::white()) {
            vis.tree_out_edge(e, g);
            put(color, v, Color::gray());
            vis.discover_vertex(v, g);
            Q.push(v);
          } else {
            vis.non_tree_out_edge(e, g);
            if (v_color == Color::gray())
              vis.gray_target(e, g);
            else
              vis.black_target(e, g);
          }
        } // for out-edges

        typename GTraits::in_edge_iterator in_ei, in_ei_end;
        for (boost::tie(in_ei, in_ei_end) = in_edges(u, g); 
             in_ei != in_ei_end; ++in_ei) {
          Edge e = *in_ei;
          vis.examine_in_edge(e, g);
          Vertex v = source(e, g);
          ColorValue v_color = get(color, v);
          if (v_color == Color::white()) {
            vis.tree_in_edge(e, g);
            put(color, v, Color::gray());
            vis.discover_vertex(v, g);
            Q.push(v);
          } else {
            vis.non_tree_in_edge(e, g);
            if (v_color == Color::gray())
              vis.gray_source(e, g);
            else
              vis.black_source(e, g);
          }
        } // for in-edges

        put(color, u, Color::black());
        vis.finish_vertex(u, g);
      } // while
    }
}}

#include <boost/parameter/config.hpp>
#include <boost/pending/queue.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
#include <boost/parameter/preprocessor.hpp>
#else
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/compose.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#endif

namespace boost {

  // Boost.Parameter-enabled variants
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
  // Boost.Parameter-enabled single-source variants capable of
  // unnamed argument deduction and perfect forwarding
  BOOST_PARAMETER_FUNCTION(
    (bool), neighbor_breadth_first_visit, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_bidirectional_graph>))
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
        (buffer
          ,*(detail::argument_predicate<detail::is_buffer>)
          ,detail::create_empty_queue(root_vertex)
        )
        (visitor
          ,*(detail::neighbor_bfs_visitor_predicate)
          ,neighbor_bfs_visitor<>()
        )
        (vertex_index_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_to_integer_map_of_graph
            >
          )
          ,detail::vertex_or_dummy_property_map(graph, vertex_index)
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
      )
    )
  )
  {
    detail::neighbor_bfs_impl(graph, root_vertex, buffer, visitor, color_map);
    return true;
  }

  BOOST_PARAMETER_FUNCTION(
    (bool), neighbor_breadth_first_search, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_bidirectional_graph>))
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
          ,*(detail::neighbor_bfs_visitor_predicate)
          ,neighbor_bfs_visitor<>()
        )
        (vertex_index_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_to_integer_map_of_graph
            >
          )
          ,detail::vertex_or_dummy_property_map(graph, vertex_index)
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
          ,detail::create_empty_queue(root_vertex)
        )
      )
    )
  )
  {
    typedef typename boost::remove_const<
      typename boost::remove_reference<graph_type>::type
    >::type VertexListGraph;
    typedef typename boost::remove_const<
      typename boost::remove_reference<color_map_type>::type
    >::type ColorMap;
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;

    // Initialization
    typename boost::graph_traits<VertexListGraph>::vertex_iterator i, i_end;

    for (boost::tie(i, i_end) = vertices(graph); i != i_end; ++i) {
      put(color_map, *i, Color::white());
      visitor.initialize_vertex(*i, graph);
    }
    detail::neighbor_bfs_impl(graph, root_vertex, buffer, visitor, color_map);
    return true;
  }
#else   // no unnamed argument deduction or perfect forwarding
  // Boost.Parameter-enabled single-source variants
  template <typename VertexListGraph, typename Args>
  inline void neighbor_breadth_first_visit
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     const Args& args,
     typename boost::enable_if<
       parameter::is_argument_pack<Args>, mpl::true_
     >::type = mpl::true_())
  {
    boost::queue<typename graph_traits<VertexListGraph>::vertex_descriptor> Q;
    neighbor_bfs_visitor<> default_visitor;
    typename boost::detail::map_maker<
        VertexListGraph,
        Args,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type c_map = detail::make_color_map_from_arg_pack(g, args);
    detail::neighbor_bfs_impl(
        g,
        s,
        args[boost::graph::keywords::_buffer | Q],
        args[boost::graph::keywords::_visitor | default_visitor],
        c_map
    );
  }

  template <typename VertexListGraph, typename Args>
  inline void neighbor_breadth_first_search
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     const Args& args,
     typename boost::enable_if<
       parameter::is_argument_pack<Args>, mpl::true_
     >::type = mpl::true_())
  {
    boost::queue<typename graph_traits<VertexListGraph>::vertex_descriptor> Q;
    neighbor_bfs_visitor<> default_visitor;
    typename boost::parameter::binding<
        Args, 
        boost::graph::keywords::tag::visitor,
        neighbor_bfs_visitor<>&
    >::type vis = args[boost::graph::keywords::_visitor | default_visitor];
    typedef typename boost::detail::map_maker<
        VertexListGraph,
        Args,
        boost::graph::keywords::tag::color_map,
        boost::default_color_type
    >::map_type ColorMap;
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;

    // Initialization
    ColorMap c_map = detail::make_color_map_from_arg_pack(g, args);
    typename boost::graph_traits<VertexListGraph>::vertex_iterator i, i_end;

    for (boost::tie(i, i_end) = vertices(g); i != i_end; ++i) {
        put(c_map, *i, Color::white());
        vis.initialize_vertex(*i, g);
    }
    detail::neighbor_bfs_impl(
        g,
        s,
        args[boost::graph::keywords::_buffer | Q],
        vis,
        c_map
    );
  }

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
  template <typename Graph, typename TA \
            BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA)> \
  inline void name \
    (const Graph &g, typename graph_traits<Graph>::vertex_descriptor s, \
     const TA& ta BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
     typename boost::enable_if< \
       parameter::are_tagged_arguments< \
         TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
       >, mpl::true_ \
     >::type = mpl::true_()) \
  { \
    name(g, s, parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta))); \
  }

BOOST_PP_REPEAT_FROM_TO(1, 5, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, neighbor_breadth_first_visit)
BOOST_PP_REPEAT_FROM_TO(1, 5, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, neighbor_breadth_first_search)

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD
#endif  // unnamed argument deduction and perfect forwarding enabled
}
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
namespace boost { namespace detail {

    template <class VertexListGraph, class ColorMap, class BFSVisitor,
      class P, class T, class R>
    void neighbor_bfs_helper
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       ColorMap color, 
       BFSVisitor vis,
       const bgl_named_params<P, T, R>& params)
    {
      typedef graph_traits<VertexListGraph> Traits;
      // Buffer default
      typedef typename Traits::vertex_descriptor Vertex;
      typedef boost::queue<Vertex> queue_t;
      queue_t Q;
      // Initialization
      typedef typename property_traits<ColorMap>::value_type ColorValue;
      typedef color_traits<ColorValue> Color;
      typename boost::graph_traits<VertexListGraph>::vertex_iterator i, i_end;
      for (boost::tie(i, i_end) = vertices(g); i != i_end; ++i) {
        put(color, *i, Color::white());
        vis.initialize_vertex(*i, g);
      }
      neighbor_bfs_impl
        (g, s, 
         choose_param(get_param(params, buffer_param_t()), boost::ref(Q)).get(),
         vis, color);
    }

    //-------------------------------------------------------------------------
    // Choose between default color and color parameters. Using
    // function dispatching so that we don't require vertex index if
    // the color default is not being used.

    template <class ColorMap>
    struct neighbor_bfs_dispatch {
      template <class VertexListGraph, class P, class T, class R>
      static void apply
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       const bgl_named_params<P, T, R>& params,
       ColorMap color)
      {
        neighbor_bfs_helper
          (g, s, color,
           choose_param(get_param(params, graph_visitor),
                        make_neighbor_bfs_visitor(null_visitor())),
           params);
      }
    };

    template <>
    struct neighbor_bfs_dispatch<param_not_found> {
      template <class VertexListGraph, class P, class T, class R>
      static void apply
      (VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       const bgl_named_params<P, T, R>& params,
       param_not_found)
      {
        std::vector<default_color_type> color_vec(num_vertices(g));
        null_visitor null_vis;
        
        neighbor_bfs_helper
          (g, s, 
           make_iterator_property_map
           (color_vec.begin(), 
            choose_const_pmap(get_param(params, vertex_index), 
                              g, vertex_index), color_vec[0]),
           choose_param(get_param(params, graph_visitor),
                        make_neighbor_bfs_visitor(null_vis)),
           params);
      }
    };
}} // namespace boost::detail
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

namespace boost {

  // Named Parameter Variant
  template <class VertexListGraph, class P, class T, class R>
  void neighbor_breadth_first_search
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
    boost::queue<typename graph_traits<VertexListGraph>::vertex_descriptor> Q;
    neighbor_bfs_visitor<> default_visitor;
    neighbor_breadth_first_search(
      ng,
      s,
      boost::graph::keywords::_buffer = arg_pack[
        boost::graph::keywords::_buffer | Q
      ],
      boost::graph::keywords::_visitor = arg_pack[
        boost::graph::keywords::_visitor | default_visitor
      ],
      boost::graph::keywords::_color_map = arg_pack[
        boost::graph::keywords::_color_map |
        make_shared_array_property_map(
          num_vertices(ng),
          white_color,
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_or_dummy_property_map(ng, vertex_index)
          ]
        )
      ]
    );
#else
    typedef typename get_param_type< vertex_color_t, bgl_named_params<P,T,R> >::type C;
    detail::neighbor_bfs_dispatch<C>::apply(ng, s, params, 
                                            get_param(params, vertex_color));
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS
  }

  // This version does not initialize colors, user has to.
  template <class IncidenceGraph, class P, class T, class R>
  void neighbor_breadth_first_visit
    (IncidenceGraph& g,
     typename graph_traits<IncidenceGraph>::vertex_descriptor s,
     const bgl_named_params<P, T, R>& params)
  {
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    boost::queue<typename graph_traits<IncidenceGraph>::vertex_descriptor> Q;
    neighbor_bfs_visitor<> default_visitor;
    neighbor_breadth_first_visit(
      g,
      s,
      boost::graph::keywords::_buffer = arg_pack[
        boost::graph::keywords::_buffer | Q
      ],
      boost::graph::keywords::_visitor = arg_pack[
        boost::graph::keywords::_visitor | default_visitor
      ],
      boost::graph::keywords::_color_map = arg_pack[
        boost::graph::keywords::_color_map |
        make_shared_array_property_map(
          num_vertices(g),
          white_color,
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_or_dummy_property_map(g, vertex_index)
          ]
        )
      ]
    );
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    typedef graph_traits<IncidenceGraph> Traits;
    // Buffer default
    typedef boost::queue<typename Traits::vertex_descriptor> queue_t;
    queue_t Q;

    detail::neighbor_bfs_impl
      (g, s,
       choose_param(get_param(params, buffer_param_t()), boost::ref(Q)).get(),
       choose_param(get_param(params, graph_visitor),
                    make_neighbor_bfs_visitor(null_visitor())),
       choose_pmap(get_param(params, vertex_color), g, vertex_color)
       );
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS
  }
} // namespace boost

#endif // BOOST_GRAPH_NEIGHBOR_BREADTH_FIRST_SEARCH_HPP

