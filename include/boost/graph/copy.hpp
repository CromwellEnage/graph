//
//============================================================================
// Copyright 1997-2001 University of Notre Dame.
// Authors: Jeremy G. Siek, Lie-Quan Lee, Andrew Lumsdaine
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================
//

/*
    This file implements the following functions:

    template <typename VertexListGraph, typename MutableGraph>
    void copy_graph(const VertexListGraph& g_in, MutableGraph& g_out)

    template <typename VertexListGraph, typename MutableGraph,
              typename P, typename T, typename R>
    void copy_graph(const VertexListGraph& g_in, MutableGraph& g_out,
                    const bgl_named_params<P, T, R>& params)

    template <typename IncidenceGraph, typename MutableGraph>
    typename graph_traits<MutableGraph>::vertex_descriptor
    copy_component(IncidenceGraph& g_in,
                   typename graph_traits<IncidenceGraph>::vertex_descriptor s,
                   MutableGraph& g_out)

    template <typename IncidenceGraph, typename MutableGraph,
              typename P, typename T, typename R>
    typename graph_traits<MutableGraph>::vertex_descriptor
    copy_component(IncidenceGraph& g_in,
                   typename graph_traits<IncidenceGraph>::vertex_descriptor s,
                   MutableGraph& g_out,
                   const bgl_named_params<P, T, R>& params)
*/

#ifndef BOOST_GRAPH_COPY_HPP
#define BOOST_GRAPH_COPY_HPP

namespace boost { namespace detail {

    template <int Version>
    struct copy_graph_impl
    {
    };

    // Hack to make transpose_graph work with the same interface as before
    template <typename Graph, typename Desc>
    struct remove_reverse_edge_descriptor
    {
        typedef Desc type;

        inline static Desc convert(const Desc& d, const Graph&)
        {
            return d;
        }
    };

    // Add a reverse_graph_edge_descriptor wrapper if the Graph is a
    // reverse_graph but the edge descriptor is from the original graph.
    // (This case comes from the fact that transpose_graph uses reverse_graph
    // internally but doesn't expose the different edge descriptor type to the
    // user.)
    template <typename Desc, typename Graph>
    struct add_reverse_edge_descriptor
    {
        typedef Desc type;

        inline static Desc convert(const Desc& d)
        {
            return d;
        }
    };

    //-------------------------------------------------------------------------
    struct choose_copier_parameter
    {
        template <typename P, typename G1, typename G2>
        struct bind_
        {
            typedef const P& result_type;

            inline static result_type apply(const P& p, const G1&, G2&)
            {
                return p;
            }
        };
    };

    template <typename Param>
    struct choose_edge_copy
    {
        typedef choose_copier_parameter type;
    };

    template <typename Param>
    struct choose_vertex_copy
    {
        typedef choose_copier_parameter type;
    };
}} // namespace boost::detail

#include <boost/graph/reverse_graph.hpp>

namespace boost { namespace detail {

    template <typename Graph, typename Desc>
    struct remove_reverse_edge_descriptor<
        Graph,
        reverse_graph_edge_descriptor<Desc>
    >
    {
        typedef Desc type;

        inline static Desc
        convert(const reverse_graph_edge_descriptor<Desc>& d, const Graph& g)
        {
            return get(edge_underlying, g, d);
        }
    };

    template <typename Desc, typename G, typename GR>
    struct add_reverse_edge_descriptor<Desc, boost::reverse_graph<G, GR> >
    {
        typedef reverse_graph_edge_descriptor<Desc> type;

        inline static reverse_graph_edge_descriptor<Desc>
        convert(const Desc& d)
        {
            return reverse_graph_edge_descriptor<Desc>(d);
        }
    };

    template <typename Desc, typename G, typename GR>
    struct add_reverse_edge_descriptor<
        reverse_graph_edge_descriptor<Desc>,
        boost::reverse_graph<G, GR>
    >
    {
        typedef reverse_graph_edge_descriptor<Desc> type;

        inline static reverse_graph_edge_descriptor<Desc>
        convert(const reverse_graph_edge_descriptor<Desc>& d)
        {
            return d;
        }
    };
}} // namespace boost::detail

#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

namespace boost { namespace detail {

    // Default edge and vertex property copiers

    template <typename Graph1, typename Graph2>
    class edge_copier
    {
        typename property_map<Graph1, edge_all_t>::const_type edge_all_map1;
        mutable typename property_map<Graph2, edge_all_t>::type edge_all_map2;

    public:
        inline edge_copier(const Graph1& g1, Graph2& g2) :
            edge_all_map1(get(edge_all, g1)),
            edge_all_map2(get(edge_all, g2))
        {
        }

        template <typename Edge1>
        inline void operator()(
            const Edge1& e1,
            const typename graph_traits<Graph2>::edge_descriptor& e2
        ) const
        {
            put(
                this->edge_all_map2,
                e2,
                get(
                    this->edge_all_map1,
                    add_reverse_edge_descriptor<Edge1, Graph1>::convert(e1)
                )
            );
        }

        template <typename Edge1>
        inline void operator()(
            const Edge1& e1,
            typename graph_traits<Graph2>::edge_descriptor& e2
        ) const
        {
            put(
                this->edge_all_map2,
                e2,
                get(
                    this->edge_all_map1,
                    add_reverse_edge_descriptor<Edge1, Graph1>::convert(e1)
                )
            );
        }
    };

    template <typename Graph1, typename Graph2>
    inline edge_copier<Graph1, Graph2>
    make_edge_copier(const Graph1& g1, Graph2& g2)
    {
        return edge_copier<Graph1, Graph2>(g1, g2);
    }

    template <typename Graph1, typename Graph2>
    class edge_copier_t
    {
        const Graph1& _g1;
        Graph2& _g2;

    public:
        typedef edge_copier<Graph1,Graph2> result_type;

        inline edge_copier_t(const Graph1& g1, Graph2& g2) : _g1(g1), _g2(g2)
        {
        }

        inline result_type operator()() const
        {
            return result_type(this->_g1, this->_g2);
        }
    };

    template <typename Graph1, typename Graph2>
    inline edge_copier_t<Graph1, Graph2>
    make_edge_copier_t(const Graph1& g1, Graph2& g2)
    {
        return edge_copier_t<Graph1, Graph2>(g1, g2);
    }

    template <typename Graph1, typename Graph2>
    class vertex_copier
    {
        typename property_map<
            Graph1, vertex_all_t
        >::const_type vertex_all_map1;
        mutable typename property_map<
            Graph2, vertex_all_t
        >::type vertex_all_map2;

    public:
        inline vertex_copier(const Graph1& g1, Graph2& g2) :
            vertex_all_map1(get(vertex_all, g1)),
            vertex_all_map2(get(vertex_all, g2))
        {
        }

        inline void operator()(
            const typename graph_traits<Graph1>::vertex_descriptor& v1,
            const typename graph_traits<Graph2>::vertex_descriptor& v2
        ) const
        {
            put(this->vertex_all_map2, v2, get(this->vertex_all_map1, v1));
        }

        inline void operator()(
            const typename graph_traits<Graph1>::vertex_descriptor& v1,
            typename graph_traits<Graph2>::vertex_descriptor& v2
        ) const
        {
            put(this->vertex_all_map2, v2, get(this->vertex_all_map1, v1));
        }
    };

    template <typename Graph1, typename Graph2>
    inline vertex_copier<Graph1, Graph2>
    make_vertex_copier(const Graph1& g1, Graph2& g2)
    {
        return vertex_copier<Graph1, Graph2>(g1, g2);
    }

    template <typename Graph1, typename Graph2>
    class vertex_copier_t
    {
        const Graph1& _g1;
        Graph2& _g2;

    public:
        typedef vertex_copier<Graph1, Graph2> result_type;

        inline vertex_copier_t(const Graph1& g1, Graph2& g2)
          : _g1(g1), _g2(g2)
        {
        }

        inline result_type operator()() const
        {
            return result_type(this->_g1, this->_g2);
        }
    };

    template <typename Graph1, typename Graph2>
    inline vertex_copier_t<Graph1, Graph2>
    make_vertex_copier_t(const Graph1& g1, Graph2& g2)
    {
        return vertex_copier_t<Graph1, Graph2>(g1, g2);
    }
}} // namespace boost::detail

#include <boost/tuple/tuple.hpp>

namespace boost { namespace detail {

    // Copy all the vertices and edges of graph g_in into graph g_out.
    // The copy_vertex and copy_edge function objects control how vertex
    // and edge properties are copied.

    template <> struct copy_graph_impl<0>
    {
      template <typename Graph, typename MutableGraph, 
        typename CopyVertex, typename CopyEdge, typename IndexMap,
        typename Orig2CopyVertexIndexMap>
      static void apply(const Graph& g_in, MutableGraph& g_out, 
                        CopyVertex copy_vertex, CopyEdge copy_edge,
                        Orig2CopyVertexIndexMap orig2copy, IndexMap)
      {
        typedef remove_reverse_edge_descriptor<Graph, typename graph_traits<Graph>::edge_descriptor> cvt;
        typename graph_traits<Graph>::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(g_in); vi != vi_end; ++vi) {
          typename graph_traits<MutableGraph>::vertex_descriptor
            new_v = add_vertex(g_out);
          put(orig2copy, *vi, new_v);
          copy_vertex(*vi, new_v);
        }
        typename graph_traits<Graph>::edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(g_in); ei != ei_end; ++ei) {
          typename graph_traits<MutableGraph>::edge_descriptor new_e;
          bool inserted;
          boost::tie(new_e, inserted) = add_edge(get(orig2copy, source(*ei, g_in)), 
                                                 get(orig2copy, target(*ei, g_in)),
                                                 g_out);
          copy_edge(cvt::convert(*ei, g_in), new_e);
        }
      }
    };

    // for directed graphs
    template <> struct copy_graph_impl<1>
    {
      template <typename Graph, typename MutableGraph, 
        typename CopyVertex, typename CopyEdge, typename IndexMap,
        typename Orig2CopyVertexIndexMap>
      static void apply(const Graph& g_in, MutableGraph& g_out, 
                        CopyVertex copy_vertex, CopyEdge copy_edge,
                        Orig2CopyVertexIndexMap orig2copy, IndexMap)
      {
        typedef remove_reverse_edge_descriptor<Graph, typename graph_traits<Graph>::edge_descriptor> cvt;
        typename graph_traits<Graph>::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(g_in); vi != vi_end; ++vi) {
          typename graph_traits<MutableGraph>::vertex_descriptor
            new_v = add_vertex(g_out);
          put(orig2copy, *vi, new_v);
          copy_vertex(*vi, new_v);
        }
        for (boost::tie(vi, vi_end) = vertices(g_in); vi != vi_end; ++vi) {
          typename graph_traits<Graph>::out_edge_iterator ei, ei_end;
          for (boost::tie(ei, ei_end) = out_edges(*vi, g_in); ei != ei_end; ++ei) {
            typename graph_traits<MutableGraph>::edge_descriptor new_e;
            bool inserted;
            boost::tie(new_e, inserted) = add_edge(get(orig2copy, source(*ei, g_in)), 
                                                   get(orig2copy, target(*ei, g_in)),
                                                   g_out);
            copy_edge(cvt::convert(*ei, g_in), new_e);
          }
        }
      }
    };
}} // namespace boost::detail

#include <boost/graph/properties.hpp>
#include <vector>

namespace boost { namespace detail {

    // for undirected graphs
    template <> struct copy_graph_impl<2>
    {
      template <typename Graph, typename MutableGraph, 
        typename CopyVertex, typename CopyEdge, typename IndexMap,
        typename Orig2CopyVertexIndexMap>
      static void apply(const Graph& g_in, MutableGraph& g_out, 
                        CopyVertex copy_vertex, CopyEdge copy_edge,
                        Orig2CopyVertexIndexMap orig2copy,
                        IndexMap index_map)
      {
        typedef remove_reverse_edge_descriptor<Graph, typename graph_traits<Graph>::edge_descriptor> cvt;
        typedef color_traits<default_color_type> Color;
        std::vector<default_color_type> 
          color(num_vertices(g_in), Color::white());
        typename graph_traits<Graph>::vertex_iterator vi, vi_end;
        for (boost::tie(vi, vi_end) = vertices(g_in); vi != vi_end; ++vi) {
          typename graph_traits<MutableGraph>::vertex_descriptor
            new_v = add_vertex(g_out);
          put(orig2copy, *vi, new_v);
          copy_vertex(*vi, new_v);
        }
        for (boost::tie(vi, vi_end) = vertices(g_in); vi != vi_end; ++vi) {
          typename graph_traits<Graph>::out_edge_iterator ei, ei_end;
          for (boost::tie(ei, ei_end) = out_edges(*vi, g_in); ei != ei_end; ++ei) {
            typename graph_traits<MutableGraph>::edge_descriptor new_e;
            bool inserted;
            if (color[get(index_map, target(*ei, g_in))] == Color::white()) {
              boost::tie(new_e, inserted) = add_edge(get(orig2copy, source(*ei,g_in)),
                                                     get(orig2copy, target(*ei,g_in)),
                                                     g_out);
              copy_edge(cvt::convert(*ei, g_in), new_e);
            }
          }
          color[get(index_map, *vi)] = Color::black();
        }
      }
    };
}} // namespace boost::detail

#include <boost/type_traits/is_convertible.hpp>

namespace boost { namespace detail {

    template <typename Graph>
    class choose_graph_copy
    {
        typedef typename graph_traits<Graph>::traversal_category Trv;
        typedef typename graph_traits<Graph>::directed_category Dr;
        enum {
            algo = (
                (
                    boost::is_convertible<Trv, vertex_list_graph_tag>::value
                 && boost::is_convertible<Trv, edge_list_graph_tag>::value
                ) ? 0 : boost::is_convertible<Dr, directed_tag>::value ? 1 : 2
            )
        };

    public:
        typedef copy_graph_impl<algo> type;
    };

    struct choose_default_edge_copier
    {
        template <typename P, typename G1, typename G2>
        struct bind_
        {
            typedef edge_copier<G1, G2> result_type;

            inline static result_type apply(const P&, const G1& g1, G2& g2)
            {
                return result_type(g1, g2);
            }
        };
    };

    struct choose_default_vertex_copier
    {
        template <typename P, typename G1, typename G2>
        struct bind_
        {
            typedef vertex_copier<G1, G2> result_type;

            inline static result_type apply(const P&, const G1& g1, G2& g2)
            {
                return result_type(g1, g2);
            }
        };
    };
}} // namespace boost::detail

#include <boost/graph/named_function_params.hpp>

namespace boost { namespace detail {

    template <>
    struct choose_edge_copy<param_not_found>
    {
        typedef choose_default_edge_copier type;
    };

    template <typename Param, typename G1, typename G2>
    struct choose_edge_copier_helper
    {
        typedef typename choose_edge_copy<Param>::type Selector;
        typedef typename Selector:: template bind_<Param, G1, G2> Bind;
        typedef Bind type;
        typedef typename Bind::result_type result_type;
    };

    template <typename Param, typename G1, typename G2>
    typename detail::choose_edge_copier_helper<Param,G1,G2>::result_type
    choose_edge_copier(const Param& params, const G1& g_in, G2& g_out)
    {
        typedef typename 
        detail::choose_edge_copier_helper<Param,G1,G2>::type Choice;
        return Choice::apply(params, g_in, g_out);
    }

    template <>
    struct choose_vertex_copy<param_not_found>
    {
        typedef choose_default_vertex_copier type;
    };

    template <typename Param, typename G1, typename G2>
    struct choose_vertex_copier_helper
    {
        typedef typename choose_vertex_copy<Param>::type Selector;
        typedef typename Selector:: template bind_<Param, G1, G2> Bind;
        typedef Bind type;
        typedef typename Bind::result_type result_type;
    };

    template <typename Param, typename G1, typename G2>
    typename detail::choose_vertex_copier_helper<Param,G1,G2>::result_type
    choose_vertex_copier(const Param& params, const G1& g_in, G2& g_out)
    {
        typedef typename 
        detail::choose_vertex_copier_helper<Param,G1,G2>::type Choice;
        return Choice::apply(params, g_in, g_out);
    }
}} // namespace boost::detail

#include <boost/graph/breadth_first_search.hpp>

namespace boost { namespace detail {

    template <class NewGraph, class Copy2OrigIndexMap, 
      class CopyVertex, class CopyEdge>
    struct graph_copy_visitor : public bfs_visitor<>
    {
      graph_copy_visitor(NewGraph& graph, Copy2OrigIndexMap c,
                         CopyVertex cv, CopyEdge ce)
        : g_out(graph), orig2copy(c), copy_vertex(cv), copy_edge(ce) { }

      template <class Vertex, class Graph>
      typename graph_traits<NewGraph>::vertex_descriptor copy_one_vertex(Vertex u) const {
        typename graph_traits<NewGraph>::vertex_descriptor
          new_u = add_vertex(g_out);
        put(orig2copy, u, new_u);
        copy_vertex(u, new_u);
        return new_u;
      }
      
      template <class Edge, class Graph>
      void tree_edge(Edge e, const Graph& g_in) const {
        // For a tree edge, the target vertex has not been copied yet.
        typename graph_traits<NewGraph>::edge_descriptor new_e;
        bool inserted;
        boost::tie(new_e, inserted) = add_edge(get(orig2copy, source(e, g_in)), 
                                               this->copy_one_vertex(target(e, g_in)),
                                               g_out);
        copy_edge(e, new_e);
      }
      
      template <class Edge, class Graph>
      void non_tree_edge(Edge e, const Graph& g_in) const {
        // For a non-tree edge, the target vertex has already been copied.
        typename graph_traits<NewGraph>::edge_descriptor new_e;
        bool inserted;
        boost::tie(new_e, inserted) = add_edge(get(orig2copy, source(e, g_in)), 
                                               get(orig2copy, target(e, g_in)),
                                               g_out);
        copy_edge(e, new_e);
      }
    private:
      NewGraph& g_out;
      Copy2OrigIndexMap orig2copy;
      CopyVertex copy_vertex;
      CopyEdge copy_edge;
    };
}} // namespace boost::detail

#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/type_traits/is_base_of.hpp>

namespace boost { namespace detail {

    template <
        typename Graph, typename MutableGraph, typename CopyVertex,
        typename CopyEdge, typename Orig2Copy, typename VertexIndexMap
    >
    typename graph_traits<MutableGraph>::vertex_descriptor
    copy_component_impl(
        const Graph& g_in,
        typename graph_traits<Graph>::vertex_descriptor src,
        MutableGraph& g_out,
        CopyVertex copy_vertex,
        CopyEdge copy_edge,
        Orig2Copy orig2copy,
        VertexIndexMap v_i_map,
        typename boost::disable_if<
            boost::is_base_of<
                detail::bgl_named_params_base, VertexIndexMap
            >,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        graph_copy_visitor<
            MutableGraph, Orig2Copy, CopyVertex, CopyEdge
        > vis(g_out, orig2copy, copy_vertex, copy_edge);
        typename graph_traits<
            MutableGraph
        >::vertex_descriptor src_copy = vis.copy_one_vertex(src);
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        breadth_first_search(g_in, src, vis, v_i_map);
#else
        breadth_first_search(
            g_in,
            src,
            boost::graph::keywords::_visitor = vis,
            boost::graph::keywords::_vertex_index_map = v_i_map
        );
#endif
        return src_copy;
    }
}} // namespace boost::detail

#include <boost/type_traits/remove_const.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS) && ( \
        !defined(BOOST_NO_CXX11_DECLTYPE) \
    )
#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/preprocessor.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace boost { namespace graph {

    BOOST_PARAMETER_FUNCTION(
        (bool), copy_graph, ::boost::graph::keywords::tag,
        (required
            (graph, *(boost::detail::argument_predicate<is_bgl_graph>))
        )
        (deduced
            (required
                (result, *(boost::detail::argument_predicate<is_bgl_graph>))
            )
            (optional
                (vertex_copy
                  , *(boost::detail::binary_function_vertex_predicate<>)
                  , boost::detail::make_vertex_copier(graph, result)
                )
                (edge_copy
                  , *(boost::detail::binary_function_edge_predicate<>)
                  , boost::detail::make_edge_copier(graph, result)
                )
                (vertex_index_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_to_integer_map_of_graph
                        >
                    )
                  , boost::detail::vertex_or_dummy_property_map(
                        graph,
                        vertex_index
                    )
                )
                (orig_to_copy
                  , *(boost::detail::orig_to_copy_vertex_map_predicate)
                  , make_shared_array_property_map(
                        num_vertices(graph),
                        boost::detail::get_null_vertex(result),
                        vertex_index_map
                    )
                )
            )
        )
    )
    {
        if (0 < num_vertices(graph))
        {
            typedef typename boost::detail::choose_graph_copy<
                typename boost::remove_const<
                    typename boost::remove_reference<graph_type>::type
                >::type
            >::type copy_impl;
            typename boost::remove_const<
                typename boost::remove_reference<vertex_copy_type>::type
            >::type v_copy = vertex_copy;
            typename boost::remove_const<
                typename boost::remove_reference<edge_copy_type>::type
            >::type e_copy = edge_copy;
            typename boost::remove_const<
                typename boost::remove_reference<orig_to_copy_type>::type
            >::type orig2copy = orig_to_copy;
            copy_impl::apply(
                graph,
                result,
                v_copy,
                e_copy,
                orig2copy,
                vertex_index_map
            );
        }

        return true;
    }

    // Copy all the vertices and edges of graph that are reachable
    // from the source vertex into result.  Return the vertex
    // in result that matches the source vertex of graph.
    BOOST_PARAMETER_FUNCTION(
        (
            boost::lazy_enable_if<
                typename mpl::has_key<
                    Args,
                    boost::graph::keywords::tag::result
                >::type,
                boost::detail::graph_vertex<
                    Args,
                    boost::graph::keywords::tag::result
                >
            >
        ), copy_component, ::boost::graph::keywords::tag,
        (required
            (graph, *(boost::detail::argument_predicate<is_bgl_graph>))
        )
        (deduced
            (required
                (root_vertex
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_of_graph
                        >
                    )
                )
                (result, *(boost::detail::argument_predicate<is_bgl_graph>))
            )
            (optional
                (vertex_copy
                  , *(boost::detail::binary_function_vertex_predicate<>)
                  , boost::detail::make_vertex_copier(graph, result)
                )
                (edge_copy
                  , *(boost::detail::binary_function_edge_predicate<>)
                  , boost::detail::make_edge_copier(graph, result)
                )
                (vertex_index_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_to_integer_map_of_graph
                        >
                    )
                  , boost::detail::vertex_or_dummy_property_map(
                        graph,
                        vertex_index
                    )
                )
                (orig_to_copy
                  , *(boost::detail::orig_to_copy_vertex_map_predicate)
                  , make_shared_array_property_map(
                        num_vertices(graph),
                        boost::detail::get_null_vertex(result),
                        vertex_index_map
                    )
                )
            )
        )
    )
    {
        typename boost::remove_const<
            typename boost::remove_reference<vertex_copy_type>::type
        >::type v_copy = vertex_copy;
        typename boost::remove_const<
            typename boost::remove_reference<edge_copy_type>::type
        >::type e_copy = edge_copy;
        typename boost::remove_const<
            typename boost::remove_reference<orig_to_copy_type>::type
        >::type orig2copy = orig_to_copy;
        return boost::detail::copy_component_impl(
            graph,
            root_vertex,
            result,
            v_copy,
            e_copy,
            orig2copy,
            vertex_index_map
        );
    }
}} // namespace boost::graph

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>

namespace boost { namespace graph {

    template <typename VertexListGraph, typename MutableGraph, typename Args>
    void copy_graph(
        const VertexListGraph& g_in,
        MutableGraph& g_out,
        const Args& args,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        if (num_vertices(g_in) == 0) return;
        typedef typename boost::detail::choose_graph_copy<
            VertexListGraph
        >::type copy_impl;
        typename boost::remove_const<
            typename parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::vertex_copy,
                boost::detail::vertex_copier_t<VertexListGraph, MutableGraph>
            >::type
        >::type v_copy = args[
            boost::graph::keywords::_vertex_copy ||
            boost::detail::make_vertex_copier_t(g_in, g_out)
        ];
        typename boost::remove_const<
            typename parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::edge_copy,
                boost::detail::edge_copier_t<VertexListGraph, MutableGraph>
            >::type
        >::type e_copy = args[
            boost::graph::keywords::_edge_copy ||
            boost::detail::make_edge_copier_t(g_in, g_out)
        ];
        typedef typename graph_traits<MutableGraph>::vertex_descriptor MV;
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::orig_to_copy,
            MV
        > orig2copy_gen(graph_traits<MutableGraph>::null_vertex());
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::orig_to_copy,
            MV
        >::map_type orig2copy = orig2copy_gen(g_in, args);
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            VertexListGraph
        >::type v_i_map = boost::detail::override_const_property(
            args,
            boost::graph::keywords::_vertex_index_map,
            g_in,
            vertex_index
        );
        copy_impl::apply(
            g_in,
            g_out,
            v_copy,
            e_copy,
            orig2copy,
            v_i_map
        );
    }

    // Copy all the vertices and edges of g_in that are reachable
    // from the source vertex into g_out.  Return the vertex
    // in g_out that matches the source vertex of g_in.
    template <typename IncidenceGraph, typename MutableGraph, typename Args>
    typename graph_traits<MutableGraph>::vertex_descriptor copy_component(
        const IncidenceGraph& g_in,
        typename graph_traits<IncidenceGraph>::vertex_descriptor src,
        MutableGraph& g_out,
        const Args& args,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename boost::remove_const<
            typename parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::vertex_copy,
                boost::detail::vertex_copier_t<IncidenceGraph, MutableGraph>
            >::type
        >::type v_copy = args[
            boost::graph::keywords::_vertex_copy ||
            boost::detail::make_vertex_copier_t(g_in, g_out)
        ];
        typename boost::remove_const<
            typename parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::edge_copy,
                boost::detail::edge_copier_t<IncidenceGraph, MutableGraph>
            >::type
        >::type e_copy = args[
            boost::graph::keywords::_edge_copy ||
            boost::detail::make_edge_copier_t(g_in, g_out)
        ];
        typedef typename graph_traits<MutableGraph>::vertex_descriptor MV;
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::orig_to_copy,
            MV
        > orig2copy_gen(graph_traits<MutableGraph>::null_vertex());
        typename boost::detail::map_maker<
            IncidenceGraph,
            Args,
            boost::graph::keywords::tag::orig_to_copy,
            MV
        >::map_type orig2copy = orig2copy_gen(g_in, args);
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            IncidenceGraph
        >::type v_i_map = boost::detail::override_const_property(
            args,
            boost::graph::keywords::_vertex_index_map,
            g_in,
            vertex_index
        );
        return boost::detail::copy_component_impl(
            g_in,
            src,
            g_out,
            v_copy,
            e_copy,
            orig2copy,
            v_i_map
        );
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <typename VertexListGraph, typename MutableGraph>
    inline void copy_graph(const VertexListGraph& g_in, MutableGraph& g_out)
    {
        copy_graph(g_in, g_out, parameter::compose());
    }

    template <typename IncidenceGraph, typename MutableGraph>
    inline typename graph_traits<MutableGraph>::vertex_descriptor
    copy_component(
        IncidenceGraph& g_in,
        typename graph_traits<IncidenceGraph>::vertex_descriptor src,
        MutableGraph& g_out
    )
    {
        copy_component(g_in, src, g_out, parameter::compose());
    }
}} // namespace boost::graph

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename VertexListGraph, typename MutableGraph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const VertexListGraph& g_in, \
        MutableGraph& g_out, \
        const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, \
            mpl::true_ \
        >::type = mpl::true_() \
    ) \
    { \
        name( \
            g_in, \
            g_out, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(1, 5, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, copy_graph)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename IncidenceGraph, typename MutableGraph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline typename graph_traits<MutableGraph>::vertex_descriptor name( \
        const IncidenceGraph& g_in, \
        typename graph_traits<IncidenceGraph>::vertex_descriptor src, \
        MutableGraph& g_out, \
        const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, \
            mpl::true_ \
        >::type = mpl::true_() \
    ) \
    { \
        return name( \
            g_in, \
            src, \
            g_out, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 5, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, copy_component
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS

namespace boost { namespace detail {

    template <typename Graph, typename MutableGraph,
              typename CopyVertex, typename CopyEdge,
              typename Orig2CopyVertexIndexMap,
              typename P, typename T, typename R>
    typename graph_traits<MutableGraph>::vertex_descriptor
    copy_component_impl
      (const Graph& g_in, 
       typename graph_traits<Graph>::vertex_descriptor src,
       MutableGraph& g_out, 
       CopyVertex copy_vertex, CopyEdge copy_edge,
       Orig2CopyVertexIndexMap orig2copy,
       const bgl_named_params<P, T, R>& params)
    {
      graph_copy_visitor<MutableGraph, Orig2CopyVertexIndexMap, 
        CopyVertex, CopyEdge> vis(g_out, orig2copy, copy_vertex, copy_edge);
      typename graph_traits<MutableGraph>::vertex_descriptor src_copy
        = vis.copy_one_vertex(src);
      breadth_first_search(g_in, src, params.visitor(vis));
      return src_copy;
    }
}} // namespace boost::detail

namespace boost {

    using ::boost::graph::copy_graph;
    using ::boost::graph::copy_component;

  template <typename VertexListGraph, typename MutableGraph, 
    class P, class T, class R>
  void copy_graph(const VertexListGraph& g_in, MutableGraph& g_out, 
                  const bgl_named_params<P, T, R>& params)
  {
    typename std::vector<T>::size_type n;
      n = is_default_param(get_param(params, orig_to_copy_t()))
        ? num_vertices(g_in) : 1;
    if (n == 0)
      return;
    std::vector<BOOST_DEDUCED_TYPENAME graph_traits<MutableGraph>::vertex_descriptor> 
      orig2copy(n);

    typedef typename detail::choose_graph_copy<VertexListGraph>::type 
      copy_impl;
    copy_impl::apply
      (g_in, g_out,
       detail::choose_vertex_copier(get_param(params, vertex_copy_t()), 
                                    g_in, g_out),
       detail::choose_edge_copier(get_param(params, edge_copy_t()), 
                                  g_in, g_out),
       choose_param(get_param(params, orig_to_copy_t()),
                    make_iterator_property_map
                    (orig2copy.begin(), 
                     choose_const_pmap(get_param(params, vertex_index), 
                                 g_in, vertex_index), orig2copy[0])),
       choose_const_pmap(get_param(params, vertex_index), g_in, vertex_index)
       );
  }

  template <typename IncidenceGraph, typename MutableGraph, 
           typename P, typename T, typename R>
  typename graph_traits<MutableGraph>::vertex_descriptor
  copy_component(IncidenceGraph& g_in, 
                 typename graph_traits<IncidenceGraph>::vertex_descriptor src,
                 MutableGraph& g_out, 
                 const bgl_named_params<P, T, R>& params)
  {
    typename std::vector<T>::size_type n;
      n = is_default_param(get_param(params, orig_to_copy_t()))
        ? num_vertices(g_in) : 1;
    std::vector<typename graph_traits<IncidenceGraph>::vertex_descriptor> 
      orig2copy(n);
    
    return detail::copy_component_impl
      (g_in, src, g_out,
       detail::choose_vertex_copier(get_param(params, vertex_copy_t()), 
                                    g_in, g_out),
       detail::choose_edge_copier(get_param(params, edge_copy_t()), 
                                  g_in, g_out),
       choose_param(get_param(params, orig_to_copy_t()),
                    make_iterator_property_map
                    (orig2copy.begin(), 
                     choose_pmap(get_param(params, vertex_index), 
                                 g_in, vertex_index), orig2copy[0])),
       params
       );
  }

} // namespace boost

#endif // BOOST_GRAPH_COPY_HPP
