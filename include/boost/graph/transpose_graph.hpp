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
#ifndef BOOST_GRAPH_TRANSPOSE_HPP
#define BOOST_GRAPH_TRANSPOSE_HPP

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <boost/graph/copy.hpp>
#include <boost/graph/detail/traits.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS) && ( \
        !defined(BOOST_NO_CXX11_DECLTYPE) \
    )
#include <boost/parameter/preprocessor.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace boost { namespace graph {

    BOOST_PARAMETER_FUNCTION(
        (bool), transpose_graph, ::boost::graph::keywords::tag,
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
        copy_graph(
            reverse_graph<
                typename boost::remove_const<
                    typename boost::remove_reference<graph_type>::type
                >::type
            >(graph),
            result,
            vertex_copy,
            edge_copy,
            vertex_index_map,
            boost::graph::keywords::_orig_to_copy = orig_to_copy
        );
        return true;
    }
}} // namespace boost::graph

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    template <typename VertexListGraph, typename MutableGraph, typename Args>
    inline void transpose_graph(
        const VertexListGraph& g_in,
        MutableGraph& g_out,
        const Args& args,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        reverse_graph<VertexListGraph> R(g_in);
        copy_graph(R, g_out, args);
    }
}} // namespace boost::graph

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/compose.hpp>
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

BOOST_PP_REPEAT_FROM_TO(
    1, 5, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, transpose_graph
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost { namespace graph {

    template <typename VertexListGraph, typename MutableGraph> 
    inline void transpose_graph(const VertexListGraph& G, MutableGraph& G_T)
    {
        reverse_graph<VertexListGraph> R(G);
        copy_graph(R, G_T);
    }
}} // namespace boost::graph

#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS

namespace boost {

    using ::boost::graph::transpose_graph;

  template <class VertexListGraph, class MutableGraph, 
    class P, class T, class R> 
  void transpose_graph(const VertexListGraph& G, MutableGraph& G_T,
                       const bgl_named_params<P, T, R>& params)
  {
    reverse_graph<VertexListGraph> Rev(G);
    copy_graph(Rev, G_T, params);
  }
} // namespace boost

#endif // BOOST_GRAPH_TRANSPOSE_HPP
