//============================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Copyright 2004, 2005 Trustees of Indiana University
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek,
//          Doug Gregor, D. Kevin McGrath
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================
#ifndef BOOST_GRAPH_CUTHILL_MCKEE_HPP
#define BOOST_GRAPH_CUTHILL_MCKEE_HPP

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <algorithm>
#include <functional>

/*
  (Reverse) Cuthill-McKee Algorithm for matrix reordering
*/

namespace boost { namespace detail {

    template <typename OutputIterator, typename Buffer, typename DegreeMap>
    struct bfs_rcm_visitor : public default_bfs_visitor
    {
        inline bfs_rcm_visitor(
            OutputIterator& iter, Buffer& b, DegreeMap deg
        ) : default_bfs_visitor(), permutation(iter), index_begin(0), Qref(b),
            degree(deg)
        {
        }

        inline bfs_rcm_visitor(bfs_rcm_visitor const& copy) :
            default_bfs_visitor(), permutation(copy.permutation),
            index_begin(copy.index_begin), Qref(copy.Qref),
            degree(copy.degree)
        {
        }

        template <typename Vertex, typename Graph>
        inline void examine_vertex(Vertex u, Graph&)
        {
            *this->permutation++ = u;
            this->index_begin = this->Qref.size();
        }

        template <typename Vertex, typename Graph>
        void finish_vertex(Vertex, Graph&)
        {
            using std::sort;
            typedef typename property_traits<DegreeMap>::value_type ds_type;
            typedef indirect_cmp<DegreeMap, std::less<ds_type> > Compare;
            Compare comp(this->degree);
            sort(
                this->Qref.begin() + this->index_begin, this->Qref.end(), comp
            );
        }

    protected:
        OutputIterator& permutation;
        typename Buffer::size_type index_begin;
        Buffer& Qref;
        DegreeMap degree;
    };
}} // namespace boost::detail

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/sparse_ordering.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/core/enable_if.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/parameter/preprocessor.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#endif

namespace boost { namespace graph {

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    // If user provides a reverse iterator, this will be a reverse
    // Cuthill-McKee algorithm, otherwise it will be a standard CM algorithm.
    BOOST_PARAMETER_FUNCTION(
        (
            boost::lazy_enable_if<
                typename mpl::has_key<
                    Args,
                    boost::graph::keywords::tag::result
                >::type,
                boost::detail::mutable_value_type<
                    Args,
                    boost::graph::keywords::tag::result
                >
            >
        ), cuthill_mckee_ordering, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_bgl_graph>)
            )
        )
        (deduced
            (required
                (result
                  , *(
                        boost::detail::argument_predicate<
                            boost::detail::is_iterator
                        >
                    )
                )
                (color_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_color_map_of_graph
                        >
                    )
                )
                (degree_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_to_integer_map_of_graph
                        >
                    )
                )
            )
            (optional
                (root_vertex
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_of_graph
                        >
                    )
                  , boost::detail::get_null_vertex(graph)
                )
                (buffer
                  , *(
                        boost::detail::argument_predicate<
                            boost::detail::has_container_typedefs
                        >
                    )
                  , boost::sparse::make_ordering_default_queue_t(
                        graph,
                        root_vertex,
                        color_map,
                        degree_map
                    )()
                )
            )
        )
    )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    // Reverse Cuthill-McKee algorithm with a given queue
    // of starting vertices.
    template <
        typename Graph, typename Buffer, typename OutputIterator,
        typename ColorMap, typename DegreeMap
    >
    typename boost::enable_if<
        boost::detail::has_container_typedefs<Buffer>,
        OutputIterator
    >::type
    cuthill_mckee_ordering(
        const Graph& graph, Buffer& buffer, OutputIterator result,
        ColorMap color_map, DegreeMap degree_map
    )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
    {
        if (boost::graph::has_no_vertices(graph)) return result;

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        typedef typename boost::remove_const<
            typename boost::remove_reference<graph_type>::type
        >::type Graph;
        typedef typename boost::remove_const<
            typename boost::remove_reference<result_type>::type
        >::type OutputIterator;
        typedef typename boost::remove_const<
            typename boost::remove_reference<color_map_type>::type
        >::type ColorMap;
        typedef typename boost::remove_const<
            typename boost::remove_reference<degree_map_type>::type
        >::type DegreeMap;
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
        // create queue, visitor...don't forget namespaces!
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typedef boost::sparse::sparse_ordering_queue<Vertex> Queue;
        typedef boost::detail::bfs_rcm_visitor<
            OutputIterator, Queue, DegreeMap
        > Visitor;
        typedef color_traits<
            typename property_traits<ColorMap>::value_type
        > Color;

        Queue Q;

        // create a bfs_rcm_visitor as defined above
        Visitor vis(result, Q, degree_map);

        typename graph_traits<Graph>::vertex_iterator ui, ui_end;    

        // Copy degree to pseudo_degree
        // initialize the color map
        for (boost::tie(ui, ui_end) = vertices(graph); ui != ui_end; ++ui)
        {
            put(color_map, *ui, Color::white());
        }

        while (!buffer.empty())
        {
            Vertex s = buffer.front();
            buffer.pop_front();

            // call BFS with visitor
            breadth_first_visit(graph, s, Q, vis, color_map);
        }

        return result;
    }
}} // namespace boost::graph

#include <boost/graph/detail/out_degree_property_map.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost { namespace graph {

    BOOST_PARAMETER_FUNCTION(
        (
            boost::lazy_enable_if<
                typename mpl::has_key<
                    Args,
                    boost::graph::keywords::tag::result
                >::type,
                boost::detail::mutable_value_type<
                    Args,
                    boost::graph::keywords::tag::result
                >
            >
        ), cuthill_mckee_ordering, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_bgl_graph>)
            )
        )
        (deduced
            (required
                (result
                  , *(
                        boost::detail::argument_predicate<
                            boost::detail::is_iterator
                        >
                    )
                )
            )
            (optional
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
            )
        )
    )
    {
        return cuthill_mckee_ordering(
            graph,
            result,
            make_shared_array_property_map(
                num_vertices(graph),
                white_color,
                vertex_index_map
            ),
            make_out_degree_map(graph)
        );
    }
}} // namespace boost::graph

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

#include <deque>

namespace boost { namespace graph {

    // Reverse Cuthill-McKee algorithm with a given starting vertex.
    template <
        typename Graph, typename OutputIterator,
        typename ColorMap, typename DegreeMap
    >
    OutputIterator cuthill_mckee_ordering(
        const Graph& graph,
        typename graph_traits<Graph>::vertex_descriptor root_vertex,
        OutputIterator result, ColorMap color_map, DegreeMap degree_map
    )
    {
        std::deque<
            typename graph_traits<Graph>::vertex_descriptor
        > Q = boost::sparse::make_ordering_default_queue_t(
            graph, root_vertex, color_map, degree_map
        )();
        return cuthill_mckee_ordering(
            graph, Q, result, color_map, degree_map
        );
    }

    // Reverse Cuthill-McKee algorithm with a default starting vertex.
    template <
        typename Graph, typename OutputIterator,
        typename ColorMap, typename DegreeMap
    >
    inline OutputIterator cuthill_mckee_ordering(
        const Graph& graph, OutputIterator result,
        ColorMap color_map, DegreeMap degree_map
    )
    {
        return cuthill_mckee_ordering(
            graph, graph_traits<Graph>::null_vertex(),
            result, color_map, degree_map
        );
    }
}} // namespace boost::graph

#include <vector>

namespace boost { namespace graph {

    // Reverse Cuthill-McKee algorithm with default color and degree maps.
    template <
        typename Graph, typename OutputIterator, typename VertexIndexMap
    >
    OutputIterator cuthill_mckee_ordering(
        const Graph& G, OutputIterator permutation, VertexIndexMap index_map
    )
    {
        if (boost::graph::has_no_vertices(G)) return permutation;
        std::vector<default_color_type> colors(num_vertices(G));
        return cuthill_mckee_ordering(
            G, permutation,
            make_iterator_property_map(
                colors.begin(), index_map, white_color
            ),
            make_out_degree_map(G)
        );
    }

    // Reverse Cuthill-McKee algorithm with a default index map.
    template <typename Graph, typename OutputIterator>
    inline OutputIterator
    cuthill_mckee_ordering(const Graph& G, OutputIterator permutation)
    {
        return cuthill_mckee_ordering(G, permutation, get(vertex_index, G));
    }
}} // namespace boost::graph

#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS

namespace boost {

    using ::boost::graph::cuthill_mckee_ordering;
} // namespace boost

#endif  // BOOST_GRAPH_CUTHILL_MCKEE_HPP

