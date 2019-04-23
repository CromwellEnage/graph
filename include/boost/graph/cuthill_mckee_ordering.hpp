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
    struct bfs_rcm_visitor : public boost::graph::default_bfs_visitor
    {
        inline bfs_rcm_visitor(
            OutputIterator& iter, Buffer& b, DegreeMap deg
        ) : boost::graph::default_bfs_visitor(), permutation(iter),
            index_begin(0), Qref(b), degree(deg)
        {
        }

        inline bfs_rcm_visitor(bfs_rcm_visitor const& copy) :
            boost::graph::default_bfs_visitor(),
            permutation(copy.permutation), index_begin(copy.index_begin),
            Qref(copy.Qref), degree(copy.degree)
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
#include <deque>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/parameter/preprocessor.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/remove_const.hpp>
#endif

namespace boost { namespace graph {

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    // If user provides a reverse iterator, this will be a reverse
    // Cuthill-McKee algorithm, otherwise it will be a standard CM algorithm.
    BOOST_PARAMETER_BASIC_FUNCTION(
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
                )
                (buffer
                  , *(
                        boost::detail::argument_predicate<
                            boost::detail::has_container_typedefs
                        >
                    )
                )
            )
        )
    )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    // Reverse Cuthill-McKee algorithm with a given queue
    // of starting vertices.
    template <
        typename Graph, typename VertexQueue, typename OutputIterator,
        typename ColorMap, typename DegreeMap
    >
    typename boost::enable_if<
        boost::detail::has_container_typedefs<VertexQueue>,
        OutputIterator
    >::type
    cuthill_mckee_ordering(
        const Graph& g, VertexQueue& vertex_queue, OutputIterator permutation,
        ColorMap color, DegreeMap degree
    )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
    {
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        typedef typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::graph
            >::type
        >::type Graph;
        const Graph& g = args[boost::graph::keywords::_graph];
        typedef typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::result
            >::type
        >::type OutputIterator;
        OutputIterator permutation = args[boost::graph::keywords::_result];
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS

        if (boost::graph::has_no_vertices(g)) return permutation;

        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        Vertex root_vertex = args[
            boost::graph::keywords::_root_vertex |
            graph_traits<Graph>::null_vertex()
        ];
        typedef typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::color_map
            >::type
        >::type ColorMap;
        ColorMap color = args[boost::graph::keywords::_color_map];
        typedef typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::degree_map
            >::type
        >::type DegreeMap;
        DegreeMap degree = args[boost::graph::keywords::_degree_map];
        std::deque<Vertex> default_vertex_queue;
        typename parameter::binding<
            Args,
            boost::graph::keywords::tag::buffer,
            std::deque<Vertex>
        >::type vertex_queue = args[
            boost::graph::keywords::_buffer ||
            boost::sparse::make_ordering_default_queue_reference(
                g, root_vertex, default_vertex_queue, color, degree
            )
        ];
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
        typedef boost::sparse::sparse_ordering_queue<Vertex> Queue;
        typedef boost::detail::bfs_rcm_visitor<
            OutputIterator, Queue, DegreeMap
        > Visitor;
        typedef color_traits<
            typename property_traits<ColorMap>::value_type
        > Color;

        Queue Q;

        // create a bfs_rcm_visitor as defined above
        Visitor vis(permutation, Q, degree);

        typename graph_traits<Graph>::vertex_iterator ui, ui_end;    

        // Copy degree to pseudo_degree
        // initialize the color map
        for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui)
        {
            put(color, *ui, Color::white());
        }

        while (!vertex_queue.empty())
        {
            Vertex s = vertex_queue.front();
            vertex_queue.pop_front();

            // call BFS with visitor
            breadth_first_visit(g, s, Q, vis, color);
        }

        return permutation;
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
        > vertex_queue;
        boost::sparse::initialize_ordering_default_queue_and_maps(
            graph, root_vertex, vertex_queue, color_map, degree_map
        );
        return cuthill_mckee_ordering(
            graph, vertex_queue, result, color_map, degree_map
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

