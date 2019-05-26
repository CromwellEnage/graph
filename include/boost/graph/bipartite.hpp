/**
 *
 * Copyright (c) 2010 Matthias Walter (xammy@xammy.homelinux.net)
 *
 * Authors: Matthias Walter
 *
 * Distributed under the Boost Software License, Version 1.0. (See
 * accompanying file LICENSE_1_0.txt or copy at
 * http://www.boost.org/LICENSE_1_0.txt)
 *
 */

#ifndef BOOST_GRAPH_BIPARTITE_HPP
#define BOOST_GRAPH_BIPARTITE_HPP

#include <utility>
#include <vector>
#include <exception>
#include <boost/graph/properties.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/bind.hpp>
#include <boost/core/enable_if.hpp>

#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
#include <boost/parameter/preprocessor.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#else
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#endif

namespace boost { namespace detail {

    /**
     * The bipartite_visitor_error is thrown if an edge cannot be colored.
     * The witnesses are the edges incident vertices.
     */

    template <typename Vertex>
    struct BOOST_SYMBOL_VISIBLE bipartite_visitor_error : std::exception
    {
        std::pair<Vertex, Vertex> witnesses;

        bipartite_visitor_error(Vertex a, Vertex b) : witnesses (a, b)
        {
        }

        const char* what() const throw()
        {
            return "Graph is not bipartite.";
        }
    };

    /**
     * Functor which colors edges to be non-monochromatic.
     */

    template <typename PartitionMap>
    class bipartition_colorize
    {
        PartitionMap partition_map_;

    public:
        typedef on_tree_edge event_filter;

        bipartition_colorize(PartitionMap partition_map) :
            partition_map_(partition_map)
        {
        }

        template <typename Edge, typename Graph>
        inline void operator()(Edge e, const Graph& g)
        {
            typedef color_traits<
                typename property_traits<PartitionMap>::value_type
            > color_traits;

            put(
                this->partition_map_,
                target(e, g),
                (
                    get(this->partition_map_, source(e, g))
                ) ? color_traits::white() : color_traits::black()
            );
        }
    };

    /**
     * Creates a bipartition_colorize functor which colors edges
     * to be non-monochromatic.
     *
     * @param partition_map Color map for the bipartition
     * @return The functor.
     */

    template <typename PartitionMap>
    inline bipartition_colorize<PartitionMap>
    colorize_bipartition(PartitionMap partition_map)
    {
        return bipartition_colorize<PartitionMap>(partition_map);
    }

    /**
     * Functor which tests an edge to be monochromatic.
     */

    template <typename PartitionMap>
    class bipartition_check
    {
        PartitionMap partition_map_;

    public:
        typedef on_back_edge event_filter;

        bipartition_check(PartitionMap partition_map) :
            partition_map_(partition_map)
        {
        }

        template <typename Edge, typename Graph>
        void operator()(Edge e, const Graph& g)
        {
            typedef typename graph_traits<
                Graph
            >::vertex_descriptor vertex_descriptor_t;

            vertex_descriptor_t source_vertex = source(e, g);
            vertex_descriptor_t target_vertex = target(e, g);

            if (
                get(
                    this->partition_map_,
                    source_vertex
                ) == get(this->partition_map_, target_vertex)
            )
            {
                throw bipartite_visitor_error<
                    vertex_descriptor_t
                >(source_vertex, target_vertex);
            }
        }
    };

    /**
     * Creates a bipartition_check functor which raises an error if a
     * monochromatic edge is found.
     *
     * @param partition_map The map for a bipartition.
     * @return The functor.
     */

    template <typename PartitionMap>
    inline bipartition_check<PartitionMap>
    check_bipartition(PartitionMap partition_map)
    {
        return bipartition_check<PartitionMap>(partition_map);
    }

    /**
     * Find the beginning of a common suffix of two sequences
     * 
     * @param sequence1 Pair of bidirectional iterators defining the first sequence.
     * @param sequence2 Pair of bidirectional iterators defining the second sequence.
     * @return Pair of iterators pointing to the beginning of the common suffix.
     */

    template <typename BiDirectionalIterator1, typename BiDirectionalIterator2>
    inline std::pair<BiDirectionalIterator1, BiDirectionalIterator2>
    reverse_mismatch(
        std::pair<BiDirectionalIterator1, BiDirectionalIterator1> sequence1,
        std::pair<BiDirectionalIterator2, BiDirectionalIterator2> sequence2
    )
    {
        if (
            (sequence1.first == sequence1.second) ||
            (sequence2.first == sequence2.second)
        )
        {
            return std::make_pair(sequence1.first, sequence2.first);
        }

        BiDirectionalIterator1 iter1 = sequence1.second;
        BiDirectionalIterator2 iter2 = sequence2.second;

        for (;;)
        {
            --iter1;
            --iter2;

            if (*iter1 != *iter2)
            {
                ++iter1;
                ++iter2;
                break;
            }

            if (iter1 == sequence1.first)
                break;
            if (iter2 == sequence2.first)
                break;
        }

        return std::make_pair(iter1, iter2);
    }
}} // namespace boost::detail

namespace boost { namespace graph {

    /**
     * Checks a given graph for bipartiteness and fills the given color map
     * with white and black according to the bipartition.  If the graph is
     * not bipartite, the contents of the color map are undefined.  Runs in
     * linear time in the size of the graph, if access to the property maps
     * is in constant time.
     *
     * @param graph The given graph.
     * @param index_map An index map associating vertices with an index.
     * @param partition_map A color map to fill with the bipartition.
     * @return true if and only if the given graph is bipartite.
     */
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    BOOST_PARAMETER_FUNCTION(
        (bool), is_bipartite, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_vertex_list_graph>)
            )
        )
        (deduced
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
                (partition_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_color_map_of_graph
                        >
                    )
                  , make_one_bit_color_map(
                        num_vertices(graph),
                        vertex_index_map
                    )
                )
            )
        )
    )
#elif !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
    BOOST_PARAMETER_FUNCTION(
        (bool), is_bipartite, ::boost::graph::keywords::tag,
        (required
            (graph, *)
        )
        (optional
            (vertex_index_map
              , *
              , boost::detail::vertex_or_dummy_property_map(
                    graph,
                    vertex_index
                )
            )
            (partition_map
              , *
              , make_one_bit_color_map(num_vertices(graph), vertex_index_map)
            )
        )
    )
#else   // MSVC-14.0 w/64-bit addressing
    template <
        typename graph_type, typename IndexMap, typename partition_map_type
    >
    inline typename boost::disable_if<
        parameter::are_tagged_arguments<IndexMap, partition_map_type>,
        bool
    >::type
    is_bipartite(
        const graph_type& graph, const IndexMap vertex_index_map,
        partition_map_type partition_map
    )
#endif
    {
        /// General types and variables
        typedef typename property_traits<
            typename boost::remove_const<
                typename boost::remove_reference<partition_map_type>::type
            >::type
        >::value_type partition_color_t;
        typedef typename graph_traits<
            typename boost::remove_const<
                typename boost::remove_reference<graph_type>::type
            >::type
        >::vertex_descriptor vertex_descriptor_t;

        /// Declare dfs visitor
        //      detail::empty_recorder recorder;
        //      typedef detail::bipartite_visitor<
        //          PartitionMap, detail::empty_recorder
        //      > dfs_visitor_t;
        //      dfs_visitor_t dfs_visitor(partition_map, recorder);

        /// Call dfs
        try
        {
            depth_first_search(
                graph,
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
                vertex_index_map,
#else
                boost::graph::keywords::_vertex_index_map = vertex_index_map,
                boost::graph::keywords::_visitor =
#endif
                make_dfs_visitor(
                    std::make_pair(
                        boost::detail::colorize_bipartition(partition_map),
                        std::make_pair(
                            boost::detail::check_bipartition(partition_map),
                            put_property(
                                partition_map,
                                color_traits<partition_color_t>::white(),
                                on_start_vertex()
                            )
                        )
                    )
                )
            );
        }
        catch (
            const boost::detail::bipartite_visitor_error<vertex_descriptor_t>&
        )
        {
            return false;
        }

        return true;
    }

    /**
     * Checks a given graph for bipartiteness and fills a given color map with
     * white and black according to the bipartition.  If the graph is not
     * bipartite, a sequence of vertices, producing an odd-cycle, is written
     * to the output iterator.  The final iterator value is returned.  Runs in
     * linear time in the size of the graph, if access to the property maps is
     * in constant time.
     *
     * @param graph The given graph.
     * @param index_map An index map associating vertices with an index.
     * @param partition_map A color map to fill with the bipartition.
     * @param result An iterator to write the odd-cycle vertices to.
     * @return The final iterator value after writing.
     */
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
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
        ), find_odd_cycle, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_vertex_list_graph>)
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
                (partition_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_color_map_of_graph
                        >
                    )
                  , make_one_bit_color_map(
                        num_vertices(graph),
                        vertex_index_map
                    )
                )
            )
        )
    )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    template <
        typename Graph, typename IndexMap, typename PartitionMap,
        typename OutputIterator
    >
    typename boost::disable_if<
        parameter::are_tagged_arguments<PartitionMap, OutputIterator>,
        OutputIterator
    >::type
    find_odd_cycle(
        const Graph& graph, const IndexMap vertex_index_map,
        PartitionMap partition_map, OutputIterator result
    )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
    {
        /// General types and variables
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        typedef typename boost::remove_const<
            typename boost::remove_reference<graph_type>::type
        >::type Graph;
        typedef typename boost::remove_const<
            typename boost::remove_reference<result_type>::type
        >::type OutputIterator;
        typedef typename boost::remove_const<
            typename boost::remove_reference<vertex_index_map_type>::type
        >::type IndexMap;
        typedef typename boost::remove_const<
            typename boost::remove_reference<partition_map_type>::type
        >::type PartitionMap;
#endif
        typedef typename property_traits<
            PartitionMap
        >::value_type partition_color_t;
        typedef typename graph_traits<
            Graph
        >::vertex_descriptor vertex_descriptor_t;
        typedef typename graph_traits<
            Graph
        >::vertex_iterator vertex_iterator_t;
        vertex_iterator_t vertex_iter, vertex_end;

        /// Declare predecessor map
        typedef std::vector<vertex_descriptor_t> predecessors_t;
        typedef iterator_property_map<
            typename predecessors_t::iterator, IndexMap, vertex_descriptor_t,
            vertex_descriptor_t&
        > predecessor_map_t;

        predecessors_t predecessors(
            num_vertices(graph), graph_traits<Graph>::null_vertex()
        );
        predecessor_map_t predecessor_map(
            predecessors.begin(), vertex_index_map
        );

        /// Initialize predecessor map
        for (
            boost::tie(vertex_iter, vertex_end) = vertices(graph);
            vertex_iter != vertex_end;
            ++vertex_iter
        )
        {
            put(predecessor_map, *vertex_iter, *vertex_iter);
        }

        /// Call dfs
        try
        {
            depth_first_search(
                graph,
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
                vertex_index_map,
#else
                boost::graph::keywords::_vertex_index_map = vertex_index_map,
                boost::graph::keywords::_visitor =
#endif
                make_dfs_visitor(
                    std::make_pair(
                        boost::detail::colorize_bipartition(partition_map),
                        std::make_pair(
                            boost::detail::check_bipartition(partition_map),
                            std::make_pair(
                                put_property(
                                    partition_map,
                                    color_traits<partition_color_t>::white(),
                                    on_start_vertex()
                                ),
                                record_predecessors(
                                    predecessor_map,
                                    on_tree_edge()
                                )
                            )
                        )
                    )
                )
            );
        }
        catch (
            const boost::detail::bipartite_visitor_error<
                vertex_descriptor_t
            >& error
        )
        {
            typedef std::vector<vertex_descriptor_t> path_t;

            path_t path1, path2;
            vertex_descriptor_t next, current;

            /// First path
            next = error.witnesses.first;

            do
            {
                current = next;
                path1.push_back(current);
                next = get(predecessor_map, current);
            }
            while (current != next);

            /// Second path
            next = error.witnesses.second;

            do
            {
                current = next;
                path2.push_back(current);
                next = get(predecessor_map, current);
            }
            while (current != next);

            /// Find beginning of common suffix
            std::pair<
                typename path_t::iterator,
                typename path_t::iterator
            > mismatch = boost::detail::reverse_mismatch(
                std::make_pair(path1.begin(), path1.end()),
                std::make_pair(path2.begin(), path2.end())
            );

            /// Copy the odd-length cycle
            OutputIterator r = result;
            r = std::copy(path1.begin(), mismatch.first + 1, r);
            return std::reverse_copy(path2.begin(), mismatch.second, r);
        }

        return result;
    }
}} // namespace boost::graph

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost { namespace graph {

    template <typename Graph, typename IndexMap, typename OutputIterator>
    typename boost::disable_if<
        parameter::is_argument_pack<OutputIterator>,
        OutputIterator
    >::type
    find_odd_cycle(
        const Graph& graph, const IndexMap index_map, OutputIterator result
    )
    {
        typedef one_bit_color_map<IndexMap> partition_map_t;
        partition_map_t partition_map(num_vertices(graph), index_map);
        return find_odd_cycle(graph, index_map, partition_map, result);
    }
}} // namespace boost::graph

#if ( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )

namespace boost { namespace graph { namespace detail {

    template <typename Graph>
    struct is_bipartite_impl
    {
        typedef bool result_type;
        typedef result_type type;

        template <typename ArgPack>
        inline bool operator()(const Graph& g, const ArgPack& arg_pack) const
        {
            typename boost::detail::override_const_property_result<
                ArgPack,
                boost::graph::keywords::tag::vertex_index_map,
                vertex_index_t,
                Graph
            >::type v_i_map = boost::detail::override_const_property(
                arg_pack,
                boost::graph::keywords::_vertex_index_map,
                g,
                vertex_index
            );
            boost::detail::make_property_map_from_arg_pack_gen<
                boost::graph::keywords::tag::partition_map,
                default_color_type
            > part_map_gen(white_color);
            typename boost::detail::map_maker<
                Graph,
                ArgPack,
                boost::graph::keywords::tag::partition_map,
                default_color_type
            >::map_type part_map = part_map_gen(g, arg_pack);
            return is_bipartite(g, v_i_map, part_map);
        }
    };
}}} // namespace boost::graph::detail

namespace boost { namespace graph {

    // Boost.Parameter-enabled variant
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(is_bipartite, 1, 3)

    /**
     * Checks a given graph for bipartiteness.  Runs in linear time in the
     * size of the graph, if access to the property maps is in constant time.
     *
     * @param graph The given graph.
     * @param index_map An index map associating vertices with an index.
     * @return true if and only if the given graph is bipartite.
     */

    template <typename Graph, typename IndexMap>
    inline typename boost::disable_if<
        parameter::is_argument_pack<IndexMap>,
        bool
    >::type
    is_bipartite(const Graph& graph, const IndexMap index_map)
    {
        typedef one_bit_color_map<IndexMap> partition_map_t;
        partition_map_t partition_map(num_vertices(graph), index_map);
        return is_bipartite(graph, index_map, partition_map);
    }
}} // namespace boost::graph

#endif  // MSVC-14.0 w/64-bit addressing

namespace boost { namespace graph { namespace detail {

    template <typename Graph, typename OutputIterator>
    struct find_odd_cycle_impl
    {
        typedef OutputIterator result_type;
        typedef result_type type;

        template <typename ArgPack>
        inline OutputIterator operator()(
            const Graph& g, OutputIterator result, const ArgPack& arg_pack
        ) const
        {
            typename boost::detail::override_const_property_result<
                ArgPack,
                boost::graph::keywords::tag::vertex_index_map,
                vertex_index_t,
                Graph
            >::type v_i_map = boost::detail::override_const_property(
                arg_pack,
                boost::graph::keywords::_vertex_index_map,
                g,
                vertex_index
            );
            boost::detail::make_property_map_from_arg_pack_gen<
                boost::graph::keywords::tag::partition_map,
                default_color_type
            > part_map_gen(white_color);
            typename boost::detail::map_maker<
                Graph,
                ArgPack,
                boost::graph::keywords::tag::partition_map,
                default_color_type
            >::map_type part_map = part_map_gen(g, arg_pack);
            return find_odd_cycle(g, v_i_map, part_map, result);
        }
    };
}}} // namespace boost::graph::detail

namespace boost { namespace graph {

    // Boost.Parameter-enabled variant
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(find_odd_cycle, 2, 4)
}} // namespace boost::graph

#endif  // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost {

    using ::boost::graph::is_bipartite;
    using ::boost::graph::find_odd_cycle;
}

#endif /// BOOST_GRAPH_BIPARTITE_HPP
