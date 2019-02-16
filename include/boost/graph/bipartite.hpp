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
#include <boost/parameter/preprocessor.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/bind.hpp>

namespace boost {

  namespace detail {

    /**
     * The bipartite_visitor_error is thrown if an edge cannot be colored.
     * The witnesses are the edges incident vertices.
     */

    template <typename Vertex>
    struct BOOST_SYMBOL_VISIBLE bipartite_visitor_error: std::exception
    {
      std::pair <Vertex, Vertex> witnesses;

      bipartite_visitor_error (Vertex a, Vertex b) :
        witnesses (a, b)
      {

      }

      const char* what () const throw ()
      {
        return "Graph is not bipartite.";
      }
    };

    /**
     * Functor which colors edges to be non-monochromatic.
     */

    template <typename PartitionMap>
    struct bipartition_colorize
    {
      typedef on_tree_edge event_filter;

      bipartition_colorize (PartitionMap partition_map) :
        partition_map_ (partition_map)
      {

      }

      template <typename Edge, typename Graph>
      void operator() (Edge e, const Graph& g)
      {
        typedef typename graph_traits <Graph>::vertex_descriptor vertex_descriptor_t;
        typedef color_traits <typename property_traits <PartitionMap>::value_type> color_traits;

        vertex_descriptor_t source_vertex = source (e, g);
        vertex_descriptor_t target_vertex = target (e, g);
        if (get (partition_map_, source_vertex) == color_traits::white ())
          put (partition_map_, target_vertex, color_traits::black ());
        else
          put (partition_map_, target_vertex, color_traits::white ());
      }

    private:
      PartitionMap partition_map_;
    };

    /**
     * Creates a bipartition_colorize functor which colors edges
     * to be non-monochromatic.
     *
     * @param partition_map Color map for the bipartition
     * @return The functor.
     */

    template <typename PartitionMap>
    inline bipartition_colorize <PartitionMap> colorize_bipartition (PartitionMap partition_map)
    {
      return bipartition_colorize <PartitionMap> (partition_map);
    }

    /**
     * Functor which tests an edge to be monochromatic.
     */

    template <typename PartitionMap>
    struct bipartition_check
    {
      typedef on_back_edge event_filter;

      bipartition_check (PartitionMap partition_map) :
        partition_map_ (partition_map)
      {

      }

      template <typename Edge, typename Graph>
      void operator() (Edge e, const Graph& g)
      {
        typedef typename graph_traits <Graph>::vertex_descriptor vertex_descriptor_t;

        vertex_descriptor_t source_vertex = source (e, g);
        vertex_descriptor_t target_vertex = target (e, g);
        if (get (partition_map_, source_vertex) == get (partition_map_, target_vertex))
          throw bipartite_visitor_error <vertex_descriptor_t> (source_vertex, target_vertex);
      }

    private:
      PartitionMap partition_map_;
    };

    /**
     * Creates a bipartition_check functor which raises an error if a
     * monochromatic edge is found.
     *
     * @param partition_map The map for a bipartition.
     * @return The functor.
     */

    template <typename PartitionMap>
    inline bipartition_check <PartitionMap> check_bipartition (PartitionMap partition_map)
    {
      return bipartition_check <PartitionMap> (partition_map);
    }

    /**
     * Find the beginning of a common suffix of two sequences
     * 
     * @param sequence1 Pair of bidirectional iterators defining the first sequence.
     * @param sequence2 Pair of bidirectional iterators defining the second sequence.
     * @return Pair of iterators pointing to the beginning of the common suffix.
     */

    template <typename BiDirectionalIterator1, typename BiDirectionalIterator2>
    inline std::pair <BiDirectionalIterator1, BiDirectionalIterator2> reverse_mismatch (std::pair <
        BiDirectionalIterator1, BiDirectionalIterator1> sequence1, std::pair <BiDirectionalIterator2,
        BiDirectionalIterator2> sequence2)
    {
      if (sequence1.first == sequence1.second || sequence2.first == sequence2.second)
        return std::make_pair (sequence1.first, sequence2.first);

      BiDirectionalIterator1 iter1 = sequence1.second;
      BiDirectionalIterator2 iter2 = sequence2.second;

      while (true)
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

      return std::make_pair (iter1, iter2);
    }

  }

  /**
   * Checks a given graph for bipartiteness and fills the given color map with
   * white and black according to the bipartition. If the graph is not
   * bipartite, the contents of the color map are undefined. Runs in linear
   * time in the size of the graph, if access to the property maps is in
   * constant time.
   *
   * @param graph The given graph.
   * @param index_map An index map associating vertices with an index.
   * @param partition_map A color map to fill with the bipartition.
   * @return true if and only if the given graph is bipartite.
   */
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  BOOST_PARAMETER_FUNCTION(
    (bool), is_bipartite, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_vertex_list_graph>))
    )
    (deduced
      (optional
        (vertex_index_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_to_integer_map_of_graph
            >
          )
          ,get(vertex_index, graph)
        )
        (partition_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_color_map_of_graph
            >
          )
          ,make_one_bit_color_map(num_vertices(graph), vertex_index_map)
        )
      )
    )
  )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  BOOST_PARAMETER_FUNCTION(
    (bool), is_bipartite, ::boost::graph::keywords::tag,
    (required
      (graph, *)
    )
    (optional
      (vertex_index_map, *, get(vertex_index, graph))
      (partition_map
        ,*
        ,make_one_bit_color_map(num_vertices(graph), vertex_index_map)
      )
    )
  )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS
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
    //    detail::empty_recorder recorder;
    //    typedef detail::bipartite_visitor <PartitionMap, detail::empty_recorder> dfs_visitor_t;
    //    dfs_visitor_t dfs_visitor (partition_map, recorder);


    /// Call dfs
    try
    {
      depth_first_search(
        graph,
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
        vertex_index_map,
#endif
        make_dfs_visitor(
          std::make_pair(
            detail::colorize_bipartition(partition_map),
            std::make_pair(
              detail::check_bipartition(partition_map),
              put_property(
                partition_map,
                color_traits<partition_color_t>::white(),
                on_start_vertex()
              )
            )
          )
        )
#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
        ,make_shared_array_property_map(
          num_vertices(graph),
          white_color,
          vertex_index_map
        )
#endif
      );
    }
    catch (const detail::bipartite_visitor_error<vertex_descriptor_t>&)
    {
      return false;
    }

    return true;
  }

  /**
   * Checks a given graph for bipartiteness and fills a given color map with
   * white and black according to the bipartition. If the graph is not
   * bipartite, a sequence of vertices, producing an odd-cycle, is written to
   * the output iterator. The final iterator value is returned. Runs in linear
   * time in the size of the graph, if access to the property maps is in
   * constant time.
   *
   * @param graph The given graph.
   * @param index_map An index map associating vertices with an index.
   * @param partition_map A color map to fill with the bipartition.
   * @param result An iterator to write the odd-cycle vertices to.
   * @return The final iterator value after writing.
   */
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  BOOST_PARAMETER_FUNCTION(
    (
      boost::lazy_enable_if<
        typename mpl::has_key<Args,boost::graph::keywords::tag::result>::type,
        detail::mutable_value_type<Args,boost::graph::keywords::tag::result>
      >
    ), find_odd_cycle, ::boost::graph::keywords::tag,
    (required
      (graph, *(detail::argument_predicate<is_vertex_list_graph>))
    )
    (deduced
      (required
        (result, *(detail::argument_predicate<detail::is_iterator>))
      )
      (optional
        (vertex_index_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_to_integer_map_of_graph
            >
          )
          ,get(vertex_index, graph)
        )
        (partition_map
          ,*(
            detail::argument_with_graph_predicate<
              detail::is_vertex_color_map_of_graph
            >
          )
          ,make_one_bit_color_map(num_vertices(graph), vertex_index_map)
        )
      )
    )
  )
  {
    /// General types and variables
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
    typedef typename property_traits <PartitionMap>::value_type partition_color_t;
    typedef typename graph_traits <Graph>::vertex_descriptor vertex_descriptor_t;
    typedef typename graph_traits <Graph>::vertex_iterator vertex_iterator_t;
    vertex_iterator_t vertex_iter, vertex_end;

    /// Declare predecessor map
    typedef std::vector <vertex_descriptor_t> predecessors_t;
    typedef iterator_property_map <typename predecessors_t::iterator, IndexMap, vertex_descriptor_t,
        vertex_descriptor_t&> predecessor_map_t;

    predecessors_t predecessors (num_vertices (graph), graph_traits <Graph>::null_vertex ());
    predecessor_map_t predecessor_map (predecessors.begin (), vertex_index_map);

    /// Initialize predecessor map
    for (boost::tie (vertex_iter, vertex_end) = vertices (graph); vertex_iter != vertex_end; ++vertex_iter)
    {
      put (predecessor_map, *vertex_iter, *vertex_iter);
    }

    /// Call dfs
    try
    {
      depth_first_search(
        graph,
        vertex_index_map,
        make_dfs_visitor(
          std::make_pair(
            detail::colorize_bipartition(partition_map),
            std::make_pair(
              detail::check_bipartition(partition_map),
              std::make_pair(
                put_property(
                  partition_map,
                  color_traits<partition_color_t>::white(),
                  on_start_vertex()
                ),
                record_predecessors(predecessor_map, on_tree_edge())
              )
            )
          )
        )
      );
    }
    catch (const detail::bipartite_visitor_error <vertex_descriptor_t>& error)
    {
      typedef std::vector <vertex_descriptor_t> path_t;

      path_t path1, path2;
      vertex_descriptor_t next, current;

      /// First path
      next = error.witnesses.first;
      do
      {
        current = next;
        path1.push_back (current);
        next = predecessor_map[current];
      }
      while (current != next);

      /// Second path
      next = error.witnesses.second;
      do
      {
        current = next;
        path2.push_back (current);
        next = predecessor_map[current];
      }
      while (current != next);

      /// Find beginning of common suffix
      std::pair <typename path_t::iterator, typename path_t::iterator> mismatch = detail::reverse_mismatch (
          std::make_pair (path1.begin (), path1.end ()), std::make_pair (path2.begin (), path2.end ()));

      /// Copy the odd-length cycle
      OutputIterator r = result;
      r = std::copy (path1.begin (), mismatch.first + 1, r);
      return std::reverse_copy (path2.begin (), mismatch.second, r);
    }

    return result;
  }
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
  // Fall back on the old public interface. -- Cromwell D. Enage
  template <typename Graph, typename IndexMap, typename PartitionMap, typename OutputIterator>
  OutputIterator find_odd_cycle (const Graph& graph, const IndexMap index_map, PartitionMap partition_map,
      OutputIterator result)
  {
    /// General types and variables
    typedef typename property_traits <PartitionMap>::value_type partition_color_t;
    typedef typename graph_traits <Graph>::vertex_descriptor vertex_descriptor_t;
    typedef typename graph_traits <Graph>::vertex_iterator vertex_iterator_t;
    vertex_iterator_t vertex_iter, vertex_end;

    /// Declare predecessor map
    typedef std::vector <vertex_descriptor_t> predecessors_t;
    typedef iterator_property_map <typename predecessors_t::iterator, IndexMap, vertex_descriptor_t,
        vertex_descriptor_t&> predecessor_map_t;

    predecessors_t predecessors (num_vertices (graph), graph_traits <Graph>::null_vertex ());
    predecessor_map_t predecessor_map (predecessors.begin (), index_map);

    /// Initialize predecessor map
    for (boost::tie (vertex_iter, vertex_end) = vertices (graph); vertex_iter != vertex_end; ++vertex_iter)
    {
      put (predecessor_map, *vertex_iter, *vertex_iter);
    }

    /// Call dfs
    try
    {
      depth_first_search(
        graph,
        make_dfs_visitor(
          std::make_pair(
            detail::colorize_bipartition(partition_map),
            std::make_pair(
              detail::check_bipartition(partition_map),
              std::make_pair(
                put_property(
                  partition_map,
                  color_traits<partition_color_t>::white(),
                  on_start_vertex()
                ),
                record_predecessors(predecessor_map, on_tree_edge())
              )
            )
          )
        ),
        make_shared_array_property_map(
          num_vertices(graph),
          white_color,
          index_map
        )
      );
    }
    catch (const detail::bipartite_visitor_error <vertex_descriptor_t>& error)
    {
      typedef std::vector <vertex_descriptor_t> path_t;

      path_t path1, path2;
      vertex_descriptor_t next, current;

      /// First path
      next = error.witnesses.first;
      do
      {
        current = next;
        path1.push_back (current);
        next = predecessor_map[current];
      }
      while (current != next);

      /// Second path
      next = error.witnesses.second;
      do
      {
        current = next;
        path2.push_back (current);
        next = predecessor_map[current];
      }
      while (current != next);

      /// Find beginning of common suffix
      std::pair <typename path_t::iterator, typename path_t::iterator> mismatch = detail::reverse_mismatch (
          std::make_pair (path1.begin (), path1.end ()), std::make_pair (path2.begin (), path2.end ()));

      /// Copy the odd-length cycle
      result = std::copy (path1.begin (), mismatch.first + 1, result);
      return std::reverse_copy (path2.begin (), mismatch.second, result);
    }

    return result;
  }

  template <typename Graph, typename IndexMap, typename OutputIterator>
  OutputIterator find_odd_cycle (const Graph& graph, const IndexMap index_map, OutputIterator result)
  {
    typedef one_bit_color_map <IndexMap> partition_map_t;
    partition_map_t partition_map (num_vertices (graph), index_map);

    return find_odd_cycle (graph, index_map, partition_map, result);
  }

  template <typename Graph, typename OutputIterator>
  OutputIterator find_odd_cycle (const Graph& graph, OutputIterator result)
  {
    return find_odd_cycle (graph, get (vertex_index, graph), result);
  }
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS
}

#endif /// BOOST_GRAPH_BIPARTITE_HPP
