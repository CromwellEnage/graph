// Copyright 2004 The Trustees of Indiana University.

// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  Authors: Douglas Gregor
//           Andrew Lumsdaine
#ifndef BOOST_GRAPH_BRANDES_BETWEENNESS_CENTRALITY_HPP
#define BOOST_GRAPH_BRANDES_BETWEENNESS_CENTRALITY_HPP

#include <boost/property_map/property_map.hpp>

namespace boost { namespace graph { namespace detail {

    // When the edge centrality map is a dummy property map,
    // no update is performed.
    template <typename Key, typename T>
    inline void update_centrality(dummy_property_map, const Key&, const T&)
    {
    }

    // When we have a real edge centrality map, add the value to the map.
    template <typename CentralityMap, typename Key, typename T>
    inline void update_centrality(CentralityMap c_map, Key k, const T& x)
    {
        put(c_map, k, get(c_map, k) + x);
    }
}}} // end namespace boost::graph::detail

#include <utility>

namespace boost { namespace graph { namespace detail {

    // When the edge centrality map is a dummy property map,
    // no initialization is needed.
    template <typename Iter>
    inline void init_centrality_map(std::pair<Iter, Iter>, dummy_property_map)
    {
    }

    // When we have a real edge centrality map,
    // initialize all of the centralities to zero.
    template <typename Iter, typename Centrality>
    void init_centrality_map(std::pair<Iter, Iter> keys, Centrality c_map)
    {
        typedef typename property_traits<Centrality>::value_type CType;

        for (
            const CType zero_c = CType();
            keys.first != keys.second;
            ++keys.first
        )
        {
            put(c_map, *keys.first, zero_c);
        }
    }

    template <typename I>
    inline void divide_centrality_by_two(std::pair<I, I>, dummy_property_map)
    {
    }

    template <typename Iter, typename CentralityMap>
    inline void divide_centrality_by_two(
        std::pair<Iter, Iter> keys, CentralityMap c_map
    )
    {
        for (
            typename property_traits<CentralityMap>::value_type two(2);
            keys.first != keys.second;
            ++keys.first
        )
        {
            put(c_map, *keys.first, get(c_map, *keys.first) / two);
        }
    }
}}} // end namespace boost::graph::detail

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/relax.hpp>
#include <boost/graph/graph_traits.hpp>
#include <stack>

namespace boost { namespace graph { namespace detail {

  /**
   * Customized visitor passed to Dijkstra's algorithm by Brandes'
   * betweenness centrality algorithm. This visitor is responsible for
   * keeping track of the order in which vertices are discovered, the
   * predecessors on the shortest path(s) to a vertex, and the number
   * of shortest paths.
   */
  template<typename Graph, typename WeightMap, typename IncomingMap,
           typename DistanceMap, typename PathCountMap>
  struct brandes_dijkstra_visitor : public bfs_visitor<>
  {
    typedef typename graph_traits<Graph>::vertex_descriptor vertex_descriptor;
    typedef typename graph_traits<Graph>::edge_descriptor edge_descriptor;

    brandes_dijkstra_visitor(std::stack<vertex_descriptor>& ordered_vertices,
                             WeightMap weight,
                             IncomingMap incoming,
                             DistanceMap distance,
                             PathCountMap path_count)
      : ordered_vertices(ordered_vertices), weight(weight), 
        incoming(incoming), distance(distance),
        path_count(path_count)
    { }

    /**
     * Whenever an edge e = (v, w) is relaxed, the incoming edge list
     * for w is set to {(v, w)} and the shortest path count of w is set to
     * the number of paths that reach {v}.
     */
    void edge_relaxed(edge_descriptor e, const Graph& g) 
    { 
      vertex_descriptor v = source(e, g), w = target(e, g);
      incoming[w].clear();
      incoming[w].push_back(e);
      put(path_count, w, get(path_count, v));
    }

    /**
     * If an edge e = (v, w) was not relaxed, it may still be the case
     * that we've found more equally-short paths, so include {(v, w)} in the
     * incoming edges of w and add all of the shortest paths to v to the
     * shortest path count of w.
     */
    void edge_not_relaxed(edge_descriptor e, const Graph& g) 
    {
      typedef typename property_traits<WeightMap>::value_type weight_type;
      typedef typename property_traits<DistanceMap>::value_type distance_type;
      vertex_descriptor v = source(e, g), w = target(e, g);
      distance_type d_v = get(distance, v), d_w = get(distance, w);
      weight_type w_e = get(weight, e);

      closed_plus<distance_type> combine;
      if (d_w == combine(d_v, w_e)) {
        put(path_count, w, get(path_count, w) + get(path_count, v));
        incoming[w].push_back(e);
      }
    }

    /// Keep track of vertices as they are reached
    void examine_vertex(vertex_descriptor w, const Graph&) 
    { 
      ordered_vertices.push(w);
    }

  private:
    std::stack<vertex_descriptor>& ordered_vertices;
    WeightMap weight;
    IncomingMap incoming;
    DistanceMap distance;
    PathCountMap path_count;
  };
}}} // end namespace boost::graph::detail

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/named_function_params.hpp>

namespace boost { namespace graph { namespace detail {

  /**
   * Function object that calls Dijkstra's shortest paths algorithm
   * using the Dijkstra visitor for the Brandes betweenness centrality
   * algorithm.
   */
  template<typename WeightMap>
  struct brandes_dijkstra_shortest_paths
  {
    brandes_dijkstra_shortest_paths(WeightMap weight_map) 
      : weight_map(weight_map) { }

    template<typename Graph, typename IncomingMap, typename DistanceMap, 
             typename PathCountMap, typename VertexIndexMap>
    void 
    operator()(Graph& g, 
               typename graph_traits<Graph>::vertex_descriptor s,
               std::stack<typename graph_traits<Graph>::vertex_descriptor>& ov,
               IncomingMap incoming,
               DistanceMap distance,
               PathCountMap path_count,
               VertexIndexMap vertex_index)
    {
      typedef brandes_dijkstra_visitor<Graph, WeightMap, IncomingMap, 
                                       DistanceMap, PathCountMap> visitor_type;
      visitor_type visitor(ov, weight_map, incoming, distance, path_count);

      dijkstra_shortest_paths(
        g,
        s,
        boost::graph::keywords::_weight_map = weight_map,
        boost::graph::keywords::_vertex_index_map = vertex_index,
        boost::graph::keywords::_distance_map = distance,
        boost::graph::keywords::_visitor = visitor
      );
    }

  private:
    WeightMap weight_map;
  };
}}} // end namespace boost::graph::detail

#include <boost/config.hpp>
#include <vector>

namespace boost { namespace graph { namespace detail {

  /**
   * Function object that invokes breadth-first search for the
   * unweighted form of the Brandes betweenness centrality algorithm.
   */
  struct brandes_unweighted_shortest_paths
  {
    /**
     * Customized visitor passed to breadth-first search, which
     * records predecessor and the number of shortest paths to each
     * vertex.
     */
    template<typename Graph, typename IncomingMap, typename DistanceMap, 
             typename PathCountMap>
    struct visitor_type : public bfs_visitor<>
    {
      typedef typename graph_traits<Graph>::edge_descriptor edge_descriptor;
      typedef typename graph_traits<Graph>::vertex_descriptor 
        vertex_descriptor;

      visitor_type(IncomingMap incoming, DistanceMap distance, 
                   PathCountMap path_count, 
                   std::stack<vertex_descriptor>& ordered_vertices)
        : incoming(incoming), distance(distance), 
          path_count(path_count), ordered_vertices(ordered_vertices) { }

      visitor_type(const visitor_type& copy)
        : incoming(copy.incoming), distance(copy.distance), 
          path_count(copy.path_count), ordered_vertices(copy.ordered_vertices) { }

      /// Keep track of vertices as they are reached
      void examine_vertex(vertex_descriptor v, Graph&)
      {
        ordered_vertices.push(v);
      }

      /**
       * Whenever an edge e = (v, w) is labelled a tree edge, the
       * incoming edge list for w is set to {(v, w)} and the shortest
       * path count of w is set to the number of paths that reach {v}.
       */
      void tree_edge(edge_descriptor e, Graph& g)
      {
        vertex_descriptor v = source(e, g);
        vertex_descriptor w = target(e, g);
        put(distance, w, get(distance, v) + 1);
        put(path_count, w, get(path_count, v));
#if 0
        typename property_traits<
          IncomingMap
        >::reference in_seq = get(incoming, w);
        in_seq.push_back(e);
#else
        incoming[w].push_back(e);
#endif
      }

      /**
       * If an edge e = (v, w) is not a tree edge, it may still be the
       * case that we've found more equally-short paths, so include (v, w)
       * in the incoming edge list of w and add all of the shortest
       * paths to v to the shortest path count of w.
       */
      void non_tree_edge(edge_descriptor e, Graph& g)
      {
        vertex_descriptor v = source(e, g);
        vertex_descriptor w = target(e, g);
        if (get(distance, w) == get(distance, v) + 1) {
          put(path_count, w, get(path_count, w) + get(path_count, v));
#if 0
          typename property_traits<
            IncomingMap
          >::reference in_seq = get(incoming, w);
          in_seq.push_back(e);
#else
          incoming[w].push_back(e);
#endif
        }
      }

    private:
      IncomingMap incoming;
      DistanceMap distance;
      PathCountMap path_count;
      std::stack<vertex_descriptor>& ordered_vertices;
    };

    template<typename Graph, typename IncomingMap, typename DistanceMap, 
             typename PathCountMap, typename VertexIndexMap>
    void 
    operator()(Graph& g, 
               typename graph_traits<Graph>::vertex_descriptor s,
               std::stack<typename graph_traits<Graph>::vertex_descriptor>& ov,
               IncomingMap incoming,
               DistanceMap distance,
               PathCountMap path_count,
               VertexIndexMap vertex_index)
    {
#if defined(BOOST_MSVC) || defined(__MINGW32__) || ( \
        defined(__APPLE_CC__) && defined(__clang_major__) && \
        (__clang_major__ < 10) \
    )
      typedef static_property_map<
        typename property_traits<DistanceMap>::value_type,
        typename graph_traits<Graph>::edge_descriptor
      > WeightMap;
      WeightMap w_map(1);
      brandes_dijkstra_shortest_paths<WeightMap> bdsp(w_map);
      bdsp(g, s, ov, incoming, distance, path_count, vertex_index);
#else   // neither MSVC nor MinGW nor XCode 9-
      typedef typename graph_traits<Graph>::vertex_descriptor
        vertex_descriptor;

      visitor_type<Graph, IncomingMap, DistanceMap, PathCountMap>
        visitor(incoming, distance, path_count, ov);

      std::vector<default_color_type> 
        colors(num_vertices(g), color_traits<default_color_type>::white());
      breadth_first_visit(
        g,
        s,
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
        boost::graph::keywords::_visitor = visitor,
        boost::graph::keywords::_color_map =
        make_iterator_property_map(colors.begin(), vertex_index)
#else
        boost::visitor(visitor).color_map(
          make_iterator_property_map(colors.begin(), vertex_index)
        )
#endif
      );
#endif  // MSVC or MinGW or XCode 9-
    }
  };
}}}  // end namespace boost::graph::detail

#include <boost/tuple/tuple.hpp>
#include <boost/type_traits/is_convertible.hpp>

namespace boost { namespace graph { namespace detail {

  template<typename Graph, typename CentralityMap, typename EdgeCentralityMap,
           typename IncomingMap, typename DistanceMap, 
           typename DependencyMap, typename PathCountMap,
           typename VertexIndexMap, typename ShortestPaths>
  void 
  brandes_betweenness_centrality_impl(const Graph& g, 
                                      CentralityMap centrality,     // C_B
                                      EdgeCentralityMap edge_centrality_map,
                                      IncomingMap incoming,         // P
                                      DistanceMap distance,         // d
                                      DependencyMap dependency,     // delta
                                      PathCountMap path_count,      // sigma
                                      VertexIndexMap vertex_index,
                                      ShortestPaths shortest_paths)
  {
    typedef typename graph_traits<Graph>::vertex_iterator vertex_iterator;
    typedef typename graph_traits<Graph>::vertex_descriptor vertex_descriptor;

    // Initialize centrality
    init_centrality_map(vertices(g), centrality);
    init_centrality_map(edges(g), edge_centrality_map);

    std::stack<vertex_descriptor> ordered_vertices;
    vertex_iterator s, s_end;
    for (boost::tie(s, s_end) = vertices(g); s != s_end; ++s) {
      // Initialize for this iteration
      vertex_iterator w, w_end;
      for (boost::tie(w, w_end) = vertices(g); w != w_end; ++w) {
#if 0
        typename property_traits<
          IncomingMap
        >::reference in_seq = get(incoming, *w);
        in_seq.clear();
#else
        incoming[*w].clear();
#endif
        put(path_count, *w, 0);
        put(dependency, *w, 0);
      }
      put(path_count, *s, 1);

      // Execute the shortest paths algorithm. This will be either
      // Dijkstra's algorithm or a customized breadth-first search,
      // depending on whether the graph is weighted or unweighted.
      shortest_paths(g, *s, ordered_vertices, incoming, distance,
                     path_count, vertex_index);

      while (!ordered_vertices.empty()) {
        vertex_descriptor w = ordered_vertices.top();
        ordered_vertices.pop();

        typedef typename property_traits<IncomingMap>::value_type
          incoming_type;
        typedef typename incoming_type::iterator incoming_iterator;
        typedef typename property_traits<DependencyMap>::value_type 
          dependency_type;

#if 0
        typename property_traits<
          IncomingMap
        >::reference in_seq = get(incoming, w);

        for (incoming_iterator vw = in_seq.begin();
             vw != in_seq.end(); ++vw)
#else
        for (incoming_iterator vw = incoming[w].begin();
             vw != incoming[w].end(); ++vw)
#endif
        {
          vertex_descriptor v = source(*vw, g);
          dependency_type factor = dependency_type(get(path_count, v))
            / dependency_type(get(path_count, w));
          factor *= (dependency_type(1) + get(dependency, w));
          put(dependency, v, get(dependency, v) + factor);
          update_centrality(edge_centrality_map, *vw, factor);
        }
        
        if (w != *s) {
          update_centrality(centrality, w, get(dependency, w));
        }
      }
    }

    typedef typename graph_traits<Graph>::directed_category directed_category;
    const bool is_undirected = 
      boost::is_convertible<directed_category*, undirected_tag*>::value;
    if (is_undirected) {
      divide_centrality_by_two(vertices(g), centrality);
      divide_centrality_by_two(edges(g), edge_centrality_map);
    }
  }
}}}  // end namespace boost::graph::detail

#include <boost/graph/overloading.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

template<typename Graph, typename CentralityMap, typename EdgeCentralityMap,
         typename IncomingMap, typename DistanceMap, 
         typename DependencyMap, typename PathCountMap, 
         typename VertexIndexMap>
void 
brandes_betweenness_centrality(const Graph& g, 
                               CentralityMap centrality,     // C_B
                               EdgeCentralityMap edge_centrality_map,
                               IncomingMap incoming,         // P
                               DistanceMap distance,         // d
                               DependencyMap dependency,     // delta
                               PathCountMap path_count,      // sigma
                               VertexIndexMap vertex_index,
                               typename boost::disable_if<
                                 parameter::are_tagged_arguments<
                                   CentralityMap, EdgeCentralityMap,
                                   IncomingMap, DistanceMap,
                                   DependencyMap, PathCountMap,
                                   VertexIndexMap
                                 >,
                                 mpl::true_
                               >::type = mpl::true_()
                               BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag))
{
  graph::detail::brandes_unweighted_shortest_paths shortest_paths;

  graph::detail::brandes_betweenness_centrality_impl(g,
                                                     centrality, 
                                                     edge_centrality_map,
                                                     incoming, distance,
                                                     dependency, path_count,
                                                     vertex_index, 
                                                     shortest_paths);
}

template<typename Graph, typename CentralityMap, typename EdgeCentralityMap, 
         typename IncomingMap, typename DistanceMap, 
         typename DependencyMap, typename PathCountMap, 
         typename VertexIndexMap, typename WeightMap>    
void 
brandes_betweenness_centrality(const Graph& g, 
                               CentralityMap centrality,     // C_B
                               EdgeCentralityMap edge_centrality_map,
                               IncomingMap incoming,         // P
                               DistanceMap distance,         // d
                               DependencyMap dependency,     // delta
                               PathCountMap path_count,      // sigma
                               VertexIndexMap vertex_index,
                               WeightMap weight_map,
                               typename boost::disable_if<
                                 parameter::are_tagged_arguments<
                                   CentralityMap, EdgeCentralityMap,
                                   IncomingMap, DistanceMap,
                                   DependencyMap, PathCountMap,
                                   VertexIndexMap, WeightMap
                                 >,
                                 mpl::true_
                               >::type = mpl::true_()
                               BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag))
{
  graph::detail::brandes_dijkstra_shortest_paths<WeightMap>
    shortest_paths(weight_map);

  graph::detail::brandes_betweenness_centrality_impl(g,
                                                     centrality, 
                                                     edge_centrality_map,
                                                     incoming, distance,
                                                     dependency, path_count,
                                                     vertex_index, 
                                                     shortest_paths);
}
}}  // end namespace boost::graph

#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/is_base_of.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <algorithm>

namespace boost { namespace graph { namespace detail {

template <typename Graph, typename Args>
void brandes_betweenness_centrality_dispatch(
    const Graph& g, const Args& arg_pack, mpl::true_
)
{
    typedef typename boost::detail::override_const_property_result<
        Args,
        boost::graph::keywords::tag::weight_map,
        edge_weight_t,
        Graph
    >::type WeightMap;
    WeightMap e_w_map = boost::detail::override_const_property(
        arg_pack,
        boost::graph::keywords::_weight_map,
        g,
        edge_weight
    );
    typedef typename graph_traits<Graph>::edge_descriptor Edge;
    typedef typename graph_traits<Graph>::degree_size_type D;
    typedef typename property_traits<
        typename mpl::eval_if<
            typename mpl::has_key<
                Args,
                boost::graph::keywords::tag::edge_centrality_map
            >::type,
            boost::detail::override_const_property_result<
                Args,
                boost::graph::keywords::tag::edge_centrality_map,
                edge_centrality_t,
                Graph
            >,
            mpl::eval_if<
                typename mpl::has_key<
                    Args,
                    boost::graph::keywords::tag::centrality_map
                >::type,
                boost::detail::override_const_property_result<
                    Args,
                    boost::graph::keywords::tag::centrality_map,
                    vertex_centrality_t,
                    Graph
                >,
                boost::detail::override_const_property_result<
                    Args,
                    boost::graph::keywords::tag::weight_map,
                    edge_weight_t,
                    Graph
                >
            >
        >::type
    >::value_type centrality_type;
    typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::centrality_map,
            dummy_property_map
        >::type
    >::type v_c_map = arg_pack[
        boost::graph::keywords::_centrality_map ||
        boost::value_factory<dummy_property_map>()
    ];
    typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::edge_centrality_map,
            dummy_property_map
        >::type
    >::type e_c_map = arg_pack[
        boost::graph::keywords::_edge_centrality_map ||
        boost::value_factory<dummy_property_map>()
    ];
    std::vector<Edge> no_edges;
    const D zero_degree = D();
    const centrality_type zero_centrality = centrality_type();
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::incoming_map,
        std::vector<Edge>
    > incoming_map_gen(no_edges);
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::distance_map,
        centrality_type
    > dist_map_gen(zero_centrality);
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::dependency_map,
        centrality_type
    > dependency_map_gen(zero_centrality);
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::path_count_map,
        D
    > path_count_map_gen(zero_degree);
    graph::detail::brandes_dijkstra_shortest_paths<
        WeightMap
    > bdsp(e_w_map);
    graph::detail::brandes_betweenness_centrality_impl(
        g,
        v_c_map,
        e_c_map,
        incoming_map_gen(g, arg_pack),
        dist_map_gen(g, arg_pack),
        dependency_map_gen(g, arg_pack),
        path_count_map_gen(g, arg_pack),
        boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
        ),
        bdsp
    );
}

template <typename Graph, typename Args>
void brandes_betweenness_centrality_dispatch(
    const Graph& g, const Args& arg_pack, mpl::false_
)
{
    typedef typename graph_traits<Graph>::edge_descriptor Edge;
    typedef typename graph_traits<Graph>::degree_size_type D;
    typedef typename property_traits<
        typename mpl::eval_if<
            typename mpl::has_key<
                Args,
                boost::graph::keywords::tag::edge_centrality_map
            >::type,
            boost::detail::override_const_property_result<
                Args,
                boost::graph::keywords::tag::edge_centrality_map,
                edge_centrality_t,
                Graph
            >,
            boost::detail::override_const_property_result<
                Args,
                boost::graph::keywords::tag::centrality_map,
                vertex_centrality_t,
                Graph
            >
        >::type
    >::value_type centrality_type;
    typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::centrality_map,
            dummy_property_map
        >::type
    >::type v_c_map = arg_pack[
        boost::graph::keywords::_centrality_map ||
        boost::value_factory<dummy_property_map>()
    ];
    typename boost::remove_const<
        typename parameter::value_type<
            Args,
            boost::graph::keywords::tag::edge_centrality_map,
            dummy_property_map
        >::type
    >::type e_c_map = arg_pack[
        boost::graph::keywords::_edge_centrality_map ||
        boost::value_factory<dummy_property_map>()
    ];
    std::vector<Edge> no_edges;
    const D zero_degree = D();
    const centrality_type zero_centrality = centrality_type();
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::incoming_map,
        std::vector<Edge>
    > incoming_map_gen(no_edges);
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::distance_map,
        centrality_type
    > dist_map_gen(zero_centrality);
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::dependency_map,
        centrality_type
    > dependency_map_gen(zero_centrality);
    boost::detail::make_property_map_from_arg_pack_gen<
        boost::graph::keywords::tag::path_count_map,
        D
    > path_count_map_gen(zero_degree);
    graph::detail::brandes_unweighted_shortest_paths usp;
    graph::detail::brandes_betweenness_centrality_impl(
        g,
        v_c_map,
        e_c_map,
        incoming_map_gen(g, arg_pack),
        dist_map_gen(g, arg_pack),
        dependency_map_gen(g, arg_pack),
        path_count_map_gen(g, arg_pack),
        boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
        ),
        usp
    );
}
}}}  // end namespace boost::graph::detail

namespace boost { namespace graph {

template <typename Graph, typename Args>
inline void brandes_betweenness_centrality(
    const Graph& g,
    const Args& arg_pack,
    typename boost::enable_if<
        parameter::is_argument_pack<Args>,
        mpl::true_
    >::type = mpl::true_()
    BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag)
)
{
    graph::detail::brandes_betweenness_centrality_dispatch(
        g,
        arg_pack,
        typename mpl::has_key<
            Args,
            boost::graph::keywords::tag::weight_map
        >::type()
    );
}
}}  // end namespace boost::graph

#include <boost/parameter/compose.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
template <typename Graph, typename TA \
          BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA)> \
inline void name( \
    const Graph& g, const TA& ta \
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
        g, \
        parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
    ); \
}

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 9, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, brandes_betweenness_centrality
)
}}  // end namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost { namespace graph {

// disable_if is required to work around problem with MSVC 7.1
// (which seems to not get partial ordering getween this overload
// and the previous one correct)
template <typename Graph, typename CentralityMap>
inline typename boost::disable_if<
    typename mpl::if_<
        boost::is_base_of<
            boost::detail::bgl_named_params_base,
            CentralityMap
        >,
        mpl::true_,
        parameter::is_argument_pack<CentralityMap>
    >::type,
    void
>::type
brandes_betweenness_centrality(
    const Graph& g,
    CentralityMap centrality,
    typename boost::disable_if<
        parameter::are_tagged_arguments<CentralityMap>,
        mpl::true_
    >::type = mpl::true_()
    BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag)
)
{
    brandes_betweenness_centrality(
        g,
        boost::graph::keywords::_centrality_map = centrality
    );
}

template <typename Graph, typename CentralityMap, typename EdgeCentralityMap>
inline void brandes_betweenness_centrality(
    const Graph& g,
    CentralityMap centrality,
    EdgeCentralityMap edge_centrality_map,
    typename boost::disable_if<
        parameter::are_tagged_arguments<CentralityMap,EdgeCentralityMap>,
        mpl::true_
    >::type = mpl::true_()
    BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag)
)
{
    brandes_betweenness_centrality(
        g,
        boost::graph::keywords::_centrality_map = centrality,
        boost::graph::keywords::_edge_centrality_map = edge_centrality_map
    );
}
}} // end namespace boost::graph

namespace boost {

using ::boost::graph::brandes_betweenness_centrality;

template <typename Graph, typename Param, typename Tag, typename Rest>
inline void brandes_betweenness_centrality(
    const Graph& g,
    const bgl_named_params<Param,Tag,Rest>& params
    BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag)
)
{
    typedef bgl_named_params<Param,Tag,Rest> named_params;
    using namespace boost::graph::keywords;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(named_params, params)
    graph::detail::brandes_betweenness_centrality_dispatch(
        g,
        arg_pack,
        typename mpl::has_key<
            arg_pack_type,
            boost::graph::keywords::tag::weight_map
        >::type()
    );
}

/**
 * Converts "absolute" betweenness centrality (as computed by the
 * brandes_betweenness_centrality algorithm) in the centrality map
 * into "relative" centrality. The result is placed back into the
 * given centrality map.
 */
template<typename Graph, typename CentralityMap>
void 
relative_betweenness_centrality(const Graph& g, CentralityMap centrality)
{
  typedef typename graph_traits<Graph>::vertex_iterator vertex_iterator;
  typedef typename property_traits<CentralityMap>::value_type centrality_type;

  typename graph_traits<Graph>::vertices_size_type n = num_vertices(g);
  centrality_type factor = centrality_type(2)/centrality_type(n*n - 3*n + 2);
  vertex_iterator v, v_end;
  for (boost::tie(v, v_end) = vertices(g); v != v_end; ++v) {
    put(centrality, *v, factor * get(centrality, *v));
  }
}

// Compute the central point dominance of a graph.
template<typename Graph, typename CentralityMap>
typename property_traits<CentralityMap>::value_type
central_point_dominance(const Graph& g, CentralityMap centrality
                        BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph,vertex_list_graph_tag))
{
  using std::max;

  typedef typename graph_traits<Graph>::vertex_iterator vertex_iterator;
  typedef typename property_traits<CentralityMap>::value_type centrality_type;

  typename graph_traits<Graph>::vertices_size_type n = num_vertices(g);

  // Find max centrality
  centrality_type max_centrality(0);
  vertex_iterator v, v_end;
  for (boost::tie(v, v_end) = vertices(g); v != v_end; ++v) {
    max_centrality = (max)(max_centrality, get(centrality, *v));
  }

  // Compute central point dominance
  centrality_type sum(0);
  for (boost::tie(v, v_end) = vertices(g); v != v_end; ++v) {
    sum += (max_centrality - get(centrality, *v));
  }
  return sum/(n-1);
}

} // end namespace boost

#endif // BOOST_GRAPH_BRANDES_BETWEENNESS_CENTRALITY_HPP
