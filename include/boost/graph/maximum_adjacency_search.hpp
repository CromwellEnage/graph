//
//=======================================================================
// Copyright 2012 Fernando Vilas
//           2010 Daniel Trebbien
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//

// The maximum adjacency search algorithm was originally part of the
// Stoer-Wagner min cut implementation by Daniel Trebbien. It has been
// broken out into its own file to be a public search algorithm, with
// visitor concepts.
#ifndef BOOST_GRAPH_MAXIMUM_ADJACENCY_SEARCH_H
#define BOOST_GRAPH_MAXIMUM_ADJACENCY_SEARCH_H

/**
 * This is an implementation of the maximum adjacency search on an
 * undirected graph. It allows a visitor object to perform some
 * operation on each vertex as that vertex is visited.
 *
 * The algorithm runs as follows:
 *
 * Initialize all nodes to be unvisited (reach count = 0)
 *   and call vis.initialize_vertex
 * For i = number of nodes in graph downto 1
 *   Select the unvisited node with the highest reach count
 *     The user provides the starting node to break the first tie,
 *     but future ties are broken arbitrarily
 *   Visit the node by calling vis.start_vertex
 *   Increment the reach count for all unvisited neighbors
 *     and call vis.examine_edge for each of these edges
 *   Mark the node as visited and call vis.finish_vertex
 *
 */

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/buffer_concepts.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <set>

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/compose.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>
#endif

namespace boost {
  template <class Visitor, class Graph>
  struct MASVisitorConcept {
    void constraints() {
      boost::function_requires< boost::CopyConstructibleConcept<Visitor> >();
      vis.initialize_vertex(u, g);
      vis.start_vertex(u, g);
      vis.examine_edge(e, g);
      vis.finish_vertex(u, g);
    }
    Visitor vis;
    Graph g;
    typename boost::graph_traits<Graph>::vertex_descriptor u;
    typename boost::graph_traits<Graph>::edge_descriptor e;
  };

  template <class Visitors = null_visitor>
  class mas_visitor {
  public:
    mas_visitor() { }
    mas_visitor(Visitors vis) : m_vis(vis) { }

    template <class Vertex, class Graph>
    void
    initialize_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_initialize_vertex());
    }

    template <class Vertex, class Graph>
    void
    start_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_start_vertex());
    }

    template <class Edge, class Graph>
    void
    examine_edge(Edge e, Graph& g)
    {
      invoke_visitors(m_vis, e, g, ::boost::on_examine_edge());
    }

    template <class Vertex, class Graph>
    void
    finish_vertex(Vertex u, Graph& g)
    {
      invoke_visitors(m_vis, u, g, ::boost::on_finish_vertex());
    }

    BOOST_GRAPH_EVENT_STUB(on_initialize_vertex,mas)
    BOOST_GRAPH_EVENT_STUB(on_start_vertex,mas)
    BOOST_GRAPH_EVENT_STUB(on_examine_edge,mas)
    BOOST_GRAPH_EVENT_STUB(on_finish_vertex,mas)

  protected:
    Visitors m_vis;
  };
  template <class Visitors>
  mas_visitor<Visitors>
  make_mas_visitor(Visitors vis) {
    return mas_visitor<Visitors>(vis);
  }
  typedef mas_visitor<> default_mas_visitor;

  namespace detail {
    template <
      typename Graph, typename WeightMap, typename MASVisitor,
      typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue
    >
    void
    maximum_adjacency_search(
      const Graph& g, WeightMap weights,
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
      MASVisitor&& vis,
#else
      MASVisitor vis,
#endif
      const typename boost::graph_traits<Graph>::vertex_descriptor start,
      VertexAssignmentMap assignments,
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
      KeyedUpdatablePriorityQueue&& pq
#else
      KeyedUpdatablePriorityQueue pq
#endif
    )
    {
      typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
      typedef typename boost::property_traits<WeightMap>::value_type weight_type;

      std::set<vertex_descriptor> assignedVertices;

      // initialize `assignments` (all vertices are initially
      // assigned to themselves)
      BGL_FORALL_VERTICES_T(v, g, Graph) {
        put(assignments, v, v);
      }

      typename boost::remove_const<
        typename boost::remove_reference<KeyedUpdatablePriorityQueue>::type
      >::type::key_map keys = pq.keys();

      // set number of visited neighbors for all vertices to 0
      BGL_FORALL_VERTICES_T(v, g, Graph) {
        if (v == get(assignments, v)) { // foreach u \in V do
          put(keys, v, weight_type(0));          vis.initialize_vertex(v, g);

          pq.push(v);
        }
      }
      BOOST_ASSERT(pq.size() >= 2);

      // Give the starting vertex high priority
      put(keys, start, get(keys, start) + num_vertices(g) + 1);
      pq.update(start);

      // start traversing the graph
      //vertex_descriptor s, t;
      //weight_type w;
      while (!pq.empty()) { // while PQ \neq {} do
        const vertex_descriptor u = pq.top(); // u = extractmax(PQ)
        /* weight_type w = */ get(keys, u);      vis.start_vertex(u, g);
        pq.pop();                  //            vis.start_vertex(u, g);

        BGL_FORALL_OUTEDGES_T(u, e, g, Graph) { // foreach (u, v) \in E do
                                                 vis.examine_edge(e, g);

          const vertex_descriptor v = get(assignments, target(e, g));

          if (pq.contains(v)) { // if v \in PQ then
            put(keys, v, get(keys, v) + get(weights, e)); // increasekey(PQ, v, wA(v) + w(u, v))
            pq.update(v);
          }
        }

        typename std::set<vertex_descriptor>::const_iterator assignedVertexIt, assignedVertexEnd = assignedVertices.end();
        for (assignedVertexIt = assignedVertices.begin(); assignedVertexIt != assignedVertexEnd; ++assignedVertexIt) {
          const vertex_descriptor uPrime = *assignedVertexIt;

          if (get(assignments, uPrime) == u) {
            BGL_FORALL_OUTEDGES_T(uPrime, e, g, Graph) { // foreach (u, v) \in E do
                                                 vis.examine_edge(e, g);

              const vertex_descriptor v = get(assignments, target(e, g));

              if (pq.contains(v)) { // if v \in PQ then
                put(keys, v, get(keys, v) + get(weights, e)); // increasekey(PQ, v, wA(v) + w(u, v))
                pq.update(v);
              }
            }
          }
        }
                                                 vis.finish_vertex(u, g);
      }
    }
  } // end namespace detail

  template <
    typename Graph, typename WeightMap, typename MASVisitor,
    typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue
  >
  void
  maximum_adjacency_search(
    const Graph& g, WeightMap weights,
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    MASVisitor&& vis,
#else
    MASVisitor vis,
#endif
    const typename boost::graph_traits<Graph>::vertex_descriptor start,
    VertexAssignmentMap assignments,
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    KeyedUpdatablePriorityQueue&& pq
#else
    KeyedUpdatablePriorityQueue pq
#endif
  ) {
    BOOST_CONCEPT_ASSERT((boost::IncidenceGraphConcept<Graph>));
    BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<Graph>));
    typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
    typedef typename boost::graph_traits<Graph>::vertices_size_type vertices_size_type;
    typedef typename boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
    BOOST_CONCEPT_ASSERT((boost::Convertible<typename boost::graph_traits<Graph>::directed_category, boost::undirected_tag>));
    BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<WeightMap, edge_descriptor>));
    // typedef typename boost::property_traits<WeightMap>::value_type weight_type;
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    BOOST_CONCEPT_ASSERT((
      MASVisitorConcept<
        typename boost::remove_const<
          typename boost::remove_reference<MASVisitor>::type
        >::type,
        Graph
      >
    ));
#else
    boost::function_requires< MASVisitorConcept<MASVisitor, Graph> >();
#endif
    BOOST_CONCEPT_ASSERT((boost::ReadWritePropertyMapConcept<VertexAssignmentMap, vertex_descriptor>));
    BOOST_CONCEPT_ASSERT((boost::Convertible<vertex_descriptor, typename boost::property_traits<VertexAssignmentMap>::value_type>));
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    BOOST_CONCEPT_ASSERT((
      boost::KeyedUpdatableQueueConcept<
        typename boost::remove_const<
          typename boost::remove_reference<KeyedUpdatablePriorityQueue>::type
        >::type
      >
    ));
#else
    BOOST_CONCEPT_ASSERT((boost::KeyedUpdatableQueueConcept<KeyedUpdatablePriorityQueue>));
#endif

    vertices_size_type n = num_vertices(g);
    if (n < 2)
      throw boost::bad_graph("the input graph must have at least two vertices.");
    else if (!pq.empty())
      throw std::invalid_argument("the max-priority queue must be empty initially.");

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS) && \
    defined(BOOST_PARAMETER_HAS_PERFECT_FORWARDING)
    detail::maximum_adjacency_search(
      g, weights, std::forward<MASVisitor>(vis), start, assignments,
      std::forward<KeyedUpdatablePriorityQueue>(pq)
    );
#else
    detail::maximum_adjacency_search(g, weights, vis, start, assignments, pq);
#endif
  }

  namespace graph {
    namespace detail {
      template <typename WeightMap>
      struct mas_dispatch {
        typedef void result_type;
        template <typename Graph, typename ArgPack>
        static result_type apply(const Graph& g,
                          //const bgl_named_params<P,T,R>& params,
                          const ArgPack& params,
                          WeightMap w) {

          using namespace boost::graph::keywords;
          typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
          typedef typename WeightMap::value_type weight_type;

          typedef boost::detail::make_priority_queue_from_arg_pack_gen<boost::graph::keywords::tag::max_priority_queue, weight_type, vertex_descriptor, std::greater<weight_type> > default_pq_gen_type;

          default_pq_gen_type pq_gen(choose_param(get_param(params, boost::distance_zero_t()), weight_type(0)));

          typename boost::result_of<default_pq_gen_type(const Graph&, const ArgPack&)>::type pq = pq_gen(g, params);

          boost::null_visitor null_vis;
          boost::mas_visitor<boost::null_visitor> default_visitor(null_vis);
          vertex_descriptor v = vertex_descriptor();
          boost::detail::make_property_map_from_arg_pack_gen<
              boost::graph::keywords::tag::vertex_assignment_map,
              vertex_descriptor
          > map_gen(v);
          typename boost::detail::map_maker<
              Graph,
              ArgPack,
              boost::graph::keywords::tag::vertex_assignment_map,
              vertex_descriptor
          >::map_type default_map = map_gen(g, params);
          boost::maximum_adjacency_search
               (g,
                w,
                params [ _visitor | default_visitor],
                params [ _root_vertex | *vertices(g).first],
                params [ _vertex_assignment_map | default_map],
                pq
                );
        }
      };

      template <>
      struct mas_dispatch<boost::param_not_found> {
        typedef void result_type;

        template <typename Graph, typename ArgPack>
        static result_type apply(const Graph& g,
                          const ArgPack& params,
                          param_not_found) {

          using namespace boost::graph::keywords;
          typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;

          // get edge_weight_t as the weight type
          typedef typename boost::property_map<Graph, edge_weight_t> WeightMap;
          typedef typename WeightMap::value_type weight_type;

          typedef boost::detail::make_priority_queue_from_arg_pack_gen<boost::graph::keywords::tag::max_priority_queue, weight_type, vertex_descriptor, std::greater<weight_type> > default_pq_gen_type;

          default_pq_gen_type pq_gen(choose_param(get_param(params, boost::distance_zero_t()), weight_type(0)));

          typename boost::result_of<default_pq_gen_type(const Graph&, const ArgPack&)>::type pq = pq_gen(g, params);

          boost::null_visitor null_vis;
          boost::mas_visitor<boost::null_visitor> default_visitor(null_vis);
          vertex_descriptor v = vertex_descriptor();
          boost::detail::make_property_map_from_arg_pack_gen<
              boost::graph::keywords::tag::vertex_assignment_map,
              vertex_descriptor
          > map_gen(v);
          typename boost::detail::map_maker<
              Graph,
              ArgPack,
              boost::graph::keywords::tag::vertex_assignment_map,
              vertex_descriptor
          >::map_type default_map = map_gen(g, params);
          boost::maximum_adjacency_search
               (g,
                get(edge_weight, g),
                params [ _visitor | default_visitor],
                params [ _root_vertex | *vertices(g).first],
                params [ _vertex_assignment_map | default_map],
                pq
                );
        }
      };
    } // end namespace detail
  } // end namespace graph

  // Named parameter interface
  //BOOST_GRAPH_MAKE_OLD_STYLE_PARAMETER_FUNCTION(maximum_adjacency_search, 1)
  template <typename Graph, typename P, typename T, typename R>
  void
  maximum_adjacency_search (const Graph& g,
      const bgl_named_params<P,T,R>& params) {

    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)

    // do the dispatch based on WeightMap
    typedef typename get_param_type<edge_weight_t, bgl_named_params<P,T,R> >::type W;
    graph::detail::mas_dispatch<W>::apply(g, arg_pack, get_param(params, edge_weight));
  }

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
  template <typename Graph, typename Args>
  inline void maximum_adjacency_search(
    const Graph& g, const Args& arg_pack, typename boost::enable_if<
      parameter::is_argument_pack<Args>, mpl::true_
    >::type = mpl::true_()
  )
  {
    using namespace boost::graph::keywords;
    typename boost::detail::override_const_property_result<
        Args,
        boost::graph::keywords::tag::vertex_index_map,
        vertex_index_t,
        Graph
    >::type v_i_map = detail::override_const_property(
        arg_pack,
        _vertex_index_map,
        g,
        vertex_index
    );
    typedef typename boost::detail::override_const_property_result<
        Args,
        boost::graph::keywords::tag::weight_map,
        edge_weight_t,
        Graph
    >::type weight_map_type;
    weight_map_type w_map = detail::override_const_property(
        arg_pack,
        _weight_map,
        g,
        edge_weight
    );
    typedef typename boost::property_traits<weight_map_type>::value_type D;
    const D zero_actual = D();
    boost::maximum_adjacency_search(
      g,
      w_map,
      arg_pack[_visitor | default_mas_visitor()],
      arg_pack[
        _root_vertex || detail::get_default_starting_vertex_t<Graph>(g)
      ],
      arg_pack[
        _vertex_assignment_map |
        make_shared_array_property_map(
          num_vertices(g),
          detail::get_null_vertex(g),
          v_i_map
        )
      ],
      arg_pack[
        _max_priority_queue |
        detail::create_empty_d_ary_heap_indirect<4>(
          detail::get_null_vertex(g),
          arg_pack[
            _index_in_heap_map |
            make_shared_array_property_map(
              num_vertices(g),
              num_vertices(g) - num_vertices(g),
              v_i_map
            )
          ],
          arg_pack[
            _distance_map |
            make_shared_array_property_map(
              num_vertices(g),
              zero_actual,
              v_i_map
            )
          ],
          std::greater<typename graph_traits<Graph>::vertex_descriptor>()
        )
      ]
    );
  }

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
  template <typename Graph, typename TA \
            BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA)> \
  inline void name( \
    const Graph &g, const TA& ta \
    BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
    typename boost::enable_if< \
      parameter::are_tagged_arguments< \
        TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
      >, mpl::true_ \
    >::type = mpl::true_() \
  ) \
  { \
    name(g, parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta))); \
  }

BOOST_PP_REPEAT_FROM_TO(1, 9, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, maximum_adjacency_search)

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
  namespace graph {
    namespace detail {
      template <typename Graph>
      struct maximum_adjacency_search_impl {
        typedef void result_type;

        template <typename ArgPack>
        void
        operator() (const Graph& g, const ArgPack& arg_pack) const {
          // call the function that does the dispatching
          typedef typename get_param_type<edge_weight_t, ArgPack >::type W;
          graph::detail::mas_dispatch<W>::apply(g, arg_pack, get_param(arg_pack, edge_weight));
        }
      };
    } // end namespace detail
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(maximum_adjacency_search,1,5)
  } // end namespace graph
#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

} // end namespace boost

#include <boost/graph/iteration_macros_undef.hpp>

#endif // BOOST_GRAPH_MAXIMUM_ADJACENCY_SEARCH_H
