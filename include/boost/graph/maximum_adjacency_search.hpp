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
#include <boost/graph/graph_traits.hpp>

namespace boost {

    template <typename Visitor, typename Graph>
    struct MASVisitorConcept
    {
        void constraints()
        {
            boost::function_requires<
                boost::CopyConstructibleConcept<Visitor>
            >();
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
}

#include <boost/graph/visitors.hpp>

namespace boost { namespace graph {

    template <typename Visitors = null_visitor>
    class mas_visitor
    {
    public:
        mas_visitor()
        {
        }

        mas_visitor(Visitors vis) : m_vis(vis)
        {
        }

        template <typename Vertex, typename Graph>
        inline void initialize_vertex(Vertex u, Graph& g)
        {
            invoke_visitors(
                this->m_vis, u, g, ::boost::on_initialize_vertex()
            );
        }

        template <typename Vertex, typename Graph>
        inline void start_vertex(Vertex u, Graph& g)
        {
            invoke_visitors(this->m_vis, u, g, ::boost::on_start_vertex());
        }

        template <typename Edge, typename Graph>
        inline void examine_edge(Edge e, Graph& g)
        {
            invoke_visitors(this->m_vis, e, g, ::boost::on_examine_edge());
        }

        template <typename Vertex, typename Graph>
        inline void finish_vertex(Vertex u, Graph& g)
        {
            invoke_visitors(this->m_vis, u, g, ::boost::on_finish_vertex());
        }

        BOOST_GRAPH_EVENT_STUB(on_initialize_vertex,mas)
        BOOST_GRAPH_EVENT_STUB(on_start_vertex,mas)
        BOOST_GRAPH_EVENT_STUB(on_examine_edge,mas)
        BOOST_GRAPH_EVENT_STUB(on_finish_vertex,mas)

    protected:
        Visitors m_vis;
    };

    template <class Visitors>
    inline mas_visitor<Visitors> make_mas_visitor(Visitors vis)
    {
        return mas_visitor<Visitors>(vis);
    }

    typedef mas_visitor<> default_mas_visitor;
}}

#include <boost/graph/buffer_concepts.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <set>

#include <boost/graph/detail/d_ary_heap.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/compose.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/functional/value_factory.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>
#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace detail {

    template <
      typename Graph, typename WeightMap, typename MASVisitor,
      typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue
    >
    void
    maximum_adjacency_search_impl(
      const Graph& g, WeightMap weights,
#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
      MASVisitor& vis,
#else
      MASVisitor vis,
#endif
      const typename boost::graph_traits<Graph>::vertex_descriptor start,
      VertexAssignmentMap assignments,
#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
      KeyedUpdatablePriorityQueue& pq
#else
      KeyedUpdatablePriorityQueue pq
#endif
    )
    {
      BOOST_CONCEPT_ASSERT((boost::IncidenceGraphConcept<Graph>));
      BOOST_CONCEPT_ASSERT((boost::VertexListGraphConcept<Graph>));
      typedef typename boost::graph_traits<Graph>::vertex_descriptor vertex_descriptor;
      typedef typename boost::graph_traits<Graph>::vertices_size_type vertices_size_type;
      typedef typename boost::graph_traits<Graph>::edge_descriptor edge_descriptor;
      BOOST_CONCEPT_ASSERT((boost::Convertible<typename boost::graph_traits<Graph>::directed_category, boost::undirected_tag>));
      BOOST_CONCEPT_ASSERT((boost::ReadablePropertyMapConcept<WeightMap, edge_descriptor>));
      // typedef typename boost::property_traits<WeightMap>::value_type weight_type;
      boost::function_requires< MASVisitorConcept<MASVisitor, Graph> >();
      BOOST_CONCEPT_ASSERT((boost::ReadWritePropertyMapConcept<VertexAssignmentMap, vertex_descriptor>));
      BOOST_CONCEPT_ASSERT((boost::Convertible<vertex_descriptor, typename boost::property_traits<VertexAssignmentMap>::value_type>));
      BOOST_CONCEPT_ASSERT((boost::KeyedUpdatableQueueConcept<KeyedUpdatablePriorityQueue>));

      vertices_size_type n = num_vertices(g);
      if (n < 2)
        throw boost::bad_graph("the input graph must have at least two vertices.");
      else if (!pq.empty())
        throw std::invalid_argument("the max-priority queue must be empty initially.");

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

#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
    template <
      typename Graph, typename WeightMap, typename MASVisitor,
      typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue
    >
    void
    maximum_adjacency_search_impl(
      const Graph& g, WeightMap weights,
      const MASVisitor& vis,
      const typename boost::graph_traits<Graph>::vertex_descriptor start,
      VertexAssignmentMap vam,
      KeyedUpdatablePriorityQueue& pq
    )
    {
      MASVisitor mas_vis = vis;
      maximum_adjacency_search_impl(g, weights, mas_vis, start, vam, pq);
    }
#endif  // not MSVC-14.0 w/64-bit addressing
}} // end namespace boost::detail

namespace boost { namespace graph {

    template <
        typename Graph, typename WeightMap, typename MASVisitor,
        typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue
    >
    inline void maximum_adjacency_search(
        const Graph& g, WeightMap weights,
#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
        MASVisitor& vis,
#else
        MASVisitor vis,
#endif
        const typename boost::graph_traits<Graph>::vertex_descriptor start,
        VertexAssignmentMap assignments, KeyedUpdatablePriorityQueue pq
    )
    {
        boost::detail::maximum_adjacency_search_impl(
            g, weights, vis, start, assignments, pq
        );
    }
}} // end namespace boost::graph

#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )

namespace boost { namespace graph {

    template <
        typename Graph, typename WeightMap, typename MASVisitor,
        typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue
    >
    inline void maximum_adjacency_search(
        const Graph& g, WeightMap weights, const MASVisitor& vis,
        const typename boost::graph_traits<Graph>::vertex_descriptor start,
        VertexAssignmentMap assignments, KeyedUpdatablePriorityQueue pq
    )
    {
        boost::detail::maximum_adjacency_search_impl(
            g, weights, vis, start, assignments, pq
        );
    }

    template <typename Graph, typename Args>
    void maximum_adjacency_search(
        const Graph& g, const Args& arg_pack, typename boost::enable_if<
            parameter::is_argument_pack<Args>, mpl::true_
        >::type = mpl::true_()
    )
    {
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            Graph
        >::type WeightMap;
        WeightMap w_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_weight_map,
            g,
            edge_weight
        );
        typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                mas_visitor<>
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<mas_visitor<> >()
        ];
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        Vertex s = arg_pack[
            boost::graph::keywords::_root_vertex ||
            boost::detail::get_default_starting_vertex_t<Graph>(g)
        ];
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::vertex_assignment_map,
            Vertex
        > vertex_assignment_map_gen(graph_traits<Graph>::null_vertex());
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::vertex_assignment_map,
            Vertex
        >::map_type vert_asgn_map = vertex_assignment_map_gen(g, arg_pack);
        typedef typename boost::property_traits<WeightMap>::value_type Weight;
        const Weight zero_weight = Weight();
        typedef boost::detail::make_priority_queue_from_arg_pack_gen<
            boost::graph::keywords::tag::max_priority_queue,
            Weight,
            Vertex,
            std::greater<Weight>
        > PQGenerator;
        PQGenerator pq_gen(
            choose_param(
                get_param(arg_pack, boost::distance_zero_t()),
                zero_weight
            )
        );
        typename PQGenerator::BOOST_NESTED_TEMPLATE result<
            PQGenerator(const Graph&, const Args&)
        >::type pq = pq_gen(g, arg_pack);
        boost::detail::maximum_adjacency_search_impl(
            g, w_map, vis, s, vert_asgn_map, pq
        );
    }
}} // end namespace boost::graph

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
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
        name( \
            g, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 9, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, maximum_adjacency_search
)
}} // end namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

#else   // MSVC-14.0 w/64-bit addressing

namespace boost { namespace graph { namespace detail {

    template <typename Graph>
    struct maximum_adjacency_search_impl
    {
        typedef void result_type;
        typedef result_type type;

        template <typename ArgPack>
        void operator()(const Graph& g, const ArgPack& arg_pack) const
        {
            typedef typename boost::detail::override_const_property_result<
                ArgPack,
                boost::graph::keywords::tag::weight_map,
                edge_weight_t,
                Graph
            >::type WeightMap;
            WeightMap w_map = boost::detail::override_const_property(
                arg_pack,
                boost::graph::keywords::_weight_map,
                g,
                edge_weight
            );
            typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
            const Vertex s = arg_pack[
                boost::graph::keywords::_root_vertex ||
                boost::detail::get_default_starting_vertex_t<Graph>(g)
            ];
            typedef typename property_traits<WeightMap>::value_type Weight;
            const Weight zero_weight = Weight();
            typedef boost::detail::make_priority_queue_from_arg_pack_gen<
                boost::graph::keywords::tag::max_priority_queue, Weight,
                Vertex, std::greater<Weight>
            > default_pq_gen_type;
            default_pq_gen_type pq_gen(
                choose_param(
                    get_param(arg_pack, boost::distance_zero_t()), zero_weight
                )
            );
            typename boost::result_of<
                default_pq_gen_type(const Graph&, const ArgPack&)
            >::type pq = pq_gen(g, arg_pack);
            typename boost::remove_const<
                typename boost::parameter::value_type<
                    ArgPack,
                    boost::graph::keywords::tag::visitor,
                    mas_visitor<>
                >::type
            >::type vis = arg_pack[
                boost::graph::keywords::_visitor ||
                boost::value_factory<mas_visitor<> >()
            ];
            boost::detail::make_property_map_from_arg_pack_gen<
                boost::graph::keywords::tag::vertex_assignment_map,
                Vertex
            > v_a_map_gen(graph_traits<Graph>::null_vertex());
            typename boost::detail::map_maker<
                Graph,
                ArgPack,
                boost::graph::keywords::tag::vertex_assignment_map,
                Vertex
            >::map_type v_a_map = v_a_map_gen(g, arg_pack);
            maximum_adjacency_search(g, w_map, vis, s, v_a_map, pq);
        }
    };
}}} // end namespace boost::graph::detail

namespace boost { namespace graph {

    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(maximum_adjacency_search, 1, 9)
}} // end namespace boost::graph

#endif  // not MSVC-14.0 w/64-bit addressing

namespace boost {

    using ::boost::graph::maximum_adjacency_search;
    using ::boost::graph::mas_visitor;
    using ::boost::graph::make_mas_visitor;
    using ::boost::graph::default_mas_visitor;

    // Old-style named parameter interface
    template <typename Graph, typename P, typename T, typename R>
    void maximum_adjacency_search(
        const Graph& g, const bgl_named_params<P, T, R>& params
    )
    {
        typedef bgl_named_params<P, T, R> params_type;
        BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
        maximum_adjacency_search(g, arg_pack);
    }
} // end namespace boost

#include <boost/graph/iteration_macros_undef.hpp>

#endif // BOOST_GRAPH_MAXIMUM_ADJACENCY_SEARCH_H
