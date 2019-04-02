//            Copyright Daniel Trebbien 2010.
// Distributed under the Boost Software License, Version 1.0.
//   (See accompanying file LICENSE_1_0.txt or the copy at
//         http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GRAPH_STOER_WAGNER_MIN_CUT_HPP
#define BOOST_GRAPH_STOER_WAGNER_MIN_CUT_HPP 1

#include <boost/assert.hpp>
#include <set>
#include <vector>
#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/buffer_concepts.hpp>
#include <boost/graph/exception.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/maximum_adjacency_search.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/one_bit_color_map.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/graph/detail/d_ary_heap.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/utility/result_of.hpp>
#include <boost/graph/iteration_macros.hpp>

namespace boost { namespace detail {

    template < typename ParityMap, typename WeightMap, typename IndexMap >
    class mas_min_cut_visitor : public boost::default_mas_visitor
    {
        typedef one_bit_color_map<IndexMap> InternalParityMap;
        typedef typename boost::property_traits<WeightMap>::value_type Weight;

    public:
        template < typename Graph >
        mas_min_cut_visitor(
            const Graph& g, ParityMap parity, Weight& cutweight,
            const WeightMap& weight_map, IndexMap index_map
        ) : m_bestParity(parity),
            m_parity(make_one_bit_color_map(num_vertices(g), index_map)),
            m_bestWeight(cutweight),
            m_cutweight(0),
            m_visited(0),
            m_weightMap(weight_map)
        {
            // set here since the init list sets the reference
            this->m_bestWeight = (std::numeric_limits<Weight>::max)();
        }

        template < typename Vertex, typename Graph >
        inline void initialize_vertex(Vertex u, const Graph& g)
        {
            typedef typename boost::property_traits<
                ParityMap
            >::value_type parity_type;
            typedef typename boost::property_traits<
                InternalParityMap
            >::value_type internal_parity_type;

            put(this->m_parity, u, internal_parity_type(0));
            put(this->m_bestParity, u, parity_type(0));
        }

        template < typename Edge, typename Graph >
        inline void examine_edge(Edge e, const Graph& g)
        {
            Weight w = get(this->m_weightMap, e);

            // if the target of e is already marked then decrease cutweight
            // otherwise, increase it
            if (get(this->m_parity, boost::target(e, g)))
            {
                this->m_cutweight -= w;
            }
            else
            {
                this->m_cutweight += w;
            }
        }

        template < typename Vertex, typename Graph >
        void finish_vertex(Vertex u, const Graph& g)
        {
            typedef typename boost::property_traits<
                InternalParityMap
            >::value_type internal_parity_type;

            ++this->m_visited;
            put(this->m_parity, u, internal_parity_type(1));

            if (
                (this->m_cutweight < this->m_bestWeight) &&
                (this->m_visited < num_vertices(g))
            )
            {
                this->m_bestWeight = this->m_cutweight;

                BGL_FORALL_VERTICES_T(i, g, Graph)
                {
                    put(this->m_bestParity, i, get(this->m_parity, i));
                }
            }
        }

        inline void clear()
        {
            this->m_bestWeight = (std::numeric_limits<Weight>::max)();
            this->m_visited = 0;
            this->m_cutweight = 0;
        }

    private:
        ParityMap m_bestParity;
        InternalParityMap m_parity;
        Weight& m_bestWeight;
        Weight m_cutweight;
        unsigned m_visited;
        const WeightMap& m_weightMap;
    };

    /**
     * \brief Computes a min-cut of the input graph
     *
     * Computes a min-cut of the input graph using the Stoer-Wagner algorithm.
     *
     * \pre \p g is a connected, undirected graph
     * \pre <code>pq.empty()</code>
     * \param[in] g the input graph
     * \param[in] weights a readable property map from each edge to its weight (a non-negative value)
     * \param[out] parities a writable property map from each vertex to a bool type object for
     *     distinguishing the two vertex sets of the min-cut
     * \param[out] assignments a read/write property map from each vertex to a \c vertex_descriptor object. This
     *     map serves as work space, and no particular meaning should be derived from property values
     *     after completion of the algorithm.
     * \param[out] pq a keyed, updatable max-priority queue
     * \returns the cut weight of the min-cut
     * \see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.114.6687&rep=rep1&type=pdf
     * \see http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.31.614&rep=rep1&type=pdf
     *
     * \author Daniel Trebbien
     * \date 2010-09-11
     */
    template <
      typename UndirectedGraph, typename WeightMap, typename ParityMap,
      typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue,
      typename IndexMap
    >
    typename boost::property_traits<WeightMap>::value_type
    stoer_wagner_min_cut_impl(
      const UndirectedGraph& g, WeightMap weights, ParityMap parities,
      VertexAssignmentMap assignments,
      KeyedUpdatablePriorityQueue& pq,
      IndexMap index_map
    )
    {
      typedef typename boost::graph_traits<UndirectedGraph>::vertex_descriptor vertex_descriptor;
      typedef typename boost::property_traits<WeightMap>::value_type weight_type;

      typename graph_traits<UndirectedGraph>::vertex_iterator u_iter, u_end;

      weight_type bestW = (std::numeric_limits<weight_type>::max)();
      weight_type bestThisTime = (std::numeric_limits<weight_type>::max)();
      vertex_descriptor bestStart = boost::graph_traits<UndirectedGraph>::null_vertex();

      detail::mas_min_cut_visitor<ParityMap, WeightMap, IndexMap>
        vis(g, parities, bestThisTime, weights, index_map);

      // for each node in the graph,
      for (boost::tie(u_iter, u_end) = vertices(g); u_iter != u_end; ++u_iter) {
        // run the MAS and find the min cut
        vis.clear();
        boost::maximum_adjacency_search(
            g,
            boost::graph::keywords::_weight_map = weights,
            boost::graph::keywords::_visitor = vis,
            boost::graph::keywords::_root_vertex = *u_iter,
            boost::graph::keywords::_vertex_assignment_map = assignments,
            boost::graph::keywords::_max_priority_queue = pq
        );
        if (bestThisTime < bestW) {
          bestW = bestThisTime;
          bestStart = *u_iter;
        }
      }

      // Run one more time, starting from the best start location, to
      // ensure the visitor has the best values.
      vis.clear();
      boost::maximum_adjacency_search(
        g,
        boost::graph::keywords::_vertex_assignment_map = assignments,
        boost::graph::keywords::_weight_map = weights,
        boost::graph::keywords::_visitor = vis,
        boost::graph::keywords::_root_vertex = bestStart,
        boost::graph::keywords::_max_priority_queue = pq
      );

//      BOOST_ASSERT(bestW == bestThisTime);
      return bestW;
    }
}} // end namespace boost::detail

namespace boost { namespace graph {

    template <
        typename UndirectedGraph, typename WeightMap, typename ParityMap,
        typename VertexAssignmentMap, typename KeyedUpdatablePriorityQueue,
        typename IndexMap
    >
    typename boost::disable_if<
        parameter::are_tagged_arguments<
            ParityMap, VertexAssignmentMap,
            KeyedUpdatablePriorityQueue, IndexMap
        >,
        typename boost::property_traits<WeightMap>::value_type
    >::type
    stoer_wagner_min_cut(
        const UndirectedGraph& g, WeightMap weights, ParityMap parities,
        VertexAssignmentMap assignments, KeyedUpdatablePriorityQueue& pq,
        IndexMap index_map
    )
    {
        BOOST_CONCEPT_ASSERT((
            boost::IncidenceGraphConcept<UndirectedGraph>
        ));
        BOOST_CONCEPT_ASSERT((
            boost::VertexListGraphConcept<UndirectedGraph>
        ));
        typedef typename graph_traits<
            UndirectedGraph
        >::vertex_descriptor vertex_descriptor;
        typedef typename graph_traits<
            UndirectedGraph
        >::vertices_size_type vertices_size_type;
        typedef typename graph_traits<
            UndirectedGraph
        >::edge_descriptor edge_descriptor;
        BOOST_CONCEPT_ASSERT((
            boost::Convertible<
                typename graph_traits<UndirectedGraph>::directed_category,
                boost::undirected_tag
            >
        ));
        BOOST_CONCEPT_ASSERT((
            boost::ReadablePropertyMapConcept<WeightMap, edge_descriptor>
        ));
        // typedef typename boost::property_traits<
        //     WeightMap
        // >::value_type weight_type;
        BOOST_CONCEPT_ASSERT((
            boost::WritablePropertyMapConcept<ParityMap, vertex_descriptor>
        ));
        // typedef typename boost::property_traits<
        //     ParityMap
        // >::value_type parity_type;
        BOOST_CONCEPT_ASSERT((
            boost::ReadWritePropertyMapConcept<
                VertexAssignmentMap,
                vertex_descriptor
            >
        ));
        BOOST_CONCEPT_ASSERT((
            boost::Convertible<
                vertex_descriptor,
                typename boost::property_traits<
                    VertexAssignmentMap
                >::value_type
            >
        ));
        BOOST_CONCEPT_ASSERT((
            boost::KeyedUpdatableQueueConcept<KeyedUpdatablePriorityQueue>
        ));

        vertices_size_type n = num_vertices(g);
        if (n < 2)
            throw boost::bad_graph(
                "the input graph must have at least two vertices."
            );
        else if (!pq.empty())
            throw std::invalid_argument(
                "the max-priority queue must be empty initially."
            );

        return boost::detail::stoer_wagner_min_cut_impl(
            g, weights, parities, assignments, pq, index_map
        );
    }
}} // end namespace boost::graph

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)

namespace boost { namespace graph {

    template <typename Graph, typename WeightMap, typename Args>
    typename boost::enable_if<
        parameter::is_argument_pack<Args>,
        typename boost::property_traits<WeightMap>::value_type
    >::type
    stoer_wagner_min_cut(
        const Graph& g, WeightMap weights, const Args& arg_pack
    )
    {
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            Graph
        >::type v_i_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
        );
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::vertex_assignment_map,
            Vertex
        > vertex_assignment_map_gen(graph_traits<Graph>::null_vertex());
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::vertex_assignment_map,
            Vertex
        >::map_type v_a_map = vertex_assignment_map_gen(g, arg_pack);
        typedef typename graph_traits<Graph>::vertices_size_type VIndex;
        const VIndex zero_index = VIndex();
        dummy_property_map dummy_prop;
        typename boost::parameter::binding<
            Args,
            boost::graph::keywords::tag::parity_map,
            dummy_property_map&
        >::type p_map = arg_pack[
            boost::graph::keywords::_parity_map | dummy_prop
        ];
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::index_in_heap_map,
            VIndex
        > i_in_h_map_gen(zero_index);
        typedef typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::index_in_heap_map,
            VIndex
        >::map_type IndexInHeapPMap;
        IndexInHeapPMap i_in_h_map = i_in_h_map_gen(g, arg_pack);
        typedef typename boost::property_traits<WeightMap>::value_type D;
        const D zero_distance = D();
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::distance_map,
            D
        > dist_map_gen(zero_distance);
        typedef typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::distance_map,
            D
        >::map_type DistanceMap;
        DistanceMap dist_map = dist_map_gen(g, arg_pack);
        typedef d_ary_heap_indirect<
            Vertex,
            4,
            IndexInHeapPMap,
            DistanceMap,
            std::greater<Vertex>
        > DefaultBuffer;
        DefaultBuffer d_buf(dist_map, i_in_h_map, std::greater<Vertex>());
        typename parameter::binding<
            Args,
            boost::graph::keywords::tag::buffer,
            DefaultBuffer&
        >::type Q = arg_pack[boost::graph::keywords::_buffer | d_buf];
        return stoer_wagner_min_cut(g, weights, p_map, v_a_map, Q, v_i_map);
    }
}} // end namespace boost::graph

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename WeightMap, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline typename boost::enable_if< \
        parameter::are_tagged_arguments< \
            TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
        >, \
        typename boost::property_traits<WeightMap>::value_type \
    >::type \
    name( \
        const Graph& g, WeightMap weights, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta) \
    ) \
    { \
        return name( \
            g, weights, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 7, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, stoer_wagner_min_cut
)
}} // end namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost { namespace graph {

    template <typename Graph, typename WeightMap>
    inline typename boost::property_traits<WeightMap>::value_type
    stoer_wagner_min_cut(const Graph& g, WeightMap weights)
    {
        return stoer_wagner_min_cut(g, weights, parameter::compose());
    }
}} // end namespace boost::graph

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)

namespace boost { namespace graph { namespace detail {

    template <class UndirectedGraph, class WeightMap>
    struct stoer_wagner_min_cut_impl {
      typedef typename boost::property_traits<WeightMap>::value_type result_type;
      typedef result_type type;
      template <typename ArgPack>
      result_type operator() (const UndirectedGraph& g, WeightMap weights, const ArgPack& arg_pack) const {
        using namespace boost::graph::keywords;
        typedef typename boost::graph_traits<UndirectedGraph>::vertex_descriptor vertex_descriptor;
        typedef typename boost::property_traits<WeightMap>::value_type weight_type;

        typedef boost::detail::make_priority_queue_from_arg_pack_gen<boost::graph::keywords::tag::max_priority_queue, weight_type, vertex_descriptor, std::greater<weight_type> > gen_type;

        gen_type gen(choose_param(get_param(arg_pack, boost::distance_zero_t()), weight_type(0)));

        typename boost::result_of<gen_type(const UndirectedGraph&, const ArgPack&)>::type pq = gen(g, arg_pack);

        boost::dummy_property_map dummy_prop;
        return stoer_wagner_min_cut(
          g,
          weights,
          arg_pack [_parity_map | dummy_prop],
          boost::detail::make_property_map_from_arg_pack_gen<tag::vertex_assignment_map, vertex_descriptor>(vertex_descriptor())(g, arg_pack),
          pq,
          boost::detail::override_const_property(arg_pack, _vertex_index_map, g, vertex_index)
        );
      }
    };
}}} // end namespace boost::graph::detail

namespace boost { namespace graph {

    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(stoer_wagner_min_cut,2,4)
}} // end namespace boost::graph

#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

namespace boost { namespace graph {

    // version without IndexMap kept for backwards compatibility
    // (but requires vertex_index_t to be defined in the graph)
    // Place after the macro to avoid compilation errors
    template <class UndirectedGraph, class WeightMap, class ParityMap, class VertexAssignmentMap, class KeyedUpdatablePriorityQueue>
    typename boost::property_traits<WeightMap>::value_type
    stoer_wagner_min_cut(const UndirectedGraph& g, WeightMap weights, ParityMap parities, VertexAssignmentMap assignments, KeyedUpdatablePriorityQueue& pq) {

      return stoer_wagner_min_cut(g, weights,
                                  parities, assignments, pq,
                                  get(vertex_index, g));
    }
}} // end namespace boost::graph

namespace boost {

    using ::boost::graph::stoer_wagner_min_cut;

    // Old-style named parameter variant
    template <
        typename Graph, typename WeightMap, typename P, typename T, typename R
    >
    typename boost::property_traits<WeightMap>::value_type
    stoer_wagner_min_cut(
        const Graph& g, WeightMap weights,
        const bgl_named_params<P, T, R>& params
    )
    {
        typedef bgl_named_params<P, T, R> params_type;
        BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
        return stoer_wagner_min_cut(g, weights, arg_pack);
    }
} // end namespace boost

#include <boost/graph/iteration_macros_undef.hpp>

#endif // !BOOST_GRAPH_STOER_WAGNER_MIN_CUT_HPP
