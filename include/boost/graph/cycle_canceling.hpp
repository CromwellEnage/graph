//============================================================================
// Copyright 2013 University of Warsaw.
// Authors: Piotr Wygocki
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================
//
//
// This algorithm is described in
// "Network Flows: Theory, Algorithms, and Applications"
// by Ahuja, Magnanti, Orlin.

#ifndef BOOST_GRAPH_CYCLE_CANCELING_HPP
#define BOOST_GRAPH_CYCLE_CANCELING_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/visitors.hpp>
#include <boost/graph/bellman_ford_shortest_paths.hpp>
#include <boost/property_map/property_map.hpp>

namespace boost { namespace detail {

    template <typename PredEdgeMap, typename Vertex>
    class RecordEdgeMapAndCycleVertex
        : public bellman_visitor<
            edge_predecessor_recorder<PredEdgeMap, on_edge_relaxed>
        >
    {
        typedef edge_predecessor_recorder<
            PredEdgeMap, on_edge_relaxed
        > PredRec;

        Vertex& m_v;
        PredEdgeMap m_pred;

    public:
        RecordEdgeMapAndCycleVertex(PredEdgeMap pred, Vertex& v)
          : bellman_visitor<PredRec>(PredRec(pred)), m_v(v), m_pred(pred)
        {
        }

        template <typename Graph, typename Edge>
        void edge_not_minimized(Edge e, const Graph& g) const
        {
            // Edge e is not minimized but does not have to be on the
            // negative weight cycle.  To find vertex m_v on negative weight
            // cycle, we move n+1 times backword in the PredEdgeMap graph.
            for (
                typename graph_traits<
                    Graph
                >::vertices_size_type n = num_vertices(g) + 1;
                n > 0;
                --n
            )
            {
                e = get(this->m_pred, source(e, g));
            }

            this->m_v = source(e, g);
        }
    };
}} // namespace boost::detail

#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/augment.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    template <
        typename Graph, typename Weight, typename Reversed,
        typename ResidualCapacity, typename Pred, typename Distance
    >
    void cycle_canceling(
        const Graph& g, Weight weight, Reversed rev,
        ResidualCapacity residual_capacity, Pred pred, Distance distance,
        typename boost::disable_if<
            parameter::are_tagged_arguments<
                Weight, Reversed, ResidualCapacity, Pred, Distance
            >,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typedef filtered_graph<
            const Graph, is_residual_edge<ResidualCapacity>
        > ResGraph;
        typedef graph_traits<ResGraph> ResGTraits;
        typedef graph_traits<Graph> GTraits;
        typedef typename ResGTraits::edge_descriptor edge_descriptor;
        typedef typename ResGTraits::vertex_descriptor vertex_descriptor;

        ResGraph gres = boost::detail::residual_graph(g, residual_capacity);
        typename GTraits::vertices_size_type N = num_vertices(g);

        BGL_FORALL_VERTICES_T(v, g, Graph)
        {
            put(pred, v, edge_descriptor());
            put(distance, v, 0);
        }

        vertex_descriptor cycleStart;

        while(
            !bellman_ford_shortest_paths(
                gres,
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
                boost::graph::keywords::_size = N,
                boost::graph::keywords::_weight_map = weight,
                boost::graph::keywords::_distance_map = distance,
                boost::graph::keywords::_visitor =
#else
                N,
                weight_map(weight).distance_map(distance).visitor(
#endif
                    boost::detail::RecordEdgeMapAndCycleVertex<
                        Pred, vertex_descriptor
                    >(pred, cycleStart)
#if !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
                )
#endif
            )
        )
        {
            boost::detail::augment(
                g, cycleStart, cycleStart, pred, residual_capacity, rev
            );

            BGL_FORALL_VERTICES_T(v, g, Graph)
            {
                put(pred, v, edge_descriptor());
                put(distance, v, 0);
            }
        }
    }
}} // namespace boost::graph

#include <boost/parameter/is_argument_pack.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename Args>
    void cycle_canceling(
        Graph& g, const Args& arg_pack, typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename boost::detail::override_property_result<
            Args,
            boost::graph::keywords::tag::residual_capacity_map,
            edge_residual_capacity_t,
            Graph
        >::type e_rc_map = boost::detail::override_property(
            arg_pack,
            boost::graph::keywords::_residual_capacity_map,
            g,
            edge_residual_capacity
        );
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::reverse_edge_map,
            edge_reverse_t,
            Graph
        >::type e_rv_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_reverse_edge_map,
            g,
            edge_reverse
        );
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
        typedef typename boost::property_traits<WeightMap>::value_type D;
        const D zero_distance = D();
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::distance_map,
            D
        > v_d_map_gen(zero_distance);
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::distance_map,
            D
        >::map_type v_d_map = v_d_map_gen(g, arg_pack);
        typedef typename graph_traits<Graph>::edge_descriptor Edge;
        const Edge no_edge = Edge();
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::predecessor_map,
            Edge
        > v_p_map_gen(no_edge);
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::predecessor_map,
            Edge
        >::map_type v_p_map = v_p_map_gen(g, arg_pack);
        cycle_canceling(g, e_w_map, e_rv_map, e_rc_map, v_p_map, v_d_map);
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <typename Graph>
    inline void cycle_canceling(Graph& g)
    {
        cycle_canceling(g, parameter::compose());
    }
}} // namespace boost::graph

#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        Graph& g, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta) \
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
    1, 4, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, cycle_canceling
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD
#include <boost/graph/find_flow_cost.hpp>
#include <vector>
#include <numeric>

//in this namespace argument dispatching takes place
namespace boost { namespace detail {

template <class Graph, class P, class T, class R, class ResidualCapacity, class Weight, class Reversed, class Pred, class Distance>
void cycle_canceling_dispatch2(
        const Graph &g,
        Weight weight,
        Reversed rev,
        ResidualCapacity residual_capacity,
        Pred pred,
        Distance dist,
        const bgl_named_params<P, T, R>& params) {
    cycle_canceling(g, weight, rev, residual_capacity, pred, dist);
}

//setting default distance map
template <class Graph, class P, class T, class R, class Pred, class ResidualCapacity, class Weight, class Reversed>
void cycle_canceling_dispatch2(
        Graph &g,
        Weight weight,
        Reversed rev,
        ResidualCapacity residual_capacity,
        Pred pred,
        param_not_found,
        const bgl_named_params<P, T, R>& params) {
    typedef typename property_traits<Weight>::value_type D;

    std::vector<D> d_map(num_vertices(g));

    cycle_canceling(g, weight, rev, residual_capacity, pred,
                    make_iterator_property_map(d_map.begin(), choose_const_pmap(get_param(params, vertex_index), g, vertex_index)));
}

template <class Graph, class P, class T, class R, class ResidualCapacity, class Weight, class Reversed, class Pred>
void cycle_canceling_dispatch1(
        Graph &g,
        Weight weight,
        Reversed rev,
        ResidualCapacity residual_capacity,
        Pred pred,
        const bgl_named_params<P, T, R>& params) {
    cycle_canceling_dispatch2(g, weight, rev,residual_capacity,  pred,
                                get_param(params, vertex_distance), params);
}

//setting default predecessors map
template <class Graph, class P, class T, class R, class ResidualCapacity, class Weight, class Reversed>
void cycle_canceling_dispatch1(
        Graph &g,
        Weight weight,
        Reversed rev,
        ResidualCapacity residual_capacity,
        param_not_found,
        const bgl_named_params<P, T, R>& params) {
    typedef typename graph_traits<Graph>::edge_descriptor edge_descriptor;
    std::vector<edge_descriptor> p_map(num_vertices(g));

    cycle_canceling_dispatch2(g, weight, rev, residual_capacity,
                              make_iterator_property_map(p_map.begin(), choose_const_pmap(get_param(params, vertex_index), g, vertex_index)),
                                get_param(params, vertex_distance), params);
}
}} // namespace boost::detail

namespace boost {

    using ::boost::graph::cycle_canceling;

    template <typename Graph, typename P, typename T, typename R>
    void cycle_canceling(Graph& g, const bgl_named_params<P, T, R>& params)
    {
        cycle_canceling_dispatch1(
            g,
            choose_const_pmap(get_param(params, edge_weight), g, edge_weight),
            choose_const_pmap(get_param(params, edge_reverse), g,
                              edge_reverse),
            choose_pmap(get_param(params, edge_residual_capacity),
                        g, edge_residual_capacity),
            get_param(params, vertex_predecessor),
            params
        );
    }
}

#endif /* BOOST_GRAPH_CYCLE_CANCELING_HPP */
