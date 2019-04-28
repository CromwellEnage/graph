//=======================================================================
// Copyright 2013 University of Warsaw.
// Authors: Piotr Wygocki 
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
#ifndef BOOST_GRAPH_FIND_FLOW_COST_HPP
#define BOOST_GRAPH_FIND_FLOW_COST_HPP

#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

template <
    typename Graph, typename Capacity, typename ResidualCapacity,
    typename Weight
>
typename boost::disable_if<
    parameter::are_tagged_arguments<Capacity, ResidualCapacity, Weight>,
    typename property_traits<Weight>::value_type
>::type
find_flow_cost(
    const Graph& g, Capacity capacity, ResidualCapacity residual_capacity,
    Weight weight
)
{
    typedef typename property_traits<Weight>::value_type Cost;
    const Cost zero_cost = Cost();
    Cost cost = zero_cost;

    BGL_FORALL_EDGES_T(e, g, Graph)
    {
        if (zero_cost < get(capacity, e))
        {
            cost += (
                get(capacity, e) - get(residual_capacity, e)
            ) * get(weight, e);
        }
    }

    return cost;
}
}} // namespace boost::graph

#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/is_argument_pack.hpp>

namespace boost { namespace graph {

template <typename Graph, typename Args> 
inline typename boost::lazy_enable_if<
    parameter::is_argument_pack<Args>,
    boost::detail::graph_or_arg_packed_property_map_value<
        Graph,
        edge_weight_t,
        Args,
        boost::graph::keywords::tag::weight_map
    >
>::type find_flow_cost(const Graph& g, const Args& args)
{
    return find_flow_cost(
        g,
        boost::detail::override_const_property(
            args,
            boost::graph::keywords::_capacity_map,
            g,
            edge_capacity
        ),
        boost::detail::override_const_property(
            args,
            boost::graph::keywords::_residual_capacity_map,
            g,
            edge_residual_capacity
        ),
        boost::detail::override_const_property(
            args,
            boost::graph::keywords::_weight_map,
            g,
            edge_weight
        )
    );
}
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

template <typename Graph>
inline typename property_traits<
    typename property_map< Graph, edge_weight_t >::type
>::value_type
find_flow_cost(const Graph& g)
{
    return find_flow_cost(g, parameter::compose());
}
}} // namespace boost::graph

#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline typename boost::lazy_enable_if< \
        parameter::are_tagged_arguments< \
            TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
        >, \
        boost::detail::graph_or_tagged_args_property_map_value< \
            Graph, boost::edge_weight_t, \
            boost::graph::keywords::tag::weight_map, \
            TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
        > \
    >::type name( \
        const Graph& g, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta) \
    ) \
    { \
        return name( \
            g, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 4, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, find_flow_cost
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

using ::boost::graph::find_flow_cost;

template <typename Graph, typename P, typename T, typename R> 
typename boost::detail::edge_weight_value<Graph, P, T, R>::type
find_flow_cost(const Graph& g, const bgl_named_params<P, T, R>& params)
{
    return find_flow_cost(g,
           choose_const_pmap(get_param(params, edge_capacity), g, edge_capacity),
           choose_const_pmap(get_param(params, edge_residual_capacity), 
                       g, edge_residual_capacity),
           choose_const_pmap(get_param(params, edge_weight), g, edge_weight));
}
} // namespace boost

#endif /* BOOST_GRAPH_FIND_FLOW_COST_HPP */

