// Copyright 2002 Rensselaer Polytechnic Institute

// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  Authors: Lauren Foutz
//           Scott Hill

/*
  This file implements the functions

  template <class VertexListGraph, class DistanceMatrix, 
    class P, class T, class R>
  bool floyd_warshall_initialized_all_pairs_shortest_paths(
    const VertexListGraph& g, DistanceMatrix& d, 
    const bgl_named_params<P, T, R>& params)

  AND

  template <class VertexAndEdgeListGraph, class DistanceMatrix, 
    class P, class T, class R>
  bool floyd_warshall_all_pairs_shortest_paths(
    const VertexAndEdgeListGraph& g, DistanceMatrix& d, 
    const bgl_named_params<P, T, R>& params)
*/


#ifndef BOOST_GRAPH_FLOYD_WARSHALL_HPP
#define BOOST_GRAPH_FLOYD_WARSHALL_HPP

namespace boost { namespace detail {

    template <typename T, typename BinaryPredicate>
    T min_with_compare(const T& x, const T& y, const BinaryPredicate& compare)
    {
        if (compare(x, y)) return x;
        else return y;
    }
}}

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/relax.hpp>
#include <boost/property_map/property_map.hpp>

namespace boost { namespace detail {

    template <
        typename VertexListGraph, typename DistanceMatrix,
        typename BinaryPredicate, typename BinaryFunction,
        typename Infinity, typename Zero
    >
    bool floyd_warshall_dispatch(
        const VertexListGraph& g, DistanceMatrix& d,
        const BinaryPredicate& compare, const BinaryFunction& combine,
        const Infinity& inf, const Zero& zero
    )
    {
        typename graph_traits<
            VertexListGraph
        >::vertex_iterator i, lasti, j, lastj, k, lastk;

        for (boost::tie(k, lastk) = vertices(g); k != lastk; k++)
            for (boost::tie(i, lasti) = vertices(g); i != lasti; i++)
                if (d[*i][*k] != inf)
                    for (boost::tie(j, lastj) = vertices(g); j != lastj; j++)
                        if (d[*k][*j] != inf)
                            d[*i][*j] = detail::min_with_compare(
                                d[*i][*j], combine(d[*i][*k], d[*k][*j]),
                                compare
                            );

        for (boost::tie(i, lasti) = vertices(g); i != lasti; i++)
            if (compare(d[*i][*i], zero))
                return false;

        return true;
    }
}} // namespace boost::detail

#include <boost/graph/graph_concepts.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/concept/assert.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    template <
        typename VertexListGraph, typename DistanceMatrix,
        typename BinaryPredicate, typename BinaryFunction,
        typename Infinity, typename Zero
    >
    inline typename boost::disable_if<
        parameter::are_tagged_arguments<
            BinaryPredicate, BinaryFunction, Infinity, Zero
        >,
        bool
    >::type
    floyd_warshall_initialized_all_pairs_shortest_paths(
        const VertexListGraph& g, DistanceMatrix& d,
        const BinaryPredicate& compare, const BinaryFunction& combine,
        const Infinity& inf, const Zero& zero
    )
    {
        BOOST_CONCEPT_ASSERT(( VertexListGraphConcept<VertexListGraph> ));
        return boost::detail::floyd_warshall_dispatch(
            g, d, compare, combine, inf, zero
        );
    }

  template <typename VertexAndEdgeListGraph, typename DistanceMatrix, 
    typename WeightMap, typename BinaryPredicate, 
    typename BinaryFunction, typename Infinity, typename Zero>
  typename boost::disable_if<
    parameter::are_tagged_arguments<
      WeightMap, BinaryPredicate, BinaryFunction, Infinity, Zero
    >,
    bool
  >::type
  floyd_warshall_all_pairs_shortest_paths(
    const VertexAndEdgeListGraph& g, 
    DistanceMatrix& d, const WeightMap& w, 
    const BinaryPredicate& compare, const BinaryFunction& combine, 
    const Infinity& inf, const Zero& zero)
  {
    BOOST_CONCEPT_ASSERT(( VertexListGraphConcept<VertexAndEdgeListGraph> ));
    BOOST_CONCEPT_ASSERT(( EdgeListGraphConcept<VertexAndEdgeListGraph> ));
    BOOST_CONCEPT_ASSERT(( IncidenceGraphConcept<VertexAndEdgeListGraph> ));
  
    typename graph_traits<VertexAndEdgeListGraph>::vertex_iterator 
      firstv, lastv, firstv2, lastv2;
    typename graph_traits<VertexAndEdgeListGraph>::edge_iterator first, last;
  
    
    for(boost::tie(firstv, lastv) = vertices(g); firstv != lastv; firstv++)
      for(boost::tie(firstv2, lastv2) = vertices(g); firstv2 != lastv2; firstv2++)
        d[*firstv][*firstv2] = inf;
    
    
    for(boost::tie(firstv, lastv) = vertices(g); firstv != lastv; firstv++)
      d[*firstv][*firstv] = zero;
    
    
    for(boost::tie(first, last) = edges(g); first != last; first++)
    {
      if (d[source(*first, g)][target(*first, g)] != inf) {
        d[source(*first, g)][target(*first, g)] = 
          boost::detail::min_with_compare(
            get(w, *first), 
            d[source(*first, g)][target(*first, g)],
            compare);
      } else 
        d[source(*first, g)][target(*first, g)] = get(w, *first);
    }
    
    bool is_undirected = is_same<typename 
      graph_traits<VertexAndEdgeListGraph>::directed_category, 
      undirected_tag>::value;
    if (is_undirected)
    {
      for(boost::tie(first, last) = edges(g); first != last; first++)
      {
        if (d[target(*first, g)][source(*first, g)] != inf)
          d[target(*first, g)][source(*first, g)] = 
            boost::detail::min_with_compare(
              get(w, *first), 
              d[target(*first, g)][source(*first, g)],
              compare);
        else 
          d[target(*first, g)][source(*first, g)] = get(w, *first);
      }
    }
    
  
    return boost::detail::floyd_warshall_dispatch(g, d, compare, combine, 
      inf, zero);
  }
}} // namespace boost::graph

#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/functional/value_factory.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <functional>

namespace boost { namespace graph {

    template <typename Graph, typename DistanceMatrix, typename Args>
    typename boost::enable_if<
        parameter::is_argument_pack<Args>,
        bool
    >::type
    floyd_warshall_initialized_all_pairs_shortest_paths(
        const Graph& g1, DistanceMatrix& D, const Args& arg_pack
    )
    {
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            Graph
        >::type WeightMap;
        typedef typename boost::property_traits<WeightMap>::value_type Weight;
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_compare,
                std::less<Weight>
            >::type
        >::type dist_comp = arg_pack[
            boost::graph::keywords::_distance_compare ||
            boost::value_factory<std::less<Weight> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_combine,
                closed_plus<Weight>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            boost::value_factory<closed_plus<Weight> >()
        ];
        const Weight inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<Weight>()
        ];
        const Weight zero_weight = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<Weight>()
        ];
        return floyd_warshall_initialized_all_pairs_shortest_paths(
            g1, D, dist_comp, dist_comb, inf, zero_weight
        );
    }

    template <typename Graph, typename DistanceMatrix, typename Args>
    typename boost::enable_if<
        parameter::is_argument_pack<Args>,
        bool
    >::type
    floyd_warshall_all_pairs_shortest_paths(
        const Graph& g1, DistanceMatrix& D, const Args& arg_pack
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
            g1,
            edge_weight
        );
        typedef typename boost::property_traits<WeightMap>::value_type Weight;
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_compare,
                std::less<Weight>
            >::type
        >::type dist_comp = arg_pack[
            boost::graph::keywords::_distance_compare ||
            boost::value_factory<std::less<Weight> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_combine,
                closed_plus<Weight>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            boost::value_factory<closed_plus<Weight> >()
        ];
        const Weight inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<Weight>()
        ];
        const Weight zero_weight = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<Weight>()
        ];
        return floyd_warshall_all_pairs_shortest_paths(
            g1, D, e_w_map, dist_comp, dist_comb, inf, zero_weight
        );
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <typename VertexListGraph, typename DistanceMatrix>
    inline bool floyd_warshall_initialized_all_pairs_shortest_paths(
        const VertexListGraph& g, DistanceMatrix& d
    )
    {
        return floyd_warshall_initialized_all_pairs_shortest_paths(
            g, d, parameter::compose()
        );
    }

    template <typename VertexAndEdgeListGraph, typename DistanceMatrix>
    inline bool floyd_warshall_all_pairs_shortest_paths(
        const VertexAndEdgeListGraph& g, DistanceMatrix& d
    )
    {
        return floyd_warshall_all_pairs_shortest_paths(
            g, d, parameter::compose()
        );
    }
}} // namespace boost::graph

#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename DistanceMatrix, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline typename boost::enable_if< \
        parameter::are_tagged_arguments< \
            TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
        >, \
        bool \
    >::type \
    name( \
        const Graph& g, DistanceMatrix& d, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta) \
    ) \
    { \
        return name( \
            g, d, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 6, BOOST_GRAPH_PP_FUNCTION_OVERLOAD,
    floyd_warshall_initialized_all_pairs_shortest_paths
)
BOOST_PP_REPEAT_FROM_TO(
    1, 6, BOOST_GRAPH_PP_FUNCTION_OVERLOAD,
    floyd_warshall_all_pairs_shortest_paths
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

    using ::boost::graph::floyd_warshall_initialized_all_pairs_shortest_paths;
    using ::boost::graph::floyd_warshall_all_pairs_shortest_paths;
} // namespace boost

namespace boost { namespace detail {

    template <class VertexListGraph, class DistanceMatrix, 
      class WeightMap, class P, class T, class R>
    bool floyd_warshall_init_dispatch(const VertexListGraph& g, 
      DistanceMatrix& d, WeightMap /*w*/, 
      const bgl_named_params<P, T, R>& params)
    {
      typedef typename property_traits<WeightMap>::value_type WM;
      WM inf =
        choose_param(get_param(params, distance_inf_t()), 
          std::numeric_limits<WM>::max BOOST_PREVENT_MACRO_SUBSTITUTION());
    
      return floyd_warshall_initialized_all_pairs_shortest_paths(g, d,
        choose_param(get_param(params, distance_compare_t()), 
          std::less<WM>()),
        choose_param(get_param(params, distance_combine_t()), 
          closed_plus<WM>(inf)),
        inf,
        choose_param(get_param(params, distance_zero_t()), 
          WM()));
    }

    template <class VertexAndEdgeListGraph, class DistanceMatrix, 
      class WeightMap, class P, class T, class R>
    bool floyd_warshall_noninit_dispatch(const VertexAndEdgeListGraph& g, 
      DistanceMatrix& d, WeightMap w, 
      const bgl_named_params<P, T, R>& params)
    {
      typedef typename property_traits<WeightMap>::value_type WM;
    
      WM inf =
        choose_param(get_param(params, distance_inf_t()), 
          std::numeric_limits<WM>::max BOOST_PREVENT_MACRO_SUBSTITUTION());
      return floyd_warshall_all_pairs_shortest_paths(g, d, w,
        choose_param(get_param(params, distance_compare_t()), 
          std::less<WM>()),
        choose_param(get_param(params, distance_combine_t()), 
          closed_plus<WM>(inf)),
        inf,
        choose_param(get_param(params, distance_zero_t()), 
          WM()));
    }
}} // namespace boost::detail

namespace boost {

  template <class VertexListGraph, class DistanceMatrix, class P, 
    class T, class R>
  bool floyd_warshall_initialized_all_pairs_shortest_paths(
    const VertexListGraph& g, DistanceMatrix& d, 
    const bgl_named_params<P, T, R>& params)
  {
    return boost::detail::floyd_warshall_init_dispatch(g, d, 
      choose_const_pmap(get_param(params, edge_weight), g, edge_weight), 
      params);
  }

  template <class VertexAndEdgeListGraph, class DistanceMatrix, 
    class P, class T, class R>
  bool floyd_warshall_all_pairs_shortest_paths(
    const VertexAndEdgeListGraph& g, DistanceMatrix& d, 
    const bgl_named_params<P, T, R>& params)
  {
    return boost::detail::floyd_warshall_noninit_dispatch(g, d, 
      choose_const_pmap(get_param(params, edge_weight), g, edge_weight), 
      params);
  }
} // namespace boost

#endif  // include guard

