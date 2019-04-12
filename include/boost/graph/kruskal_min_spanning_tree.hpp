//
//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
#ifndef BOOST_GRAPH_MST_KRUSKAL_HPP
#define BOOST_GRAPH_MST_KRUSKAL_HPP

/*
 *Minimum Spanning Tree 
 *         Kruskal Algorithm
 *
 *Requirement:
 *      undirected graph
 */

#include <vector>
#include <queue>
#include <functional>

#include <boost/property_map/property_map.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/concept/assert.hpp>

// Kruskal's algorithm for Minimum Spanning Tree
//
// This is a greedy algorithm to calculate the Minimum Spanning Tree
// for an undirected graph with weighted edges. The output will be a
// set of edges.
//

namespace boost { namespace detail {

    template <class Graph, class OutputIterator, 
              class Rank, class Parent, class Weight>
    void
    kruskal_mst_impl(const Graph& G, 
                     OutputIterator spanning_tree_edges, 
                     Rank rank, Parent parent, Weight weight)
    {
      if (num_vertices(G) == 0) return; // Nothing to do in this case
      typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
      typedef typename graph_traits<Graph>::edge_descriptor Edge;
      BOOST_CONCEPT_ASSERT(( VertexListGraphConcept<Graph> ));
      BOOST_CONCEPT_ASSERT(( EdgeListGraphConcept<Graph> ));
      BOOST_CONCEPT_ASSERT(( OutputIteratorConcept<OutputIterator, Edge> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<Rank, Vertex> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<Parent, Vertex> ));
      BOOST_CONCEPT_ASSERT(( ReadablePropertyMapConcept<Weight, Edge> ));
      typedef typename property_traits<Weight>::value_type W_value;
      typedef typename property_traits<Rank>::value_type R_value;
      typedef typename property_traits<Parent>::value_type P_value;
      BOOST_CONCEPT_ASSERT(( ComparableConcept<W_value> ));
      BOOST_CONCEPT_ASSERT(( ConvertibleConcept<P_value, Vertex> ));
      BOOST_CONCEPT_ASSERT(( IntegerConcept<R_value> ));

      disjoint_sets<Rank, Parent>  dset(rank, parent);

      typename graph_traits<Graph>::vertex_iterator ui, uiend;
      for (boost::tie(ui, uiend) = vertices(G); ui != uiend; ++ui)
        dset.make_set(*ui);

      typedef indirect_cmp<Weight, std::greater<W_value> > weight_greater;
      weight_greater wl(weight);
      std::priority_queue<Edge, std::vector<Edge>, weight_greater> Q(wl);
      /*push all edge into Q*/
      typename graph_traits<Graph>::edge_iterator ei, eiend;
      for (boost::tie(ei, eiend) = edges(G); ei != eiend; ++ei) 
        Q.push(*ei);

      while (! Q.empty()) {
        Edge e = Q.top();
        Q.pop();
        Vertex u = dset.find_set(source(e, G));
        Vertex v = dset.find_set(target(e, G));
        if ( u != v ) {
          *spanning_tree_edges++ = e;
          dset.link(u, v);
        }
      }
    }
}} // namespace boost::detail 

#include <boost/graph/named_function_params.hpp>

    // Named parameter variants
#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)

#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/preprocessor.hpp>

namespace boost { namespace graph {

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    BOOST_PARAMETER_FUNCTION(
        (bool), kruskal_minimum_spanning_tree, ::boost::graph::keywords::tag,
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
                (weight_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_edge_property_map_of_graph
                        >
                    )
                  , boost::detail::edge_or_dummy_property_map(
                        graph,
                        edge_weight
                    )
                )
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
                (predecessor_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_to_vertex_map_of_graph
                        >
                    )
                  , make_shared_array_property_map(
                        num_vertices(graph),
                        boost::detail::get_null_vertex(graph),
                        vertex_index_map
                    )
                )
            )
        )
        (optional
            (rank_map
              , *(
                    boost::detail::argument_with_graph_predicate<
                        boost::detail::is_vertex_to_integer_map_of_graph
                    >
                )
              , make_shared_array_property_map(
                    num_vertices(graph),
                    num_vertices(graph) - num_vertices(graph),
                    vertex_index_map
                )
            )
        )
    )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    BOOST_PARAMETER_FUNCTION(
        (bool), kruskal_minimum_spanning_tree, ::boost::graph::keywords::tag,
        (required
            (graph, *)
            (result, *)
        )
        (optional
            (weight_map
              , *
              , boost::detail::edge_or_dummy_property_map(graph, edge_weight)
            )
            (vertex_index_map
              , *
              , boost::detail::vertex_or_dummy_property_map(
                    graph,
                    vertex_index
                )
            )
            (predecessor_map
              , *
              , make_shared_array_property_map(
                    num_vertices(graph),
                    boost::detail::get_null_vertex(graph),
                    vertex_index_map
                )
            )
            (rank_map
              , *
              , make_shared_array_property_map(
                    num_vertices(graph),
                    num_vertices(graph) - num_vertices(graph),
                    vertex_index_map
                )
            )
        )
    )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
    {
        if (0 < num_vertices(graph))
            boost::detail::kruskal_mst_impl(
                graph, result, rank_map, predecessor_map, weight_map
            );
        return true;
    }
}} // namespace boost::graph

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)

#include <boost/parameter/is_argument_pack.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename OutputIterator, typename Args>
    void kruskal_minimum_spanning_tree(
        const Graph& g, OutputIterator spanning_tree_edges, const Args& args,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        if (num_vertices(g) == 0) return; // Nothing to do in this case
        typedef typename graph_traits<Graph>::vertices_size_type VIndex;
        const VIndex zero_index = VIndex();
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::rank_map,
            VIndex
        > v_r_map_gen(zero_index);
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::rank_map,
            VIndex
        >::map_type v_r_map = v_r_map_gen(g, args);
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::predecessor_map,
            Vertex
        > v_p_map_gen(graph_traits<Graph>::null_vertex());
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::predecessor_map,
            Vertex
        >::map_type v_p_map = v_p_map_gen(g, args);
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            Graph
        >::type e_w_map = boost::detail::override_const_property(
            args,
            boost::graph::keywords::_weight_map,
            g,
            edge_weight
        );
        boost::detail::kruskal_mst_impl(
            g, spanning_tree_edges, v_r_map, v_p_map, e_w_map
        );
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename OutputIterator>
    inline void kruskal_minimum_spanning_tree(
        const Graph& g, OutputIterator spanning_tree_edges
    )
    {
        kruskal_minimum_spanning_tree(
            g, spanning_tree_edges, parameter::compose()
        );
    }
}} // namespace boost::graph

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename OutputIterator, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const Graph& g, OutputIterator spanning_tree_edges, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, mpl::true_ \
        >::type = mpl::true_() \
    ) \
    { \
        name( \
            g, spanning_tree_edges, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 5, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, kruskal_minimum_spanning_tree
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

namespace boost {

    using ::boost::graph::kruskal_minimum_spanning_tree;
} // namespace boost

namespace boost {

  template <class Graph, class OutputIterator, class P, class T, class R>
  inline void
  kruskal_minimum_spanning_tree(const Graph& g,
                                OutputIterator spanning_tree_edges, 
                                const bgl_named_params<P, T, R>& params)
  {
    typedef typename graph_traits<Graph>::vertices_size_type size_type;
    typedef typename graph_traits<Graph>::vertex_descriptor vertex_t;
    if (num_vertices(g) == 0) return; // Nothing to do in this case
    typename graph_traits<Graph>::vertices_size_type n;
    n = is_default_param(get_param(params, vertex_rank))
                                   ? num_vertices(g) : 1;
    std::vector<size_type> rank_map(n);
    n = is_default_param(get_param(params, vertex_predecessor))
                                   ? num_vertices(g) : 1;
    std::vector<vertex_t> pred_map(n);
    
    detail::kruskal_mst_impl
      (g, spanning_tree_edges, 
       choose_param
       (get_param(params, vertex_rank), 
        make_iterator_property_map
        (rank_map.begin(), 
         choose_pmap(get_param(params, vertex_index), g, vertex_index), rank_map[0])),
       choose_param
       (get_param(params, vertex_predecessor), 
        make_iterator_property_map
        (pred_map.begin(), 
         choose_const_pmap(get_param(params, vertex_index), g, vertex_index), 
         pred_map[0])),
       choose_const_pmap(get_param(params, edge_weight), g, edge_weight));
  }
    
} // namespace boost


#endif // BOOST_GRAPH_MST_KRUSKAL_HPP

