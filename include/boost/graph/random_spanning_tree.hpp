// Copyright 2010 The Trustees of Indiana University.

// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  Authors: Jeremiah Willcock
//           Andrew Lumsdaine

#ifndef BOOST_GRAPH_RANDOM_SPANNING_TREE_HPP
#define BOOST_GRAPH_RANDOM_SPANNING_TREE_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/loop_erased_random_walk.hpp>
#include <boost/graph/iteration_macros.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/next_prior.hpp>
#include <boost/config.hpp>
#include <boost/assert.hpp>
#include <vector>

namespace boost { namespace detail {

    // Use Wilson's algorithm (based on loop-free random walks) to generate a
    // random spanning tree.  The distribution of edges used is controlled by
    // the next_edge() function, so this version allows either weighted or
    // unweighted selection of trees.
    // Algorithm is from http://en.wikipedia.org/wiki/Uniform_spanning_tree
    template <
        typename Graph, typename PredMap, typename ColorMap, typename NextEdge
    >
    void random_spanning_tree_internal(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        PredMap pred, ColorMap color, NextEdge next_edge
    )
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

        // g must also be undirected (or symmetric) and connected
        BOOST_ASSERT(1 <= num_vertices(g));

        typedef color_traits<
            typename property_traits<ColorMap>::value_type
        > color_gen;

        BGL_FORALL_VERTICES_T(v, g, Graph)
            put(color, v, color_gen::white());

        std::vector<Vertex> path;

        put(color, s, color_gen::black());
        put(pred, s, graph_traits<Graph>::null_vertex());

        typedef typename std::vector<Vertex>::const_reverse_iterator VRItr;

        BGL_FORALL_VERTICES_T(v, g, Graph)
        {
            if (get(color, v) != color_gen::white()) continue;
            loop_erased_random_walk(g, v, next_edge, color, path);

            for (
                VRItr i = path.rbegin();
                boost::next(i) != static_cast<VRItr>(path.rend());
                ++i
            )
            {
                VRItr j = i;
                ++j;
                BOOST_ASSERT(get(color, *j) == color_gen::gray());
                put(color, *j, color_gen::black());
                put(pred, *j, *i);
            }
        }
    }
}}

#include <boost/graph/random.hpp>
#include <boost/graph/named_function_params.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

#include <boost/graph/detail/traits.hpp>
#include <boost/parameter/preprocessor.hpp>

namespace boost { namespace graph {

    // Compute a uniformly-distributed spanning tree on a graph.
    // Use Wilson's algorithm:
    // @inproceedings{wilson96generating,
    //    author = {Wilson, David Bruce},
    //    title = {Generating random spanning trees more quickly than the cover time},
    //    booktitle = {STOC '96: Proceedings of the twenty-eighth annual ACM symposium on Theory of computing},
    //    year = {1996},
    //    isbn = {0-89791-785-5},
    //    pages = {296--303},
    //    location = {Philadelphia, Pennsylvania, United States},
    //    doi = {http://doi.acm.org/10.1145/237814.237880},
    //    publisher = {ACM},
    //    address = {New York, NY, USA},
    //  }
    //
    BOOST_PARAMETER_FUNCTION(
        (bool), random_spanning_tree, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_bgl_graph>)
            )
        )
        (deduced
            (required
                (generator_function
                  , *(boost::detail::generator_predicate)
                )
                (predecessor_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_to_vertex_map_of_graph
                        >
                    )
                )
            )
            (optional
                (root_vertex
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_of_graph
                        >
                    )
                  , boost::detail::get_default_starting_vertex(graph)
                )
                (weight_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_edge_property_map_of_graph
                        >
                    )
                  , boost::static_property_map<double>(1.)
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
                (color_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_color_map_of_graph
                        >
                    )
                  , make_shared_array_property_map(
                        num_vertices(graph),
                        white_color,
                        vertex_index_map
                    )
                )
            )
        )
    )
    {
        weighted_random_out_edge_gen<
            typename boost::remove_const<
                typename boost::remove_reference<graph_type>::type
            >::type,
            typename boost::remove_const<
                typename boost::remove_reference<weight_map_type>::type
            >::type,
            typename boost::remove_const<
                typename boost::remove_reference<
                    generator_function_type
                >::type
            >::type
        > random_oe(weight_map, generator_function);
        boost::detail::random_spanning_tree_internal(
            graph, root_vertex, predecessor_map, color_map, random_oe
        );
        return true;
    }
}}

#include <boost/graph/detail/static_property_map_gen.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/parameter/value_type.hpp>

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost { namespace graph {

  template <typename Graph, typename Gen, typename PredMap, typename ColorMap>
  void random_spanning_tree(const Graph& g, Gen& gen, typename graph_traits<Graph>::vertex_descriptor root,
                            PredMap pred, static_property_map<double>, ColorMap color) {
    unweighted_random_out_edge_gen<Graph, Gen> random_oe(gen);
    boost::detail::random_spanning_tree_internal(g, root, pred, color, random_oe);
  }

  // Compute a weight-distributed spanning tree on a graph.
  template <typename Graph, typename Gen, typename PredMap, typename WeightMap, typename ColorMap>
  void random_spanning_tree(const Graph& g, Gen& gen, typename graph_traits<Graph>::vertex_descriptor root,
                            PredMap pred, WeightMap weight, ColorMap color) {
    weighted_random_out_edge_gen<Graph, WeightMap, Gen> random_oe(weight, gen);
    boost::detail::random_spanning_tree_internal(g, root, pred, color, random_oe);
  }
}}

#include <boost/graph/detail/static_property_map_gen.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/functional/value_factory.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename Gen, typename Args>
    void random_spanning_tree(
        const Graph& g, Gen& gen, const Args& arg_pack,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        Vertex start_vertex = arg_pack[
            boost::graph::keywords::_root_vertex ||
            boost::detail::get_default_starting_vertex_t<Graph>(g)
        ];
        typename boost::parameter::binding<
            Args,
            boost::graph::keywords::tag::predecessor_map
        >::type v_p_map = arg_pack[boost::graph::keywords::_predecessor_map];
        typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::weight_map,
                static_property_map<double>
            >::type
        >::type e_w_map = arg_pack[
            boost::graph::keywords::_weight_map ||
            boost::detail::static_property_map_generator<double>(1.)
        ];
        typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::color_map,
            boost::default_color_type
        >::map_type v_c_map = boost::detail::make_color_map_from_arg_pack(
            g,
            arg_pack
        );
        random_spanning_tree(g, gen, start_vertex, v_p_map, e_w_map, v_c_map);
    }
}}

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/compose.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename Gen, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const Graph& g, Gen& gen, const TA& ta \
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
            g, gen, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 6, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, random_spanning_tree
)
}}

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS

namespace boost {

    using ::boost::graph::random_spanning_tree;

    template <
        typename Graph, typename Gen, typename P, typename T, typename R
    >
    void random_spanning_tree(
        const Graph& g, Gen& gen, const bgl_named_params<P, T, R>& params
    )
    {
        typedef bgl_named_params<P, T, R> params_type;
        BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        Vertex start_vertex = arg_pack[
            boost::graph::keywords::_root_vertex ||
            boost::detail::get_default_starting_vertex_t<Graph>(g)
        ];
        typename boost::parameter::binding<
            arg_pack_type,
            boost::graph::keywords::tag::predecessor_map
        >::type v_p_map = arg_pack[boost::graph::keywords::_predecessor_map];
        typename boost::remove_const<
            typename parameter::value_type<
                arg_pack_type,
                boost::graph::keywords::tag::weight_map,
                static_property_map<double>
            >::type
        >::type e_w_map = arg_pack[
            boost::graph::keywords::_weight_map ||
            boost::detail::static_property_map_generator<double>(1.)
        ];
        typename boost::detail::map_maker<
            Graph,
            arg_pack_type,
            boost::graph::keywords::tag::color_map,
            boost::default_color_type
        >::map_type v_c_map = boost::detail::make_color_map_from_arg_pack(
            g,
            arg_pack
        );
        random_spanning_tree(g, gen, start_vertex, v_p_map, e_w_map, v_c_map);
    }
}

#include <boost/graph/iteration_macros_undef.hpp>

#endif // BOOST_GRAPH_RANDOM_SPANNING_TREE_HPP
