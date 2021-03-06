// Copyright 2010 The Trustees of Indiana University.

// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  Authors: Jeremiah Willcock
//           Andrew Lumsdaine

#ifndef BOOST_GRAPH_LOOP_ERASED_RANDOM_WALK_HPP
#define BOOST_GRAPH_LOOP_ERASED_RANDOM_WALK_HPP

#include <exception>

namespace boost { namespace graph {

    struct BOOST_SYMBOL_VISIBLE loop_erased_random_walk_stuck :
        public std::exception
    {
        virtual ~loop_erased_random_walk_stuck() throw()
        {
        }

        inline virtual const char* what() const throw()
        {
            return "Loop-erased random walk found a vertex with no out-edges";
        }
    };
}}

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/random.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename Gen>
    class unweighted_random_out_edge_gen
    {
        Gen& gen;

        typedef boost::graph_traits<Graph> gt;

    public:
        unweighted_random_out_edge_gen(Gen& gen) : gen(gen)
        {
        }

        typename gt::edge_descriptor
        operator()(typename gt::vertex_descriptor src, const Graph& g) const
        {
            if (out_degree(src, g) == 0)
            {
                throw loop_erased_random_walk_stuck();
            }

            return boost::random_out_edge(g, src, gen);
        }
    };

    template <typename Graph, typename WeightMap, typename Gen>
    class weighted_random_out_edge_gen
    {
        WeightMap weight;
        Gen& gen;

        typedef boost::graph_traits<Graph> gt;

    public:
        weighted_random_out_edge_gen(const WeightMap& weight, Gen& gen) :
            weight(weight), gen(gen)
        {
        }

        typename gt::edge_descriptor
        operator()(typename gt::vertex_descriptor src, const Graph& g) const
        {
            if (out_degree(src, g) == 0)
            {
                throw loop_erased_random_walk_stuck();
            }

            return boost::weighted_random_out_edge(g, src, weight, gen);
        }
    };
}}

#include <boost/graph/named_function_params.hpp>
#include <boost/graph/properties.hpp>
#include <boost/assert.hpp>
#include <boost/config.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/graph/detail/traits.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#endif

namespace boost { namespace graph {

    // Do a loop-erased random walk from vertex s to any vertex colored black (or
    // actually any color other than white or gray) in the color map.  The color
    // white is for vertices that are not part of the path, while gray is for
    // those that are on the path (for cycle detection).  The vector path is used
    // for temporary storage and as the result of the algorithm; while all
    // elements of the path except the last have their colors set to gray upon
    // return.  Vertex s must start off colored white.
    //
    // Useful references:
    // http://everything2.com/title/loop-erased+random+walk
    // Wikipedia page on "Loop-Erased Random Walk"

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    BOOST_PARAMETER_FUNCTION(
        (bool), loop_erased_random_walk, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_bgl_graph>)
            )
        )
        (deduced
            (required
                (root_vertex
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_of_graph
                        >
                    )
                )
                (generator_function
                  , *(boost::detail::binary_function_graph_predicate<>)
                )
                (color_map
                  , *(
                        boost::detail::argument_with_graph_predicate<
                            boost::detail::is_vertex_color_map_of_graph
                        >
                    )
                )
                (buffer
                  , *(
                        boost::detail::argument_predicate<
                            boost::detail::has_container_typedefs
                        >
                    )
                )
            )
        )
    )
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    template <
        typename Graph, typename NextEdge, typename ColorMap, typename VertSeq
    >
    void loop_erased_random_walk(
        const Graph& graph,
        typename boost::graph_traits<Graph>::vertex_descriptor root_vertex,
        NextEdge generator_function,
        ColorMap color_map,
        VertSeq& buffer
    )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
    {
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        typedef typename boost::remove_const<
            typename boost::remove_reference<graph_type>::type
        >::type Graph;
        typedef typename boost::remove_const<
            typename boost::remove_reference<color_map_type>::type
        >::type ColorMap;
        typedef typename boost::remove_const<
            typename boost::remove_reference<buffer_type>::type
        >::type VertSeq;
#endif
        typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
        typedef typename boost::graph_traits<Graph>::edge_descriptor Edge;
        typedef typename boost::property_traits<ColorMap>::value_type color_t;
        typedef boost::color_traits<color_t> color_gen;

        Vertex s = root_vertex;
        BOOST_ASSERT((get(color_map, s) == color_gen::white()));
        buffer.clear();
        buffer.push_back(s);
        put(color_map, s, color_gen::gray());

        for (;;)
        {
            Edge e = generator_function(s, graph);
            Vertex t = target(e, graph);
            color_t t_color = get(color_map, t);

            if (t_color == color_gen::white())
            {
                buffer.push_back(t);
                put(color_map, t, color_gen::gray());
                s = t;
            }
            else if (t_color == color_gen::gray())
            {
                // Found a loop; delete from path from the first occurrence
                // of t to the end, coloring vertices white.
                typename VertSeq::iterator i = std::find(
                    buffer.begin(),
                    buffer.end(),
                    t
                );
                BOOST_ASSERT(i != buffer.end());
                ++i;

                for (typename VertSeq::iterator j = i; j != buffer.end(); ++j)
                {
                    put(color_map, *j, color_gen::white());
                }

                buffer.erase(i, buffer.end());
                s = t;
            }
            else
            {
                // Done
                buffer.push_back(t);
                break;
            }
        }

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        return true;
#endif
    }
}}

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/binding.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename Args>
    void loop_erased_random_walk(
        const Graph& graph, const Args& arg_pack,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        Vertex s = arg_pack[boost::graph::keywords::_root_vertex];
        typename boost::parameter::value_type<
            Args,
            boost::graph::keywords::tag::generator_function
        >::type next_edge = arg_pack[
            boost::graph::keywords::_generator_function
        ];
        typename boost::parameter::value_type<
            Args,
            boost::graph::keywords::tag::color_map
        >::type v_c_map = arg_pack[boost::graph::keywords::_color_map];
        typename boost::parameter::binding<
            Args,
            boost::graph::keywords::tag::buffer
        >::type path = arg_pack[
            boost::graph::keywords::_buffer
        ];
        loop_erased_random_walk(graph, s, next_edge, v_c_map, path);
    }
}}

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <
        typename Graph, typename TA0, typename TA1, typename TA2, typename TA3
    >
    void loop_erased_random_walk(
        const Graph& graph,
        const TA0& ta0, const TA1& ta1, const TA2& ta2, const TA3& ta3,
        typename boost::enable_if<
            parameter::are_tagged_arguments<TA0, TA1, TA2, TA3>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        loop_erased_random_walk(
            graph, parameter::compose(ta0, ta1, ta2, ta3)
        );
    }
}}

#endif  // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)

namespace boost {

    using ::boost::graph::loop_erased_random_walk_stuck;
    using ::boost::graph::unweighted_random_out_edge_gen;
    using ::boost::graph::weighted_random_out_edge_gen;
    using ::boost::graph::loop_erased_random_walk;
}

#endif // BOOST_GRAPH_LOOP_ERASED_RANDOM_WALK_HPP
