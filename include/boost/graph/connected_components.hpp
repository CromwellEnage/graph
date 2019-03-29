//
//=======================================================================
// Copyright 1997-2001 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
#ifndef BOOST_GRAPH_CONNECTED_COMPONENTS_HPP
#define BOOST_GRAPH_CONNECTED_COMPONENTS_HPP

#include <boost/config.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/graph/detail/mpi_include.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/type_traits/is_same.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#include <boost/static_assert.hpp>
#include <boost/concept/assert.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
#include <boost/parameter/preprocessor.hpp>
#include <boost/mpl/has_key.hpp>

#if !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
#include <boost/mpl/eval_if.hpp>
#endif
#endif

namespace boost { namespace detail {

    // This visitor is used both in the connected_components algorithm
    // and in the kosaraju strong components algorithm during the
    // second DFS traversal.
    template <typename ComponentsMap>
    class components_recorder : public dfs_visitor<>
    {
        typedef typename property_traits<ComponentsMap>::value_type comp_type;

    public:
        components_recorder(ComponentsMap c, comp_type& c_count)
        : m_component(c), m_count(c_count) {}

        template <typename Vertex, typename Graph>
        void start_vertex(Vertex, Graph&)
        {
            if (this->m_count == (std::numeric_limits<comp_type>::max)())
                this->m_count = 0; // start counting components at zero
            else
                ++this->m_count;
        }

        template <typename Vertex, typename Graph>
        void discover_vertex(Vertex u, Graph&)
        {
            put(this->m_component, u, this->m_count);
        }

    protected:
        ComponentsMap m_component;
        comp_type& m_count;
    };
}} // namespace boost::detail

#if defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)

namespace boost { namespace graph {

    // This function computes the connected components of an undirected graph
    // using a single application of depth first search.
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    BOOST_PARAMETER_FUNCTION(
        (
            boost::lazy_enable_if<
                typename mpl::has_key<
                    Args,
                    boost::graph::keywords::tag::component_map
                >::type,
                boost::detail::property_map_value<
                    Args,
                    boost::graph::keywords::tag::component_map
                >
            >
        ), connected_components, ::boost::graph::keywords::tag,
        (required
            (graph
              , *(boost::detail::argument_predicate<is_vertex_list_graph>)
            )
            (component_map
              , *(
                    boost::detail::argument_with_graph_predicate<
                        boost::detail::is_vertex_property_map_of_graph
                    >
                )
            )
        )
        (deduced
            (optional
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
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    BOOST_PARAMETER_FUNCTION(
        (
            boost::lazy_enable_if<
                typename mpl::eval_if<
                    boost::detail::is_bgl_named_param_argument<
                        Args,
                        boost::graph::keywords::tag::vertex_index_map
                    >,
                    mpl::false_,
                    mpl::has_key<
                        Args,
                        boost::graph::keywords::tag::component_map
                    >
                >::type,
                boost::detail::property_map_value<
                    Args,
                    boost::graph::keywords::tag::component_map
                >
            >
        ), connected_components, ::boost::graph::keywords::tag,
        (required
            (graph, *)
            (component_map, *)
        )
        (optional
            (vertex_index_map
              , *
              , boost::detail::vertex_or_dummy_property_map(
                    graph,
                    vertex_index
                )
            )
            (color_map
              , *
              , make_shared_array_property_map(
                    num_vertices(graph),
                    white_color,
                    vertex_index_map
                )
            )
        )
    )
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS
    {
        if (num_vertices(graph) == 0) return 0;

        typedef typename boost::remove_const<
            typename boost::remove_reference<graph_type>::type
        >::type Graph;
        typedef typename boost::remove_const<
            typename boost::remove_reference<component_map_type>::type
        >::type ComponentMap;
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        BOOST_CONCEPT_ASSERT((
            WritablePropertyMapConcept<ComponentMap, Vertex>
        ));
        // typedef typename graph_traits<Graph>::directed_category directed;
        // BOOST_STATIC_ASSERT((
        //     boost::is_same<directed, undirected_tag>::value
        // ));

        typedef typename property_traits<ComponentMap>::value_type comp_type;
        // c_count initialized to "nil" (with nil represented by (max)())
        comp_type c_count((std::numeric_limits<comp_type>::max)());
        boost::detail::components_recorder<
            ComponentMap
        > vis(component_map, c_count);
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
        depth_first_search(graph, vis, vertex_index_map, color_map);
#else
        depth_first_search(graph, vis, color_map);
#endif
        return c_count + 1;
    }
}} // namespace boost::graph

#else   // !defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)

namespace boost { namespace graph {

  // This function computes the connected components of an undirected graph
  // using a single application of depth first search.
  template <class Graph, class ComponentMap>
  inline typename property_traits<ComponentMap>::value_type
  connected_components(const Graph& g, ComponentMap c,
                       typename boost::disable_if<
                         parameter::is_argument_pack<ComponentMap>,
                         mpl::true_
                       >::type = mpl::true_()
                       BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph, vertex_list_graph_tag))
  {
    if (num_vertices(g) == 0) return 0;

    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    BOOST_CONCEPT_ASSERT(( WritablePropertyMapConcept<ComponentMap, Vertex> ));
    // typedef typename boost::graph_traits<Graph>::directed_category directed;
    // BOOST_STATIC_ASSERT((boost::is_same<directed, undirected_tag>::value));

    typedef typename property_traits<ComponentMap>::value_type comp_type;
    // c_count initialized to "nil" (with nil represented by (max)())
    comp_type c_count((std::numeric_limits<comp_type>::max)());
    boost::detail::components_recorder<ComponentMap> vis(c, c_count);
    depth_first_search(g, boost::graph::keywords::_visitor = vis);
    return c_count + 1;
  }
}} // namespace boost::graph

namespace boost { namespace graph { namespace detail {

    template <typename Graph, typename ComponentMap>
    struct connected_components_impl
    {
        typedef typename property_traits<
            ComponentMap
        >::value_type result_type;
        typedef result_type type;

        template <typename ArgPack>
        inline result_type operator()(
            const Graph& g, ComponentMap c, const ArgPack& arg_pack
        ) const
        {
            // c_count initialized to "nil" (with nil represented by (max)())
            result_type c_count((std::numeric_limits<result_type>::max)());
            boost::detail::components_recorder<
                typename boost::remove_const<
                    typename boost::remove_reference<ComponentMap>::type
                >::type
            > vis(c, c_count);
            depth_first_search(
                g,
                vis,
                arg_pack[
                    boost::graph::keywords::_color_map |
                    make_shared_array_property_map(
                        num_vertices(g),
                        white_color,
                        arg_pack[
                            boost::graph::keywords::_vertex_index_map |
                            boost::detail::vertex_or_dummy_property_map(
                                g,
                                vertex_index
                            )
                        ]
                    )
                ]
            )
            return c_count + 1;
        }
    };
}}} // namespace boost::graph::detail

namespace boost { namespace graph {

    // Boost.Parameter-enabled variant
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(connected_components, 2, 4)
}} // namespace boost::graph

#endif  // BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS

namespace boost {

    using ::boost::graph::connected_components;

  template <class Graph, class ComponentMap, class P, class T, class R>
  inline typename property_traits<ComponentMap>::value_type
  connected_components(const Graph& g, ComponentMap c, 
                       const bgl_named_params<P, T, R>& params
                       BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph, vertex_list_graph_tag))
  {
    if (num_vertices(g) == 0) return 0;

    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    BOOST_CONCEPT_ASSERT(( WritablePropertyMapConcept<ComponentMap, Vertex> ));
    typedef typename boost::graph_traits<Graph>::directed_category directed;
    BOOST_STATIC_ASSERT((boost::is_same<directed, undirected_tag>::value));

    typedef typename property_traits<ComponentMap>::value_type comp_type;
    // c_count initialized to "nil" (with nil represented by (max)())
    comp_type c_count((std::numeric_limits<comp_type>::max)());
    detail::components_recorder<ComponentMap> vis(c, c_count);
    depth_first_search(g, params.visitor(vis));
    return c_count + 1;
  }
} // namespace boost

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/connected_components.hpp>)

#endif // BOOST_GRAPH_CONNECTED_COMPONENTS_HPP
