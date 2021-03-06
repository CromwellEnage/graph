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

#ifndef BOOST_GRAPH_STRONG_COMPONENTS_HPP
#define BOOST_GRAPH_STRONG_COMPONENTS_HPP

#include <stack>
#include <boost/config.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/overloading.hpp>
#include <boost/graph/detail/mpi_include.hpp>
#include <boost/graph/detail/traits.hpp>
#include <boost/type_traits/conversion_traits.hpp>
#include <boost/concept/assert.hpp>
#include <boost/static_assert.hpp>

#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
#include <boost/parameter/preprocessor.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/mpl/has_key.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>
#else
#include <boost/parameter/compose.hpp>
#endif

namespace boost { namespace detail {

    //========================================================================
    // This is Tarjan's algorithm for strongly connected components
    // from his paper "Depth first search and linear graph algorithms".
    // It calculates the components in a single application of DFS.
    // We implement the algorithm as a dfs-visitor.

    template <typename ComponentMap, typename RootMap, typename DiscoverTime,
              typename Stack>
    class tarjan_scc_visitor : public boost::graph::dfs_visitor<>
    {
        typedef typename property_traits<ComponentMap>::value_type comp_type;
        typedef typename property_traits<DiscoverTime>::value_type time_type;

    public:
        inline tarjan_scc_visitor(
            ComponentMap comp_map, RootMap r, DiscoverTime d,
            comp_type& c_, Stack& s_
        ) : boost::graph::dfs_visitor<>(), c(c_), comp(comp_map), root(r),
            discover_time(d), dfs_time(time_type()), s(s_)
        {
        }

        template <typename Graph>
        inline void discover_vertex(
            typename graph_traits<Graph>::vertex_descriptor v,
            const Graph&
        )
        {
            put(this->root, v, v);
            put(this->comp, v, (std::numeric_limits<comp_type>::max)());
            put(this->discover_time, v, this->dfs_time++);
            this->s.push(v);
        }

        template <typename Graph>
        void finish_vertex(
            typename graph_traits<Graph>::vertex_descriptor v,
            const Graph& g
        )
        {
            typename graph_traits<Graph>::vertex_descriptor w;
            typename graph_traits<Graph>::out_edge_iterator ei, ei_end;

            for (boost::tie(ei, ei_end) = out_edges(v, g); ei != ei_end; ++ei)
            {
                w = target(*ei, g);
                if (
                    get(this->comp, w) == (
                        std::numeric_limits<comp_type>::max
                    )()
                )
                {
                    put(
                        this->root,
                        v,
                        this->min_discover_time(
                            get(this->root, v),
                            get(this->root, w)
                        )
                    );
                }
            }

            if (get(this->root, v) == v)
            {
                do
                {
	                w = this->s.top(); this->s.pop();
	                put(this->comp, w, this->c);
                    put(this->root, w, v);
                } while (w != v);
                ++c;
            }
        }

    private:
        template <typename Vertex>
        inline Vertex min_discover_time(Vertex u, Vertex v)
        {
            return (
                get(this->discover_time, u) < get(this->discover_time, v)
            ) ? u : v;
        }

        comp_type& c;
        ComponentMap comp;
        RootMap root;
        DiscoverTime discover_time;
        time_type dfs_time;
        Stack& s;
    };

    template <typename Graph, typename ComponentMap, typename RootMap,
              typename DiscoverTime, typename ColorMap>
    typename property_traits<ComponentMap>::value_type
    strong_components_impl(
        const Graph& g,    // Input
        ComponentMap comp, // Output
        // Internal record keeping
        RootMap root,
        DiscoverTime discover_time,
        ColorMap color
    )
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        BOOST_CONCEPT_ASSERT((
            ReadWritePropertyMapConcept<ComponentMap, Vertex>
        ));
        BOOST_CONCEPT_ASSERT((
            ReadWritePropertyMapConcept<RootMap, Vertex>
        ));
        typedef typename property_traits<RootMap>::value_type RootV;
        BOOST_CONCEPT_ASSERT(( ConvertibleConcept<RootV, Vertex> ));
        BOOST_CONCEPT_ASSERT((
            ReadWritePropertyMapConcept<DiscoverTime, Vertex>
        ));

        typename property_traits<ComponentMap>::value_type total = 0;

        std::stack<Vertex> s;
        detail::tarjan_scc_visitor<
            ComponentMap, RootMap, DiscoverTime, std::stack<Vertex>
        > vis(comp, root, discover_time, total, s);
        depth_first_search(g, vis, color);
        return total;
    }

#if ( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
    template <class Graph, class ComponentMap, class RootMap,
              class DiscoverTime, class P, class T, class R>
    typename property_traits<ComponentMap>::value_type
    strong_components_impl
      (const Graph& g,    // Input
       ComponentMap comp, // Output
       // Internal record keeping
       RootMap root, 
       DiscoverTime discover_time,
       const bgl_named_params<P, T, R>& params)
    {
      typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<ComponentMap, Vertex> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<RootMap, Vertex> ));
      typedef typename property_traits<RootMap>::value_type RootV;
      BOOST_CONCEPT_ASSERT(( ConvertibleConcept<RootV, Vertex> ));
      BOOST_CONCEPT_ASSERT(( ReadWritePropertyMapConcept<DiscoverTime, Vertex> ));

      typename property_traits<ComponentMap>::value_type total = 0;

      std::stack<Vertex> s;
      detail::tarjan_scc_visitor<ComponentMap, RootMap, DiscoverTime, 
        std::stack<Vertex> > 
        vis(comp, root, discover_time, total, s);
      depth_first_search(g, params.visitor(vis));
      return total;
    }

    //-------------------------------------------------------------------------
    // The dispatch functions handle the defaults for the rank and discover
    // time property maps.
    // dispatch with class specialization to avoid VC++ bug

    template <class DiscoverTimeMap>
    struct strong_comp_dispatch2 {
      template <class Graph, class ComponentMap, class RootMap, class P, class T, class R>
      inline static typename property_traits<ComponentMap>::value_type
      apply(const Graph& g,
            ComponentMap comp,
            RootMap r_map,
            const bgl_named_params<P, T, R>& params,
            DiscoverTimeMap time_map)
      {
        return strong_components_impl(g, comp, r_map, time_map, params);
      }
    };


    template <>
    struct strong_comp_dispatch2<param_not_found> {
      template <class Graph, class ComponentMap, class RootMap,
                class P, class T, class R>
      inline static typename property_traits<ComponentMap>::value_type
      apply(const Graph& g,
            ComponentMap comp,
            RootMap r_map,
            const bgl_named_params<P, T, R>& params,
            param_not_found)
      {
        typedef typename graph_traits<Graph>::vertices_size_type size_type;
        size_type       n = num_vertices(g) > 0 ? num_vertices(g) : 1;
        std::vector<size_type> time_vec(n);
        return strong_components_impl
          (g, comp, r_map,
           make_iterator_property_map(time_vec.begin(), choose_const_pmap
                                      (get_param(params, vertex_index),
                                       g, vertex_index), time_vec[0]),
           params);
      }
    };

    template <class Graph, class ComponentMap, class RootMap,
              class P, class T, class R, class DiscoverTimeMap>
    inline typename property_traits<ComponentMap>::value_type
    scc_helper2(const Graph& g,
                ComponentMap comp,
                RootMap r_map,
                const bgl_named_params<P, T, R>& params,
                DiscoverTimeMap time_map)
    {
      return strong_comp_dispatch2<DiscoverTimeMap>::apply(g, comp, r_map, params, time_map);
    }

    template <class RootMap>
    struct strong_comp_dispatch1 {

      template <class Graph, class ComponentMap, class P, class T, class R>
      inline static typename property_traits<ComponentMap>::value_type
      apply(const Graph& g,
            ComponentMap comp,
            const bgl_named_params<P, T, R>& params,
            RootMap r_map)
      {
        return scc_helper2(g, comp, r_map, params, get_param(params, vertex_discover_time));
      }
    };
    template <>
    struct strong_comp_dispatch1<param_not_found> {

      template <class Graph, class ComponentMap, 
                class P, class T, class R>
      inline static typename property_traits<ComponentMap>::value_type
      apply(const Graph& g,
            ComponentMap comp,
            const bgl_named_params<P, T, R>& params,
            param_not_found)
      {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typename std::vector<Vertex>::size_type
          n = num_vertices(g) > 0 ? num_vertices(g) : 1;
        std::vector<Vertex> root_vec(n);
        return scc_helper2
          (g, comp, 
           make_iterator_property_map(root_vec.begin(), choose_const_pmap
                                      (get_param(params, vertex_index),
                                       g, vertex_index), root_vec[0]),
           params, 
           get_param(params, vertex_discover_time));
      }
    };

    template <class Graph, class ComponentMap, class RootMap,
              class P, class T, class R>
    inline typename property_traits<ComponentMap>::value_type
    scc_helper1(const Graph& g,
               ComponentMap comp,
               const bgl_named_params<P, T, R>& params,
               RootMap r_map)
    {
      return detail::strong_comp_dispatch1<RootMap>::apply(g, comp, params,
                                                           r_map);
    }
#endif  // MSVC-14.0 w/64-bit addressing
}} // namespace boost::detail

#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )

namespace boost { namespace graph {

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
        ), strong_components, ::boost::graph::keywords::tag,
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
            (root_map
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
            (discover_time_map
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
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
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
        ), strong_components, ::boost::graph::keywords::tag,
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
            (root_map
              , *
              , make_shared_array_property_map(
                    num_vertices(graph),
                    boost::detail::get_null_vertex(graph),
                    vertex_index_map
                )
            )
            (discover_time_map
              , *
              , make_shared_array_property_map(
                    num_vertices(graph),
                    num_vertices(graph) - num_vertices(graph),
                    vertex_index_map
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
        typedef typename graph_traits<
            typename boost::remove_const<
                typename boost::remove_reference<graph_type>::type
            >::type
        >::directed_category DirCat;
        BOOST_STATIC_ASSERT((
            boost::is_convertible<DirCat*, directed_tag*>::value
        ));
        return boost::detail::strong_components_impl(
            graph, component_map, root_map, discover_time_map, color_map
        );
    }
}} // namespace boost::graph

#else   // MSVC-14.0 w/64-bit addressing

namespace boost { namespace graph { namespace detail {

    template <typename Graph, typename ComponentMap>
    struct strong_components_impl
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
            typedef typename graph_traits<Graph>::directed_category DirCat;
            BOOST_STATIC_ASSERT((
                boost::is_convertible<DirCat*, directed_tag*>::value
            ));
            typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
            boost::detail::make_property_map_from_arg_pack_gen<
                boost::graph::keywords::tag::root_map,
                Vertex
            > rt_map_gen(graph_traits<Graph>::null_vertex());
            typename boost::detail::map_maker<
                Graph,
                ArgPack,
                boost::graph::keywords::tag::root_map,
                Vertex
            >::map_type rt_map = rt_map_gen(g, arg_pack);
            typedef typename graph_traits<Graph>::vertices_size_type VSize;
            const VSize no_vertices = VSize();
            boost::detail::make_property_map_from_arg_pack_gen<
                boost::graph::keywords::tag::discover_time_map,
                VSize
            > dtime_map_gen(no_vertices);
            typename boost::detail::map_maker<
                Graph,
                ArgPack,
                boost::graph::keywords::tag::discover_time_map,
                VSize
            >::map_type dtime_map = dtime_map_gen(g, arg_pack);
            typename boost::detail::map_maker<
                Graph,
                ArgPack,
                boost::graph::keywords::tag::color_map,
                boost::default_color_type
            >::map_type c_map = boost::detail::make_color_map_from_arg_pack(
                g,
                arg_pack
            );
            return boost::detail::strong_components_impl(
                g,
                c,
                rt_map,
                dtime_map,
                c_map
            );
        }
    };
}}} // namespace boost::graph::detail

namespace boost { namespace graph {

    // Boost.Parameter-enabled variant
    BOOST_GRAPH_MAKE_FORWARDING_FUNCTION(strong_components, 2, 6)
}} // namespace boost::graph

#endif  // not MSVC-14.0 w/64-bit addressing

namespace boost {

    using ::boost::graph::strong_components;

  template <class Graph, class ComponentMap, 
            class P, class T, class R>
  inline typename property_traits<ComponentMap>::value_type
  strong_components(const Graph& g, ComponentMap comp,
                    const bgl_named_params<P, T, R>& params
                    BOOST_GRAPH_ENABLE_IF_MODELS_PARM(Graph, vertex_list_graph_tag))
  {
    typedef typename graph_traits<Graph>::directed_category DirCat;
    BOOST_STATIC_ASSERT((boost::is_convertible<DirCat*, directed_tag*>::value));
#if !( \
        BOOST_WORKAROUND(BOOST_MSVC, >= 1900) && \
        BOOST_WORKAROUND(BOOST_MSVC, < 1910) && defined(_WIN64) \
    )
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    return detail::strong_components_impl(
      g,
      comp,
      boost::graph::keywords::_root_map = arg_pack[
        boost::graph::keywords::_root_map |
        make_shared_array_property_map(
          num_vertices(g),
          graph_traits<Graph>::null_vertex(),
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_or_dummy_property_map(g, vertex_index)
          ]
        )
      ],
      boost::graph::keywords::_discover_time_map = arg_pack[
        boost::graph::keywords::_discover_time_map |
        make_shared_array_property_map(
          num_vertices(g),
          typename graph_traits<Graph>::vertices_size_type(),
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_or_dummy_property_map(g, vertex_index)
          ]
        )
      ],
      boost::graph::keywords::_color_map = arg_pack[
        boost::graph::keywords::_color_map |
        make_shared_array_property_map(
          num_vertices(g),
          white_color,
          arg_pack[
            boost::graph::keywords::_vertex_index_map |
            detail::vertex_or_dummy_property_map(g, vertex_index)
          ]
        )
      ]
    );
#else
    return detail::scc_helper1(g, comp, params, 
                               get_param(params, vertex_root_t()));
#endif
  }

  template <typename Graph, typename ComponentMap, typename ComponentLists>
  void build_component_lists
    (const Graph& g,
     typename graph_traits<Graph>::vertices_size_type num_scc,
     ComponentMap component_number,
     ComponentLists& components)
  {
    components.resize(num_scc);
    typename graph_traits<Graph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi, vi_end) = vertices(g); vi != vi_end; ++vi)
      components[component_number[*vi]].push_back(*vi);
  }


} // namespace boost

#include <queue>
#include <vector>
#include <boost/graph/transpose_graph.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/graph/connected_components.hpp> // for components_recorder

namespace boost {

  //==========================================================================
  // This is the version of strongly connected components from
  // "Intro. to Algorithms" by Cormen, Leiserson, Rivest, which was
  // adapted from "Data Structure and Algorithms" by Aho, Hopcroft,
  // and Ullman, who credit the algorithm to S.R. Kosaraju and M. Sharir.
  // The algorithm is based on computing DFS forests the graph
  // and its transpose.

  // This algorithm is slower than Tarjan's by a constant factor, uses
  // more memory, and puts more requirements on the graph type.

  template <class Graph, class DFSVisitor, class ComponentsMap,
            class DiscoverTime, class FinishTime,
            class ColorMap>
  typename property_traits<ComponentsMap>::value_type
  kosaraju_strong_components(Graph& G, ComponentsMap c,
                             FinishTime finish_time, ColorMap color)
  {
    BOOST_CONCEPT_ASSERT(( MutableGraphConcept<Graph> ));
    // ...
    
    typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename property_traits<FinishTime>::value_type time = 0;
    depth_first_search
     (G, make_dfs_visitor(stamp_times(finish_time, time, on_finish_vertex())),
      color);

    Graph G_T(num_vertices(G));
    transpose_graph(G, G_T);

    typedef typename property_traits<ComponentsMap>::value_type count_type;

    count_type c_count(0);
    detail::components_recorder<ComponentsMap>
      vis(c, c_count);

    // initialize G_T
    typename graph_traits<Graph>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(G_T); ui != ui_end; ++ui)
      put(color, *ui, Color::white());

    typedef typename property_traits<FinishTime>::value_type D;
    typedef indirect_cmp< FinishTime, std::less<D> > Compare;

    Compare fl(finish_time);
    std::priority_queue<Vertex, std::vector<Vertex>, Compare > Q(fl);

    typename graph_traits<Graph>::vertex_iterator i, j, iend, jend;
    boost::tie(i, iend) = vertices(G_T);
    boost::tie(j, jend) = vertices(G);
    for ( ; i != iend; ++i, ++j) {
      put(finish_time, *i, get(finish_time, *j));
       Q.push(*i);
    }

    while ( !Q.empty() ) {
      Vertex u = Q.top();
      Q.pop();
      if  (get(color, u) == Color::white()) {
        depth_first_visit(G_T, u, vis, color);
        ++c_count; 
      }
    }
    return c_count;
  }

} // namespace boost

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/strong_components.hpp>)

#endif // BOOST_GRAPH_STRONG_COMPONENTS_HPP
