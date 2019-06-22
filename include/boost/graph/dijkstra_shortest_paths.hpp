//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
//
// Revision History:
//   04 April 2001: Added named parameter variant. (Jeremy Siek)
//   01 April 2001: Modified to use new <boost/limits.hpp> header. (JMaddock)
//
#ifndef BOOST_GRAPH_DIJKSTRA_HPP
#define BOOST_GRAPH_DIJKSTRA_HPP

namespace boost {

    /**
     * @brief Updates a particular value in a queue used by Dijkstra's
     * algorithm.
     *
     * This routine is called by Dijkstra's algorithm after it has
     * decreased the distance from the source vertex to the given @p
     * vertex.  By default, this routine will just call @c
     * Q.update(vertex).  However, other queues may provide more
     * specialized versions of this routine.
     *
     * @param Q             the queue that will be updated.
     * @param vertex        the vertex whose distance has been updated
     * @param old_distance  the previous distance to @p vertex
     */
    template <typename Buffer, typename Vertex, typename DistanceType>
    inline void
    dijkstra_queue_update(Buffer& Q, Vertex vertex, DistanceType old_distance)
    {
        (void)old_distance;
        Q.update(vertex);
    }
} // namespace boost

#include <boost/graph/graph_traits.hpp>
#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>

namespace boost {

    template <typename Visitor, typename Graph>
    struct DijkstraVisitorConcept
    {
        void constraints()
        {
            BOOST_CONCEPT_ASSERT(( CopyConstructibleConcept<Visitor> ));
            vis.initialize_vertex(u, g);
            vis.discover_vertex(u, g);
            vis.examine_vertex(u, g);
            vis.examine_edge(e, g);
            vis.edge_relaxed(e, g);
            vis.edge_not_relaxed(e, g);
            vis.finish_vertex(u, g);
        }

        Visitor vis;
        Graph g;
        typename graph_traits<Graph>::vertex_descriptor u;
        typename graph_traits<Graph>::edge_descriptor e;
    };
} // namespace boost

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/visitors.hpp>

namespace boost { namespace graph {

    template <typename Visitors = null_visitor>
    class dijkstra_visitor : public bfs_visitor<Visitors>
    {
    public:
        inline dijkstra_visitor()
        {
        }

        inline dijkstra_visitor(Visitors vis) : bfs_visitor<Visitors>(vis)
        {
        }

        template <typename Edge, typename Graph>
        inline void edge_relaxed(Edge e, Graph& g)
        {
            invoke_visitors(this->m_vis, e, g, on_edge_relaxed());
        }

        template <typename Edge, typename Graph>
        inline void edge_not_relaxed(Edge e, Graph& g)
        {
            invoke_visitors(this->m_vis, e, g, on_edge_not_relaxed());
        }

    private:
        template <typename Edge, typename Graph>
        inline void tree_edge(Edge u, Graph& g)
        {
        }
    };

    template <typename Visitors>
    dijkstra_visitor<Visitors>
    make_dijkstra_visitor(Visitors vis)
    {
        return dijkstra_visitor<Visitors>(vis);
    }
}} // namespace boost::graph

#include <boost/graph/relax.hpp>
#include <boost/graph/exception.hpp>
#include <boost/property_map/property_map.hpp>

#ifdef BOOST_GRAPH_DIJKSTRA_TESTING
#  include <boost/pending/mutable_queue.hpp>
#endif // BOOST_GRAPH_DIJKSTRA_TESTING

namespace boost { namespace detail {

    template <
        typename UniformCostVisitor, typename UpdatableQueue,
        typename WeightMap, typename PredecessorMap, typename DistanceMap,
        typename BinaryFunction, typename BinaryPredicate
    >
    class dijkstra_bfs_visitor
    {
        typedef typename property_traits<DistanceMap>::value_type D;
        typedef typename property_traits<WeightMap>::value_type W;

        UniformCostVisitor m_vis;
        UpdatableQueue& m_Q;
        WeightMap m_weight;
        PredecessorMap m_predecessor;
        DistanceMap m_distance;
        BinaryFunction m_combine;
        BinaryPredicate m_compare;
        D m_zero;

    public:
        inline dijkstra_bfs_visitor(
            UniformCostVisitor vis, UpdatableQueue& Q, WeightMap w,
            PredecessorMap p, DistanceMap d, BinaryFunction combine,
            BinaryPredicate compare, D zero
        ) : m_vis(vis), m_Q(Q), m_weight(w), m_predecessor(p), m_distance(d),
            m_combine(combine), m_compare(compare), m_zero(zero)
        {
        }

        template <typename Edge, typename Graph>
        inline void tree_edge(Edge e, Graph& g)
        {
            if (
                relax_target(
                    e, g, this->m_weight, this->m_predecessor,
                    this->m_distance, this->m_combine, this->m_compare
                )
            )
            {
                this->m_vis.edge_relaxed(e, g);
            }
            else
            {
                this->m_vis.edge_not_relaxed(e, g);
            }
        }

        template <typename Edge, typename Graph>
        void gray_target(Edge e, Graph& g)
        {
            D old_distance = get(this->m_distance, target(e, g));

            if (
                relax_target(
                    e, g, this->m_weight, this->m_predecessor,
                    this->m_distance, this->m_combine, this->m_compare
                )
            )
            {
                dijkstra_queue_update(this->m_Q, target(e, g), old_distance);
                this->m_vis.edge_relaxed(e, g);
            }
            else
            {
                this->m_vis.edge_not_relaxed(e, g);
            }
        }

        template <typename Vertex, typename Graph>
        inline void initialize_vertex(Vertex u, Graph& g)
        {
            this->m_vis.initialize_vertex(u, g);
        }

        template <typename Edge, typename Graph>
        inline void non_tree_edge(Edge, Graph&)
        {
        }

        template <typename Vertex, typename Graph>
        inline void discover_vertex(Vertex u, Graph& g)
        {
            this->m_vis.discover_vertex(u, g);
        }

        template <typename Vertex, typename Graph>
        inline void examine_vertex(Vertex u, Graph& g)
        {
            this->m_vis.examine_vertex(u, g);
        }

        template <typename Edge, typename Graph>
        inline void examine_edge(Edge e, Graph& g)
        {
            // Test for negative-weight edges:
            //
            // Reasons that other comparisons do not work:
            //
            // m_compare(e_weight, D(0)):
            //    m_compare only needs to work on distances, not weights,
            //    and those types do not need to be the same (bug 8398,
            //    https://svn.boost.org/trac/boost/ticket/8398).
            // m_compare(m_combine(source_dist, e_weight), source_dist):
            //    if m_combine is project2nd (as in
            //    prim_minimum_spanning_tree), this test will claim that the
            //    edge weight is negative whenever the edge weight is less
            //    than source_dist, even if both of those are positive
            //    (bug 9012, https://svn.boost.org/trac/boost/ticket/9012).
            // m_compare(m_combine(e_weight, source_dist), source_dist):
            //    would fix project2nd issue, but documentation only requires
            //    that m_combine be able to take a distance and a weight
            //    (in that order) and return a distance.

            // W e_weight = get(m_weight, e);
            // sd_plus_ew = source_dist + e_weight.
            // D sd_plus_ew = m_combine(source_dist, e_weight);
            // sd_plus_2ew = source_dist + 2 * e_weight.
            // D sd_plus_2ew = m_combine(sd_plus_ew, e_weight);
            // The test here is equivalent to e_weight < 0 if m_combine has a
            // cancellation law, but always returns false when m_combine is a
            // projection operator.
            if (
                this->m_compare(
                    this->m_combine(this->m_zero, get(this->m_weight, e)),
                    this->m_zero
                )
            )
            {
                boost::throw_exception(negative_edge());
            }
            // End of test for negative-weight edges.

            this->m_vis.examine_edge(e, g);
        }

        template <typename Edge, typename Graph>
        inline void black_target(Edge, Graph&)
        {
        }

        template <typename Vertex, typename Graph>
        inline void finish_vertex(Vertex u, Graph& g)
        {
            this->m_vis.finish_vertex(u, g);
        }
    };
}} // namespace boost::detail

#include <boost/graph/detail/d_ary_heap.hpp>
#include <boost/graph/detail/vertex_property_map_gen.hpp>
#include <boost/pending/indirect_cmp.hpp>
#include <boost/smart_ptr/scoped_array.hpp>

namespace boost { namespace graph {

    // Call breadth first search with multiple source vertices.
    template <
        typename Graph, typename SourceInputIter, typename PredecessorMap,
        typename DistanceMap, typename WeightMap, typename IndexMap,
        typename Compare, typename Combine, typename DistZero,
        typename DijkstraVisitor, typename ColorMap
    >
    void dijkstra_shortest_paths_no_init(
        const Graph& g, SourceInputIter s_begin, SourceInputIter s_end,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistZero zero,
        DijkstraVisitor vis, ColorMap color
    )
    {
        typedef indirect_cmp<DistanceMap, Compare> IndirectCmp;
        IndirectCmp icmp(distance, compare);

        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;

        // Now the default: use a d-ary heap
        boost::scoped_array<std::size_t> index_in_heap_map_holder;
        typedef boost::detail::vertex_property_map_generator<
            Graph, IndexMap, std::size_t
        > IndexInHeapMapHelper;
        typedef typename IndexInHeapMapHelper::type IndexInHeapMap;
        IndexInHeapMap index_in_heap = IndexInHeapMapHelper(
            g, index_map, index_in_heap_map_holder
        )();
        typedef d_ary_heap_indirect<
            Vertex, 4, IndexInHeapMap, DistanceMap, Compare
        > MutableQueue;
        MutableQueue Q(distance, index_in_heap, compare);
        boost::detail::dijkstra_bfs_visitor<
            DijkstraVisitor, MutableQueue, WeightMap, PredecessorMap,
            DistanceMap, Combine, Compare
        > bfs_vis(
            vis, Q, weight, predecessor, distance, combine, compare, zero
        );

        breadth_first_visit(g, s_begin, s_end, Q, bfs_vis, color);
    }

    // Call breadth first search with a single source vertex.
    template <
        typename Graph, typename PredecessorMap, typename DistanceMap,
        typename WeightMap, typename IndexMap, typename Compare,
        typename Combine, typename DistZero, typename DijkstraVisitor,
        typename ColorMap
    >
    inline void dijkstra_shortest_paths_no_init(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistZero zero,
        DijkstraVisitor vis, ColorMap color
    )
    {
        typename graph_traits<Graph>::vertex_descriptor srcs[1] = {s};
        dijkstra_shortest_paths_no_init(
            g, srcs, srcs + 1, predecessor, distance, weight, index_map,
            compare, combine, zero, vis, color
        );
    }
}} // namespace boost::graph

#include <boost/graph/detail/two_bit_color_map_generator.hpp>

namespace boost { namespace graph {

    // Call breadth first search with multiple source vertices
    // and the default color map.
    template <
        typename Graph, typename SourceInputIter, typename PredecessorMap,
        typename DistanceMap, typename WeightMap, typename IndexMap,
        typename Compare, typename Combine, typename DistZero,
        typename DijkstraVisitor
    >
    inline void dijkstra_shortest_paths_no_init(
        const Graph& g, SourceInputIter s_begin, SourceInputIter s_end,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistZero zero,
        DijkstraVisitor vis
    )
    {
        typedef boost::detail::two_bit_color_map_generator<
            Graph, IndexMap
        > ColorMapHelper;
        typedef typename ColorMapHelper::type ColorMap;
        ColorMap color = ColorMapHelper(g, index_map)();
        dijkstra_shortest_paths_no_init(
            g, s_begin, s_end, predecessor, distance, weight, index_map,
            compare, combine, zero, vis, color
        );
    }

    // Call breadth first search with a single source vertex
    // and the default color map.
    template <
        typename Graph, typename PredecessorMap, typename DistanceMap,
        typename WeightMap, typename IndexMap, typename Compare,
        typename Combine, typename DistZero, typename DijkstraVisitor
    >
    inline void dijkstra_shortest_paths_no_init(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistZero zero,
        DijkstraVisitor vis
    )
    {
        typename graph_traits<Graph>::vertex_descriptor srcs[1] = {s};
        dijkstra_shortest_paths_no_init(
            g, srcs, srcs + 1, predecessor, distance, weight, index_map,
            compare, combine, zero, vis
        );
    }

    // Initialize distances and call breadth first search
    // with multiple source vertices.
    template <
        typename Graph, typename SourceInputIter, typename PredecessorMap,
        typename DistanceMap, typename WeightMap, typename IndexMap,
        typename Compare, typename Combine, typename DistInf,
        typename DistZero, typename DijkstraVisitor, typename ColorMap
    >
    void dijkstra_shortest_paths(
        const Graph& g, SourceInputIter s_begin, SourceInputIter s_end,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistInf inf,
        DistZero zero, DijkstraVisitor vis, ColorMap color
    )
    {
        typedef typename property_traits<ColorMap>::value_type ColorValue;
        typedef color_traits<ColorValue> Color;
        typename graph_traits<Graph>::vertex_iterator ui, ui_end;

        for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui)
        {
            vis.initialize_vertex(*ui, g);
            put(distance, *ui, inf);
            put(predecessor, *ui, *ui);
            put(color, *ui, Color::white());
        }

        for (SourceInputIter it = s_begin; it != s_end; ++it)
        {
            put(distance, *it, zero);
        }

        dijkstra_shortest_paths_no_init(
            g, s_begin, s_end, predecessor, distance, weight, index_map,
            compare, combine, zero, vis, color
        );
    }
}} // namespace boost::graph

#include <boost/graph/two_bit_color_map.hpp>

namespace boost { namespace graph {

    // Initialize distances and call breadth first search
    // with multiple source vertices and the default color map.
    template <
        typename Graph, typename SourceInputIter, typename PredecessorMap,
        typename DistanceMap, typename WeightMap, typename IndexMap,
        typename Compare, typename Combine, typename DistInf,
        typename DistZero, typename DijkstraVisitor
    >
    inline void dijkstra_shortest_paths(
        const Graph& g, SourceInputIter s_begin, SourceInputIter s_end,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistInf inf,
        DistZero zero, DijkstraVisitor vis
    )
    {
        boost::two_bit_color_map<IndexMap> color(num_vertices(g), index_map);
        dijkstra_shortest_paths(
            g, s_begin, s_end, predecessor, distance, weight, index_map,
            compare, combine, inf, zero, vis, color// no_named_parameters()
        );
    }
}} // namespace boost::graph

#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

    // Initialize distances and call breadth first search
    // with a single source vertex.
    template <
        typename Graph, typename PredecessorMap, typename DistanceMap,
        typename WeightMap, typename IndexMap, typename Compare,
        typename Combine, typename DistInf, typename DistZero,
        typename DijkstraVisitor, typename ColorMap
    >
    inline void dijkstra_shortest_paths(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistInf inf,
        DistZero zero, DijkstraVisitor vis, ColorMap color,
        typename boost::disable_if<
            parameter::are_tagged_arguments<
                PredecessorMap, DistanceMap, WeightMap, IndexMap, Compare,
                Combine, DistInf, DistZero, DijkstraVisitor, ColorMap
            >,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename graph_traits<Graph>::vertex_descriptor srcs[1] = {s};
        dijkstra_shortest_paths(
            g, srcs, srcs + 1, predecessor, distance, weight, index_map,
            compare, combine, inf, zero, vis, color
        );
    }

    // Initialize distances and call breadth first search
    // with a single source vertex and the default color map.
    template <
        typename Graph, typename PredecessorMap, typename DistanceMap,
        typename WeightMap, typename IndexMap, typename Compare,
        typename Combine, typename DistInf, typename DistZero,
        typename DijkstraVisitor
    >
    inline void dijkstra_shortest_paths(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
        IndexMap index_map, Compare compare, Combine combine, DistInf inf,
        DistZero zero, DijkstraVisitor vis, typename boost::disable_if<
            parameter::are_tagged_arguments<
                PredecessorMap, DistanceMap, WeightMap, IndexMap, Compare,
                Combine, DistInf, DistZero, DijkstraVisitor
            >,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename graph_traits<Graph>::vertex_descriptor srcs[1] = {s};
        dijkstra_shortest_paths(
            g, srcs, srcs + 1, predecessor, distance, weight, index_map,
            compare, combine, inf, zero, vis
        );
    }
}} // namespace boost::graph

#include <boost/graph/named_function_params.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/functional/value_factory.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <functional>

namespace boost { namespace graph {

    template <typename Graph, typename Args>
    void dijkstra_shortest_paths(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        const Args& arg_pack, typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename graph_traits<Graph>::vertex_descriptor srcs[1] = {s};
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            Graph
        >::type IndexMap;
        IndexMap v_i_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
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
        typedef typename boost::property_traits<WeightMap>::value_type Weight;
        const Weight zero_weight = Weight();
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::distance_map,
            Weight
        > v_d_map_gen(zero_weight);
        typedef typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type DistanceMap;
        DistanceMap v_d_map = v_d_map_gen(g, arg_pack);
        typedef typename boost::property_traits<DistanceMap>::value_type D;
        const D inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<D>()
        ];
        const D zero_distance = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<D>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                dijkstra_visitor<>
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<dijkstra_visitor<> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::predecessor_map,
                dummy_property_map
            >::type
        >::type v_p_map = arg_pack[
            boost::graph::keywords::_predecessor_map ||
            boost::value_factory<dummy_property_map>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_compare,
                std::less<D>
            >::type
        >::type dist_comp = arg_pack[
            boost::graph::keywords::_distance_compare ||
            boost::value_factory<std::less<D> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::distance_combine,
                std::plus<D>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            boost::value_factory<std::plus<D> >()
        ];
        typename boost::remove_const<
            typename boost::parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::color_map,
                boost::detail::two_bit_color_map_generator<
                    Graph,
                    IndexMap
                >
            >::type
        >::type v_c_map = arg_pack[
            boost::graph::keywords::_color_map ||
            boost::detail::two_bit_color_map_generator<
                Graph,
                IndexMap
            >(g, v_i_map)
        ];
        dijkstra_shortest_paths(
            g, srcs, srcs + 1, v_p_map, v_d_map, e_w_map, v_i_map,
            dist_comp, dist_comb, inf, zero_distance, vis, v_c_map
        );
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const Graph& g, \
        typename graph_traits<Graph>::vertex_descriptor s, \
        const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, mpl::true_ \
        >::type = mpl::true_() \
    ) \
    { \
        name( \
            g, s, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 11, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, dijkstra_shortest_paths
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

    using ::boost::graph::dijkstra_shortest_paths_no_init;
    using ::boost::graph::dijkstra_shortest_paths;
    using ::boost::graph::dijkstra_visitor;
    using ::boost::graph::make_dijkstra_visitor;
} // namespace boost

#include <boost/graph/overloading.hpp>

namespace boost {

  // Initialize distances and call breadth first search with default color map
  template <class VertexListGraph, class SourceInputIter, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, typename T, typename Tag,
            typename Base>
  inline void
  dijkstra_shortest_paths
    (const VertexListGraph& g,
     SourceInputIter s_begin, SourceInputIter s_end,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis,
     const bgl_named_params<T, Tag, Base>&
     BOOST_GRAPH_ENABLE_IF_MODELS_PARM(VertexListGraph,vertex_list_graph_tag))
  {
    boost::two_bit_color_map<IndexMap> color(num_vertices(g), index_map);
    dijkstra_shortest_paths(g, s_begin, s_end, predecessor, distance, weight,
                            index_map, compare, combine, inf, zero, vis,
                            color);
  }

  // Initialize distances and call breadth first search with default color map
  template <class VertexListGraph, class DijkstraVisitor,
            class PredecessorMap, class DistanceMap,
            class WeightMap, class IndexMap, class Compare, class Combine,
            class DistInf, class DistZero, typename T, typename Tag,
            typename Base>
  inline void
  dijkstra_shortest_paths
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     PredecessorMap predecessor, DistanceMap distance, WeightMap weight,
     IndexMap index_map,
     Compare compare, Combine combine, DistInf inf, DistZero zero,
     DijkstraVisitor vis,
     const bgl_named_params<T, Tag, Base>&
     BOOST_GRAPH_ENABLE_IF_MODELS_PARM(VertexListGraph,vertex_list_graph_tag))
  {
    typename graph_traits<VertexListGraph>::vertex_descriptor srcs[1] = {s};
    dijkstra_shortest_paths(g, srcs, srcs + 1, predecessor, distance, weight,
                            index_map, compare, combine, inf, zero, vis);
  }
} // namespace boost

#include <boost/limits.hpp>

namespace boost { namespace detail {

    // Handle defaults for PredecessorMap and
    // Distance Compare, Combine, Inf and Zero
    template <class VertexListGraph, class DistanceMap, class WeightMap,
              class IndexMap, class Param, class Tag, class Rest>
    inline void
    dijkstra_dispatch2
      (const VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       DistanceMap distance, WeightMap weight, IndexMap index_map,
       const bgl_named_params<Param, Tag, Rest>& params)
    {
      // Default for predecessor map
      dummy_property_map p_map;

      typedef typename property_traits<DistanceMap>::value_type D;
      D inf = choose_param(get_param(params, distance_inf_t()),
                           (std::numeric_limits<D>::max)());

      dijkstra_shortest_paths
        (g, s,
         choose_param(get_param(params, vertex_predecessor), p_map),
         distance, weight, index_map,
         choose_param(get_param(params, distance_compare_t()),
                      std::less<D>()),
         choose_param(get_param(params, distance_combine_t()),
                      std::plus<D>()),
         inf,
         choose_param(get_param(params, distance_zero_t()),
                      D()),
         choose_param(get_param(params, graph_visitor),
                      make_dijkstra_visitor(null_visitor())),
         params);
    }
}} // namespace boost::detail

#include <vector>

namespace boost { namespace detail {

    template <class VertexListGraph, class DistanceMap, class WeightMap,
              class IndexMap, class Param, class Tag, class Rest>
    inline void
    dijkstra_dispatch1
      (const VertexListGraph& g,
       typename graph_traits<VertexListGraph>::vertex_descriptor s,
       DistanceMap distance, WeightMap weight, IndexMap index_map,
       const bgl_named_params<Param, Tag, Rest>& params)
    {
      // Default for distance map
      typedef typename property_traits<WeightMap>::value_type D;
      typename std::vector<D>::size_type
        n = is_default_param(distance) ? num_vertices(g) : 1;
      std::vector<D> distance_map(n);

      detail::dijkstra_dispatch2
        (g, s, choose_param(distance, make_iterator_property_map
                            (distance_map.begin(), index_map,
                             distance_map[0])),
         weight, index_map, params);
    }
}} // namespace boost::detail

namespace boost {

  // Old-style named parameter variant
  template <class VertexListGraph, class Param, class Tag, class Rest>
  inline void
  dijkstra_shortest_paths
    (const VertexListGraph& g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     const bgl_named_params<Param, Tag, Rest>& params)
  {
    // Default for edge weight and vertex index map is to ask for them
    // from the graph.  Default for the visitor is null_visitor.
    detail::dijkstra_dispatch1
      (g, s,
       get_param(params, vertex_distance),
       choose_const_pmap(get_param(params, edge_weight), g, edge_weight),
       choose_const_pmap(get_param(params, vertex_index), g, vertex_index),
       params);
  }
} // namespace boost

#include <boost/graph/detail/mpi_include.hpp>

#include BOOST_GRAPH_MPI_INCLUDE(<boost/graph/distributed/dijkstra_shortest_paths.hpp>)

#endif // BOOST_GRAPH_DIJKSTRA_HPP
