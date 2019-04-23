

//
//=======================================================================
// Copyright (c) 2004 Kristopher Beevers
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//

#ifndef BOOST_GRAPH_ASTAR_SEARCH_HPP
#define BOOST_GRAPH_ASTAR_SEARCH_HPP

#include <utility>

namespace boost { namespace graph_detail {

    template <typename A, typename B>
    struct select1st
    {
        typedef std::pair<A, B> argument_type;
        typedef A result_type;

        inline A operator()(const std::pair<A, B>& p) const
        {
            return p.first;
        }
    };
}} // namespace boost::graph_detail

#include <boost/graph/graph_traits.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename CostType>
    struct astar_heuristic
    {
        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        typedef Vertex argument_type;
        typedef CostType result_type;

        inline astar_heuristic()
        {
        }

        inline CostType operator()(Vertex u) const
        {
            return static_cast<CostType>(0);
        }
    };
}} // namespace boost::graph

#include <boost/concept_check.hpp>
#include <boost/concept/assert.hpp>

namespace boost {

    template <typename Heuristic, typename Graph>
    struct AStarHeuristicConcept
    {
        void constraints()
        {
            BOOST_CONCEPT_ASSERT(( CopyConstructibleConcept<Heuristic> ));
            h(u);
        }

        Heuristic h;
        typename graph_traits<Graph>::vertex_descriptor u;
    };

    template <typename Visitor, typename Graph>
    struct AStarVisitorConcept
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
            vis.black_target(e, g);
            vis.finish_vertex(u, g);
        }

        Visitor vis;
        Graph g;
        typename graph_traits<Graph>::vertex_descriptor u;
        typename graph_traits<Graph>::edge_descriptor e;
    };
} // namespace boost

#include <boost/graph/breadth_first_search.hpp>

namespace boost { namespace graph {

    template <typename Visitors = null_visitor>
    class astar_visitor : public bfs_visitor<Visitors>
    {
    public:
        inline astar_visitor()
        {
        }

        inline astar_visitor(Visitors vis) : bfs_visitor<Visitors>(vis)
        {
        }

        template <typename Edge, typename Graph>
        inline void edge_relaxed(Edge e, const Graph& g)
        {
            invoke_visitors(this->m_vis, e, g, on_edge_relaxed());
        }

        template <typename Edge, typename Graph>
        inline void edge_not_relaxed(Edge e, const Graph& g)
        {
            invoke_visitors(this->m_vis, e, g, on_edge_not_relaxed());
        }

    private:
        template <typename Edge, typename Graph>
        inline void tree_edge(Edge e, const Graph& g)
        {
        }

        template <typename Edge, typename Graph>
        inline void non_tree_edge(Edge e, const Graph& g)
        {
        }
    };

    template <typename Visitors>
    astar_visitor<Visitors>
    make_astar_visitor(Visitors vis)
    {
        return astar_visitor<Visitors>(vis);
    }

    typedef astar_visitor<> default_astar_visitor;
}} // namespace boost::graph

#include <boost/graph/relax.hpp>
#include <boost/graph/exception.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/throw_exception.hpp>
#include <boost/limits.hpp>
#include <vector>
#include <functional>

namespace boost { namespace detail {

    template <class AStarHeuristic, class UniformCostVisitor,
              class UpdatableQueue, class PredecessorMap,
              class CostMap, class DistanceMap, class WeightMap,
              class ColorMap, class BinaryFunction,
              class BinaryPredicate>
    struct astar_bfs_visitor
    {

      typedef typename property_traits<CostMap>::value_type C;
      typedef typename property_traits<ColorMap>::value_type ColorValue;
      typedef color_traits<ColorValue> Color;
      typedef typename property_traits<DistanceMap>::value_type distance_type;

      astar_bfs_visitor(AStarHeuristic h, UniformCostVisitor vis,
                        UpdatableQueue& Q, PredecessorMap p,
                        CostMap c, DistanceMap d, WeightMap w,
                        ColorMap col, BinaryFunction combine,
                        BinaryPredicate compare, C zero)
        : m_h(h), m_vis(vis), m_Q(Q), m_predecessor(p), m_cost(c),
          m_distance(d), m_weight(w), m_color(col),
          m_combine(combine), m_compare(compare), m_zero(zero) {}


      template <class Vertex, class Graph>
      void initialize_vertex(Vertex u, const Graph& g) {
        m_vis.initialize_vertex(u, g);
      }
      template <class Vertex, class Graph>
      void discover_vertex(Vertex u, const Graph& g) {
        m_vis.discover_vertex(u, g);
      }
      template <class Vertex, class Graph>
      void examine_vertex(Vertex u, const Graph& g) {
        m_vis.examine_vertex(u, g);
      }
      template <class Vertex, class Graph>
      void finish_vertex(Vertex u, const Graph& g) {
        m_vis.finish_vertex(u, g);
      }
      template <class Edge, class Graph>
      void examine_edge(Edge e, const Graph& g) {
        if (m_compare(get(m_weight, e), m_zero))
          BOOST_THROW_EXCEPTION(negative_edge());
        m_vis.examine_edge(e, g);
      }
      template <class Edge, class Graph>
      void non_tree_edge(Edge, const Graph&) {}



      template <class Edge, class Graph>
      void tree_edge(Edge e, const Graph& g) {
        using boost::get;
        bool m_decreased =
          relax(e, g, m_weight, m_predecessor, m_distance,
                m_combine, m_compare);

        if(m_decreased) {
          m_vis.edge_relaxed(e, g);
          put(m_cost, target(e, g),
              m_combine(get(m_distance, target(e, g)),
                        m_h(target(e, g))));
        } else
          m_vis.edge_not_relaxed(e, g);
      }


      template <class Edge, class Graph>
      void gray_target(Edge e, const Graph& g) {
        using boost::get;
        bool m_decreased =
          relax(e, g, m_weight, m_predecessor, m_distance,
                m_combine, m_compare);

        if(m_decreased) {
          put(m_cost, target(e, g),
              m_combine(get(m_distance, target(e, g)),
                        m_h(target(e, g))));
          m_Q.update(target(e, g));
          m_vis.edge_relaxed(e, g);
        } else
          m_vis.edge_not_relaxed(e, g);
      }


      template <class Edge, class Graph>
      void black_target(Edge e, const Graph& g) {
        using boost::get;
        bool m_decreased =
          relax(e, g, m_weight, m_predecessor, m_distance,
                m_combine, m_compare);

        if(m_decreased) {
          m_vis.edge_relaxed(e, g);
          put(m_cost, target(e, g),
              m_combine(get(m_distance, target(e, g)),
                        m_h(target(e, g))));
          m_Q.push(target(e, g));
          put(m_color, target(e, g), Color::gray());
          m_vis.black_target(e, g);
        } else
          m_vis.edge_not_relaxed(e, g);
      }



      AStarHeuristic m_h;
      UniformCostVisitor m_vis;
      UpdatableQueue& m_Q;
      PredecessorMap m_predecessor;
      CostMap m_cost;
      DistanceMap m_distance;
      WeightMap m_weight;
      ColorMap m_color;
      BinaryFunction m_combine;
      BinaryPredicate m_compare;
      C m_zero;

    };
}} // namespace boost::detail

#include <boost/graph/iteration_macros.hpp>
#include <boost/graph/detail/d_ary_heap.hpp>
#include <boost/graph/property_maps/constant_property_map.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/property_map/function_property_map.hpp>
#include <boost/parameter/are_tagged_arguments.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/core/enable_if.hpp>

namespace boost { namespace graph {

  template <typename VertexListGraph, typename AStarHeuristic,
            typename AStarVisitor, typename PredecessorMap,
            typename CostMap, typename DistanceMap,
            typename WeightMap, typename ColorMap,
            typename VertexIndexMap,
            typename CompareFunction, typename CombineFunction,
            typename CostInf, typename CostZero>
  inline void
  astar_search_no_init
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, AStarVisitor vis,
     PredecessorMap predecessor, CostMap cost,
     DistanceMap distance, WeightMap weight,
     ColorMap color, VertexIndexMap index_map,
     CompareFunction compare, CombineFunction combine,
     CostInf /*inf*/, CostZero zero, typename boost::disable_if<
       parameter::are_tagged_arguments<
         AStarVisitor, PredecessorMap, CostMap, DistanceMap, WeightMap,
         ColorMap, VertexIndexMap, CompareFunction, CombineFunction, CostInf,
         CostZero
       >,
       mpl::true_
     >::type = mpl::true_())
  {
    typedef typename graph_traits<VertexListGraph>::vertex_descriptor
      Vertex;
    typedef boost::vector_property_map<std::size_t, VertexIndexMap> IndexInHeapMap;
    IndexInHeapMap index_in_heap(index_map);
    typedef d_ary_heap_indirect<Vertex, 4, IndexInHeapMap, CostMap, CompareFunction>
      MutableQueue;
    MutableQueue Q(cost, index_in_heap, compare);

    boost::detail::astar_bfs_visitor<AStarHeuristic, AStarVisitor,
        MutableQueue, PredecessorMap, CostMap, DistanceMap,
        WeightMap, ColorMap, CombineFunction, CompareFunction>
      bfs_vis(h, vis, Q, predecessor, cost, distance, weight,
              color, combine, compare, zero);

    breadth_first_visit(g, s, Q, bfs_vis, color);
  }

  template <typename VertexListGraph, typename AStarHeuristic,
            typename AStarVisitor, typename PredecessorMap,
            typename CostMap, typename DistanceMap,
            typename WeightMap,
            typename CompareFunction, typename CombineFunction,
            typename CostInf, typename CostZero>
  inline void
  astar_search_no_init_tree
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, AStarVisitor vis,
     PredecessorMap predecessor, CostMap cost,
     DistanceMap distance, WeightMap weight,
     CompareFunction compare, CombineFunction combine,
     CostInf /*inf*/, CostZero zero, typename boost::disable_if<
       parameter::are_tagged_arguments<
         AStarVisitor, PredecessorMap, CostMap, DistanceMap, WeightMap,
         CompareFunction, CombineFunction, CostInf, CostZero
       >,
       mpl::true_
     >::type = mpl::true_())
  {
    typedef typename graph_traits<VertexListGraph>::vertex_descriptor
      Vertex;
    typedef typename property_traits<DistanceMap>::value_type Distance;
    typedef d_ary_heap_indirect<
              std::pair<Distance, Vertex>,
              4,
              null_property_map<std::pair<Distance, Vertex>, std::size_t>,
              function_property_map<graph_detail::select1st<Distance, Vertex>, std::pair<Distance, Vertex> >,
              CompareFunction>
      MutableQueue;
    MutableQueue Q(
      make_function_property_map<std::pair<Distance, Vertex> >(boost::graph_detail::select1st<Distance, Vertex>()),
      null_property_map<std::pair<Distance, Vertex>, std::size_t>(),
      compare);

    vis.discover_vertex(s, g);
    Q.push(std::make_pair(get(cost, s), s));
    while (!Q.empty()) {
      Vertex v;
      Distance v_rank;
      boost::tie(v_rank, v) = Q.top();
      Q.pop();
      vis.examine_vertex(v, g);
      BGL_FORALL_OUTEDGES_T(v, e, g, VertexListGraph) {
        Vertex w = target(e, g);
        vis.examine_edge(e, g);
        Distance e_weight = get(weight, e);
        if (compare(e_weight, zero))
          BOOST_THROW_EXCEPTION(negative_edge());
        bool decreased =
          relax(e, g, weight, predecessor, distance,
                combine, compare);
        combine(get(distance, v), e_weight);
        if (decreased) {
          vis.edge_relaxed(e, g);
          Distance w_rank = combine(get(distance, w), h(w));
          put(cost, w, w_rank);
          vis.discover_vertex(w, g);
          Q.push(std::make_pair(w_rank, w));
        } else {
          vis.edge_not_relaxed(e, g);
        }
      }
      vis.finish_vertex(v, g);
    }
  }

  // Non-named parameter interface
  template <typename VertexListGraph, typename AStarHeuristic,
            typename AStarVisitor, typename PredecessorMap,
            typename CostMap, typename DistanceMap,
            typename WeightMap, typename VertexIndexMap,
            typename ColorMap,
            typename CompareFunction, typename CombineFunction,
            typename CostInf, typename CostZero>
  inline void
  astar_search
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, AStarVisitor vis,
     PredecessorMap predecessor, CostMap cost,
     DistanceMap distance, WeightMap weight,
     VertexIndexMap index_map, ColorMap color,
     CompareFunction compare, CombineFunction combine,
     CostInf inf, CostZero zero, typename boost::disable_if<
       parameter::are_tagged_arguments<
         AStarVisitor, PredecessorMap, CostMap, DistanceMap, WeightMap,
         VertexIndexMap, ColorMap, CompareFunction, CombineFunction, CostInf,
         CostZero
       >,
       mpl::true_
     >::type = mpl::true_())
  {

    typedef typename property_traits<ColorMap>::value_type ColorValue;
    typedef color_traits<ColorValue> Color;
    typename graph_traits<VertexListGraph>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
      put(color, *ui, Color::white());
      put(distance, *ui, inf);
      put(cost, *ui, inf);
      put(predecessor, *ui, *ui);
      vis.initialize_vertex(*ui, g);
    }
    put(distance, s, zero);
    put(cost, s, h(s));

    astar_search_no_init
      (g, s, h, vis, predecessor, cost, distance, weight,
       color, index_map, compare, combine, inf, zero);

  }

  // Non-named parameter interface
  template <typename VertexListGraph, typename AStarHeuristic,
            typename AStarVisitor, typename PredecessorMap,
            typename CostMap, typename DistanceMap,
            typename WeightMap,
            typename CompareFunction, typename CombineFunction,
            typename CostInf, typename CostZero>
  inline void
  astar_search_tree
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, AStarVisitor vis,
     PredecessorMap predecessor, CostMap cost,
     DistanceMap distance, WeightMap weight,
     CompareFunction compare, CombineFunction combine,
     CostInf inf, CostZero zero, typename boost::disable_if<
       parameter::are_tagged_arguments<
         AStarVisitor, PredecessorMap, CostMap, DistanceMap, WeightMap,
         CompareFunction, CombineFunction, CostInf, CostZero
       >,
       mpl::true_
     >::type = mpl::true_())
  {

    typename graph_traits<VertexListGraph>::vertex_iterator ui, ui_end;
    for (boost::tie(ui, ui_end) = vertices(g); ui != ui_end; ++ui) {
      put(distance, *ui, inf);
      put(cost, *ui, inf);
      put(predecessor, *ui, *ui);
      vis.initialize_vertex(*ui, g);
    }
    put(distance, s, zero);
    put(cost, s, h(s));

    astar_search_no_init_tree
      (g, s, h, vis, predecessor, cost, distance, weight,
       compare, combine, inf, zero);

  }
}} // namespace boost::graph

#include <boost/graph/named_function_params.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/functional/value_factory.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace boost { namespace graph {

    // Named parameter interfaces
    template <
        typename VertexListGraph, typename AStarHeuristic, typename Args
    >
    void astar_search(
        const VertexListGraph& g,
        typename graph_traits<VertexListGraph>::vertex_descriptor s,
        AStarHeuristic h,
        const Args& arg_pack,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                default_astar_visitor
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<default_astar_visitor>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::predecessor_map,
                dummy_property_map
            >::type
        >::type pred_map = arg_pack[
            boost::graph::keywords::_predecessor_map ||
            boost::value_factory<dummy_property_map>()
        ];
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            VertexListGraph
        >::type v_i_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
        );
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::color_map,
            boost::default_color_type
        >::map_type c_map = boost::detail::make_color_map_from_arg_pack(
            g,
            arg_pack
        );
        // Distance type is the value type of the distance map
        // if there is one, otherwise the value type of the weight map.
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            VertexListGraph
        >::type WeightMap;
        WeightMap w_map = boost::detail::override_const_property(
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
        > dist_map_gen(zero_weight);
        typedef typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type DistanceMap;
        DistanceMap dist_map = dist_map_gen(g, arg_pack);
        typedef typename boost::property_traits<DistanceMap>::value_type D;
        const D inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<D>()
        ];
        const D zero_actual = D();
        const D zero_distance = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<D>()
        ];
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::rank_map,
            D
        > rank_map_gen(zero_actual);
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::rank_map,
            D
        >::map_type r_map = rank_map_gen(g, arg_pack);
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
                closed_plus<D>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            closed_plus_gen<D>(inf)
        ];
        astar_search(
            g, s, h, vis, pred_map, r_map, dist_map, w_map, v_i_map, c_map,
            dist_comp, dist_comb, inf, zero_distance
        );
    }

    template <
        typename VertexListGraph, typename AStarHeuristic, typename Args
    >
    void astar_search_tree(
        const VertexListGraph& g,
        typename graph_traits<VertexListGraph>::vertex_descriptor s,
        AStarHeuristic h,
        const Args& arg_pack,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                default_astar_visitor
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<default_astar_visitor>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::predecessor_map,
                dummy_property_map
            >::type
        >::type pred_map = arg_pack[
            boost::graph::keywords::_predecessor_map ||
            boost::value_factory<dummy_property_map>()
        ];
        // Distance type is the value type of the distance map
        // if there is one, otherwise the value type of the weight map.
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            VertexListGraph
        >::type WeightMap;
        WeightMap w_map = boost::detail::override_const_property(
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
        > dist_map_gen(zero_weight);
        typedef typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type DistanceMap;
        DistanceMap dist_map = dist_map_gen(g, arg_pack);
        typedef typename boost::property_traits<DistanceMap>::value_type D;
        const D inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<D>()
        ];
        const D zero_actual = D();
        const D zero_distance = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<D>()
        ];
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::rank_map,
            D
        > rank_map_gen(zero_actual);
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::rank_map,
            D
        >::map_type r_map = rank_map_gen(g, arg_pack);
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
                closed_plus<D>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            closed_plus_gen<D>(inf)
        ];
        astar_search_tree(
            g, s, h, vis, pred_map, r_map, dist_map, w_map,
            dist_comp, dist_comb, inf, zero_distance
        );
    }

    template <
        typename VertexListGraph, typename AStarHeuristic, typename Args
    >
    void astar_search_no_init(
        const VertexListGraph& g,
        typename graph_traits<VertexListGraph>::vertex_descriptor s,
        AStarHeuristic h,
        const Args& arg_pack,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                default_astar_visitor
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<default_astar_visitor>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::predecessor_map,
                dummy_property_map
            >::type
        >::type pred_map = arg_pack[
            boost::graph::keywords::_predecessor_map ||
            boost::value_factory<dummy_property_map>()
        ];
        typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            VertexListGraph
        >::type v_i_map = boost::detail::override_const_property(
            arg_pack,
            boost::graph::keywords::_vertex_index_map,
            g,
            vertex_index
        );
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::color_map,
            boost::default_color_type
        >::map_type c_map = boost::detail::make_color_map_from_arg_pack(
            g,
            arg_pack
        );
        // Distance type is the value type of the distance map
        // if there is one, otherwise the value type of the weight map.
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            VertexListGraph
        >::type WeightMap;
        WeightMap w_map = boost::detail::override_const_property(
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
        > dist_map_gen(zero_weight);
        typedef typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type DistanceMap;
        DistanceMap dist_map = dist_map_gen(g, arg_pack);
        typedef typename boost::property_traits<DistanceMap>::value_type D;
        const D inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<D>()
        ];
        const D zero_actual = D();
        const D zero_distance = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<D>()
        ];
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::rank_map,
            D
        > rank_map_gen(zero_actual);
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::rank_map,
            D
        >::map_type r_map = rank_map_gen(g, arg_pack);
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
                closed_plus<D>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            closed_plus_gen<D>(inf)
        ];
        astar_search_no_init(
            g, s, h, vis, pred_map, r_map, dist_map, w_map, c_map, v_i_map,
            dist_comp, dist_comb, inf, zero_distance
        );
    }

    template <
        typename VertexListGraph, typename AStarHeuristic, typename Args
    >
    void astar_search_no_init_tree(
        const VertexListGraph& g,
        typename graph_traits<VertexListGraph>::vertex_descriptor s,
        AStarHeuristic h,
        const Args& arg_pack,
        typename boost::enable_if<
            parameter::is_argument_pack<Args>,
            mpl::true_
        >::type = mpl::true_()
    )
    {
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::visitor,
                default_astar_visitor
            >::type
        >::type vis = arg_pack[
            boost::graph::keywords::_visitor ||
            boost::value_factory<default_astar_visitor>()
        ];
        typename boost::remove_const<
            typename boost::parameter::value_type<
                Args,
                boost::graph::keywords::tag::predecessor_map,
                dummy_property_map
            >::type
        >::type pred_map = arg_pack[
            boost::graph::keywords::_predecessor_map ||
            boost::value_factory<dummy_property_map>()
        ];
        // Distance type is the value type of the distance map
        // if there is one, otherwise the value type of the weight map.
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::weight_map,
            edge_weight_t,
            VertexListGraph
        >::type WeightMap;
        WeightMap w_map = boost::detail::override_const_property(
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
        > dist_map_gen(zero_weight);
        typedef typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::distance_map,
            Weight
        >::map_type DistanceMap;
        DistanceMap dist_map = dist_map_gen(g, arg_pack);
        typedef typename boost::property_traits<DistanceMap>::value_type D;
        const D inf = arg_pack[
            boost::graph::keywords::_distance_inf ||
            boost::detail::get_max<D>()
        ];
        const D zero_actual = D();
        const D zero_distance = arg_pack[
            boost::graph::keywords::_distance_zero ||
            boost::value_factory<D>()
        ];
        boost::detail::make_property_map_from_arg_pack_gen<
            boost::graph::keywords::tag::rank_map,
            D
        > rank_map_gen(zero_actual);
        typename boost::detail::map_maker<
            VertexListGraph,
            Args,
            boost::graph::keywords::tag::rank_map,
            D
        >::map_type r_map = rank_map_gen(g, arg_pack);
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
                closed_plus<D>
            >::type
        >::type dist_comb = arg_pack[
            boost::graph::keywords::_distance_combine ||
            closed_plus_gen<D>(inf)
        ];
        astar_search_no_init_tree(
            g, s, h, vis, pred_map, r_map, dist_map, w_map,
            dist_comp, dist_comb, inf, zero_distance
        );
    }
}} // namespace boost::graph

#include <boost/parameter/compose.hpp>
#include <boost/preprocessor/repetition/enum_trailing_binary_params.hpp>
#include <boost/preprocessor/repetition/enum_trailing_params.hpp>

#define BOOST_GRAPH_PP_FUNCTION_OVERLOAD(z, n, name) \
    template < \
        typename Graph, typename H, typename TA \
        BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, typename TA) \
    > \
    inline void name( \
        const Graph &g, typename graph_traits<Graph>::vertex_descriptor s, \
        H h, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta), \
        typename boost::enable_if< \
            parameter::are_tagged_arguments< \
                TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
            >, mpl::true_ \
        >::type = mpl::true_() \
    ) \
    { \
        name( \
            g, s, h, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(
    1, 12, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, astar_search
)
BOOST_PP_REPEAT_FROM_TO(
    1, 10, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, astar_search_tree
)
BOOST_PP_REPEAT_FROM_TO(
    1, 12, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, astar_search_no_init
)
BOOST_PP_REPEAT_FROM_TO(
    1, 10, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, astar_search_no_init_tree
)
}} // namespace boost::graph

#undef BOOST_GRAPH_PP_FUNCTION_OVERLOAD

namespace boost {

    using ::boost::graph::astar_search;
    using ::boost::graph::astar_search_tree;
    using ::boost::graph::astar_search_no_init;
    using ::boost::graph::astar_search_no_init_tree;
    using ::boost::graph::astar_heuristic;
    using ::boost::graph::astar_visitor;
    using ::boost::graph::make_astar_visitor;
    using ::boost::graph::default_astar_visitor;

  template <typename VertexListGraph,
            typename AStarHeuristic,
            typename P, typename T, typename R>
  inline void
  astar_search
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, const bgl_named_params<P, T, R>& params)
  {
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    astar_search(g, s, h, arg_pack);
  }

  template <typename VertexListGraph,
            typename AStarHeuristic,
            typename P, typename T, typename R>
  inline void
  astar_search_tree
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, const bgl_named_params<P, T, R>& params)
  {
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    astar_search_tree(g, s, h, arg_pack);
  }

  template <typename VertexListGraph,
            typename AStarHeuristic,
            typename P, typename T, typename R>
  inline void
  astar_search_no_init
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, const bgl_named_params<P, T, R>& params)
  {
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    astar_search_no_init(g, s, h, arg_pack);
  }

  template <typename VertexListGraph,
            typename AStarHeuristic,
            typename P, typename T, typename R>
  inline void
  astar_search_no_init_tree
    (const VertexListGraph &g,
     typename graph_traits<VertexListGraph>::vertex_descriptor s,
     AStarHeuristic h, const bgl_named_params<P, T, R>& params)
  {
    typedef bgl_named_params<P, T, R> params_type;
    BOOST_GRAPH_DECLARE_CONVERTED_PARAMETERS(params_type, params)
    astar_search_no_init_tree(g, s, h, arg_pack);
  }
} // namespace boost

#endif // BOOST_GRAPH_ASTAR_SEARCH_HPP
