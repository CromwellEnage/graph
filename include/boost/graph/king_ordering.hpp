//============================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Copyright 2004, 2005 Trustees of Indiana University
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek,
//          Doug Gregor, D. Kevin McGrath
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================
#ifndef BOOST_GRAPH_KING_HPP
#define BOOST_GRAPH_KING_HPP

#include <boost/graph/breadth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <deque>
#include <algorithm>

/*
King Algorithm for matrix reordering
*/

namespace boost { namespace detail {

    template <
        typename OutputIterator, typename Buffer, typename Compare,
        typename PseudoDegreeMap, typename VecMap, typename VertexIndexMap
    >
    class bfs_king_visitor : public default_bfs_visitor
    {
    public:
        bfs_king_visitor(
            OutputIterator& iter, Buffer& b, Compare compare,
            PseudoDegreeMap deg, std::vector<int> loc, VecMap color,
            VertexIndexMap vertices
        ) : default_bfs_visitor(), permutation(iter), Qref(b), degree(deg),
            comp(compare), Qlocation(loc), colors(color), vertex_map(vertices)
        {
        }

        template <typename Vertex, typename Graph>
        void finish_vertex(Vertex, Graph& g)
        {
            typename graph_traits<Graph>::out_edge_iterator ei, ei_end;
            Vertex v, w;

            typedef typename std::deque<
                Vertex
            >::reverse_iterator reverse_iterator;

            reverse_iterator rend = this->Qref.rend() - this->index_begin;
            reverse_iterator rbegin = this->Qref.rbegin();

            // heap the vertices already there
            std::make_heap(
                rbegin, rend, boost::bind<bool>(this->comp, _2, _1)
            );

            unsigned i = 0;

            for (i = this->index_begin; i != this->Qref.size(); ++i)
            {
                this->colors[get(this->vertex_map, this->Qref[i])] = 1;
                this->Qlocation[get(this->vertex_map, this->Qref[i])] = i;
            }

            i = 0;

            for (; rbegin != rend; --rend)
            {
                this->percolate_down<Vertex>(i);
                w = this->Qref[this->index_begin + i];

                for (
                    boost::tie(ei, ei_end) = out_edges(w, g);
                    ei != ei_end;
                    ++ei
                )
                {
                    v = target(*ei, g);
                    put(this->degree, v, get(this->degree, v) - 1);

                    if (this->colors[get(this->vertex_map, v)] == 1)
                    {
                        this->percolate_up<
                            Vertex
                        >(get(this->vertex_map, v), i);
                    }
                }

                this->colors[get(this->vertex_map, w)] = 0;
                ++i;
            }
        }

        template <typename Vertex, typename Graph>
        inline void examine_vertex(Vertex u, const Graph&)
        {
            *this->permutation++ = u;
            this->index_begin = this->Qref.size();
        }

    protected:
        // this function replaces pop_heap, and tracks state information
        template <typename Vertex>
        void percolate_down(int offset)
        {
            int heap_last = this->index_begin + offset;
            int heap_first = this->Qref.size() - 1;

            // pop_heap functionality:
            // swap first, last
            std::swap(this->Qref[heap_last], this->Qref[heap_first]);

            // swap in the location queue
            std::swap(
                this->Qlocation[heap_first], this->Qlocation[heap_last]
            );

            // set drifter, children
            int drifter = heap_first;
            int drifter_heap = this->Qref.size() - drifter;

            int right_child_heap = drifter_heap * 2 + 1;
            int right_child = this->Qref.size() - right_child_heap;

            int left_child_heap = drifter_heap * 2;
            int left_child = this->Qref.size() - left_child_heap;

            // check that we are staying in the heap
            bool valid = (right_child < heap_last) ? false : true;

            for (
                // pick smallest child of drifter, and
                // keep in mind there might only be left child
                int smallest_child = (
                    (
                        valid && (
                            get(this->degree, this->Qref[left_child]) >
                            get(this->degree, this->Qref[right_child])
                        )
                    ) ? right_child : left_child
                );
                valid && (smallest_child < heap_last) && this->comp(
                    this->Qref[drifter], this->Qref[smallest_child]
                );
                smallest_child = (
                    valid && (
                        get(this->degree, this->Qref[left_child]) >
                        get(this->degree, this->Qref[right_child])
                    )
                ) ? right_child : left_child
            )
            {
                // if smallest child smaller than drifter, swap them
                std::swap(this->Qref[smallest_child], this->Qref[drifter]);
                std::swap(
                    this->Qlocation[drifter], this->Qlocation[smallest_child]
                );

                // update the values, run again, as necessary
                drifter = smallest_child;
                drifter_heap = this->Qref.size() - drifter;

                right_child_heap = drifter_heap * 2 + 1;
                right_child = this->Qref.size() - right_child_heap;

                left_child_heap = drifter_heap * 2;
                left_child = this->Qref.size() - left_child_heap;

                valid = (right_child < heap_last) ? false : true;
            }
        }

        // this is like percolate down, but we always compare against the
        // parent, as there is only a single choice
        template <typename Vertex>
        void percolate_up(int vertex, int offset)
        {
            int child_location = this->Qlocation[vertex];
            int heap_child_location = this->Qref.size() - child_location;
            int heap_parent_location = static_cast<
                int
            >(heap_child_location / 2);
            unsigned parent_location = this->Qref.size() - heap_parent_location;

            for (
                bool valid = (
                    (heap_parent_location != 0) && (
                        child_location > this->index_begin + offset
                    ) && (parent_location < this->Qref.size())
                );
                valid && this->comp(
                    this->Qref[child_location], this->Qref[parent_location]
                );
                valid = (
                    (heap_parent_location != 0) && (
                        child_location > this->index_begin + offset
                    )
                )
            )
            {
                // swap in the heap
                std::swap(
                    this->Qref[child_location], this->Qref[parent_location]
                );

                // swap in the location queue
                std::swap(
                    this->Qlocation[child_location],
                    this->Qlocation[parent_location]
                );

                child_location = parent_location;
                heap_child_location = heap_parent_location;
                heap_parent_location = static_cast<
                    int
                >(heap_child_location / 2);
                parent_location = this->Qref.size() - heap_parent_location;
            }
        }

        OutputIterator& permutation;
        int index_begin;
        Buffer& Qref;
        PseudoDegreeMap degree;
        Compare comp;
        std::vector<int> Qlocation;
        VecMap colors;
        VertexIndexMap vertex_map;
    };
}} // namespace boost::detail  

#include <boost/graph/named_function_params.hpp>
#include <boost/graph/detail/out_degree_property_map.hpp>
#include <boost/graph/detail/sparse_ordering.hpp>
#include <boost/parameter/is_argument_pack.hpp>
#include <boost/parameter/value_type.hpp>
#include <boost/core/enable_if.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename OutputIterator, typename Args>
    typename boost::enable_if<
        parameter::is_argument_pack<Args>,
        OutputIterator
    >::type
    king_ordering(const Graph& graph, OutputIterator result, const Args& args)
    {
        if (boost::graph::has_no_vertices(graph)) return result;

        typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
        Vertex root_vertex = args[
            boost::graph::keywords::_root_vertex |
            graph_traits<Graph>::null_vertex()
        ];
        typedef typename boost::detail::override_const_property_result<
            Args,
            boost::graph::keywords::tag::vertex_index_map,
            vertex_index_t,
            Graph
        >::type VertexIndexMap;
        VertexIndexMap vertex_index_map =
        boost::detail::override_const_property(
            args,
            boost::graph::keywords::_vertex_index_map,
            graph,
            vertex_index
        );
        typedef typename boost::detail::map_maker<
            Graph,
            Args,
            boost::graph::keywords::tag::color_map,
            boost::default_color_type
        >::map_type ColorMap;
        ColorMap color_map = boost::detail::make_color_map_from_arg_pack(
            graph,
            args
        );
        typedef typename boost::remove_const<
            typename parameter::lazy_value_type<
                Args,
                boost::graph::keywords::tag::degree_map,
                boost::detail::out_degree_property_map_generator<Graph>
            >::type
        >::type DegreeMap;
        DegreeMap degree_map = args[
            boost::graph::keywords::_degree_map ||
            boost::detail::make_out_degree_map_generator(graph)
        ];
        typedef typename boost::remove_const<
            typename parameter::value_type<
                Args,
                boost::graph::keywords::tag::buffer,
                std::deque<Vertex>
            >::type
        >::type Buffer;
        Buffer buffer = args[
            boost::graph::keywords::_buffer ||
            boost::sparse::make_ordering_default_queue_t<
                std::deque<Vertex>
            >(graph, root_vertex, color_map, degree_map)
        ];
        typedef boost::sparse::sparse_ordering_queue<Vertex> Queue;
        Queue Q;
        typedef color_traits<
            typename property_traits<ColorMap>::value_type
        > Color;
        typedef typename property_traits<DegreeMap>::value_type Degree;
        std::vector<Degree> ps_deg_vec(num_vertices(graph));
        typedef iterator_property_map<
            typename std::vector<Degree>::iterator, VertexIndexMap,
            Degree, Degree&
        > PseudoDegreeMap;
        PseudoDegreeMap pseudo_degree(ps_deg_vec.begin(), vertex_index_map);
        typename graph_traits<Graph>::vertex_iterator ui, ui_end;

        // Copy degree to pseudo_degree
        // initialize the color map
        for (boost::tie(ui, ui_end) = vertices(graph); ui != ui_end; ++ui)
        {
            put(pseudo_degree, *ui, get(degree_map, *ui));
            put(color_map, *ui, Color::white());
        }

        typedef indirect_cmp<PseudoDegreeMap, std::less<Degree> > Compare;
        typedef typename graph_traits<Graph>::vertices_size_type VSize;

        Compare comp(pseudo_degree);
        std::vector<int> colors(num_vertices(graph));

        for (VSize i = 0; i < num_vertices(graph); ++i)
        {
            colors[i] = 0;
        }

        std::vector<int> loc(num_vertices(graph));

        // create the visitor
        typedef boost::detail::bfs_king_visitor<
            OutputIterator, Queue, Compare, PseudoDegreeMap,
            std::vector<int>, VertexIndexMap
        > Visitor;

        Visitor vis(
            result, Q, comp, pseudo_degree, loc, colors, vertex_index_map
        );
    
        while (!buffer.empty())
        {
            Vertex s = buffer.front();
            buffer.pop_front();

            // call BFS with visitor
            breadth_first_visit(graph, s, Q, vis, color_map);
        }

        return result;
    }
}} // namespace boost::graph  

#include <boost/parameter/compose.hpp>

namespace boost { namespace graph {

    template <typename Graph, typename OutputIterator>
    inline OutputIterator
    king_ordering(const Graph& g, OutputIterator permutation)
    {
        return king_ordering(g, permutation, parameter::compose());
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
    inline typename boost::enable_if< \
        parameter::are_tagged_arguments< \
            TA BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, TA) \
        >, \
        OutputIterator \
    >::type \
    name( \
        const Graph& g, OutputIterator permutation, const TA& ta \
        BOOST_PP_ENUM_TRAILING_BINARY_PARAMS_Z(z, n, const TA, &ta) \
    ) \
    { \
        return name( \
            g, permutation, \
            parameter::compose(ta BOOST_PP_ENUM_TRAILING_PARAMS_Z(z, n, ta)) \
        ); \
    }

#include <boost/preprocessor/repetition/repeat_from_to.hpp>

namespace boost { namespace graph {

BOOST_PP_REPEAT_FROM_TO(1, 6, BOOST_GRAPH_PP_FUNCTION_OVERLOAD, king_ordering)
}} // namespace boost::graph

#include <boost/graph/detail/traits.hpp>

namespace boost { namespace graph {

    template <
        typename Graph, typename OutputIterator, typename VertexIndexMap
    >
    inline typename boost::enable_if<
        boost::detail::is_vertex_to_integer_map_of_graph<
            VertexIndexMap,
            Graph
        >,
        OutputIterator
    >::type
    king_ordering(
        const Graph& g, OutputIterator permutation, VertexIndexMap index_map
    )
    {
        return king_ordering(
            g, permutation,
            boost::graph::keywords::_vertex_index_map = index_map
        );
    }
}} // namespace boost::graph

#include <boost/mpl/bool.hpp>
#include <boost/mpl/if.hpp>
#include <boost/mpl/eval_if.hpp>

namespace boost { namespace graph {

    template <
        typename Graph, typename OutputIterator, typename ColorMap,
        typename DegreeMap, typename VertexIndexMap
    >
    inline typename boost::enable_if<
        typename mpl::eval_if<
            boost::detail::is_vertex_to_integer_map_of_graph<
                VertexIndexMap,
                Graph
            >,
            mpl::if_<
                boost::detail::is_vertex_to_integer_map_of_graph<
                    DegreeMap,
                    Graph
                >,
                boost::detail::is_vertex_color_map_of_graph<ColorMap,Graph>,
                mpl::false_
            >,
            mpl::false_
        >::type,
        OutputIterator
    >::type
    king_ordering(
        const Graph& g, OutputIterator permutation, ColorMap color,
        DegreeMap degree, VertexIndexMap index_map
    )
    {
        return king_ordering(
            g, permutation,
            boost::graph::keywords::_color_map = color,
            boost::graph::keywords::_degree_map = degree,
            boost::graph::keywords::_vertex_index_map = index_map
        );
    }

    template <
        typename Graph, typename OutputIterator, typename ColorMap,
        typename DegreeMap, typename VertexIndexMap
    >
    inline typename boost::enable_if<
        typename mpl::eval_if<
            boost::detail::is_vertex_to_integer_map_of_graph<
                VertexIndexMap,
                Graph
            >,
            mpl::if_<
                boost::detail::is_vertex_to_integer_map_of_graph<
                    DegreeMap,
                    Graph
                >,
                boost::detail::is_vertex_color_map_of_graph<ColorMap,Graph>,
                mpl::false_
            >,
            mpl::false_
        >::type,
        OutputIterator
    >::type
    king_ordering(
        const Graph& g, typename graph_traits<Graph>::vertex_descriptor s,
        OutputIterator permutation, ColorMap color, DegreeMap degree,
        VertexIndexMap index_map
    )
    {
        return king_ordering(
            g, permutation,
            boost::graph::keywords::_root_vertex = s,
            boost::graph::keywords::_color_map = color,
            boost::graph::keywords::_degree_map = degree,
            boost::graph::keywords::_vertex_index_map = index_map
        );
    }

    template <
        typename Graph, typename OutputIterator, typename ColorMap,
        typename DegreeMap, typename VertexIndexMap
    >
    inline typename boost::enable_if<
        typename mpl::eval_if<
            boost::detail::is_vertex_to_integer_map_of_graph<
                VertexIndexMap,
                Graph
            >,
            mpl::if_<
                boost::detail::is_vertex_to_integer_map_of_graph<
                    DegreeMap,
                    Graph
                >,
                boost::detail::is_vertex_color_map_of_graph<ColorMap,Graph>,
                mpl::false_
            >,
            mpl::false_
        >::type,
        OutputIterator
    >::type
    king_ordering(
        const Graph& g,
        std::deque<
            typename graph_traits<Graph>::vertex_descriptor
        > vertex_queue,
        OutputIterator permutation,
        ColorMap color,
        DegreeMap degree,
        VertexIndexMap index_map
    )
    {
        return king_ordering(
            g, permutation,
            boost::graph::keywords::_buffer = vertex_queue,
            boost::graph::keywords::_color_map = color,
            boost::graph::keywords::_degree_map = degree,
            boost::graph::keywords::_vertex_index_map = index_map
        );
    }
}} // namespace boost::graph  

namespace boost {

    using ::boost::graph::king_ordering;
} // namespace boost

#endif // BOOST_GRAPH_KING_HPP

