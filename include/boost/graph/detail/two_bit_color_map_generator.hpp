//============================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#ifndef BOOST_GRAPH_DETAIL_TWO_BIT_COLOR_MAP_GENERATOR_HPP
#define BOOST_GRAPH_DETAIL_TWO_BIT_COLOR_MAP_GENERATOR_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/two_bit_color_map.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/mpl/if.hpp>

namespace boost { namespace detail {

    template <typename Graph, typename IndexMap>
    class two_bit_color_map_generator
    {
        const Graph& _g;
        const IndexMap& _index_map;

     public:
        typedef typename mpl::if_<
            is_vertex_list_graph<Graph>,
            boost::two_bit_color_map<IndexMap>,
            boost::vector_property_map<boost::two_bit_color_type, IndexMap>
        >::type result_type;
        typedef result_type type;

        inline two_bit_color_map_generator(
            const Graph& g,
            const IndexMap& index_map
        ) : _g(g), _index_map(index_map)
        {
        }

     private:
        inline result_type eval(mpl::true_) const
        {
            typename graph_traits<
                Graph
            >::vertices_size_type nv = num_vertices(this->_g);
            return boost::two_bit_color_map<IndexMap>(nv, this->_index_map);
        }

        inline result_type eval(mpl::false_) const
        {
            return boost::make_vector_property_map<
                boost::two_bit_color_type
            >(this->_index_map);
        }

     public:
        inline result_type operator()() const
        {
            return this->eval(is_vertex_list_graph<Graph>());
        }
    };
}}

#endif  // include guard

