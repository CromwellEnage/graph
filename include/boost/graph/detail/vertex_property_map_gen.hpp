//============================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#ifndef BOOST_GRAPH_DETAIL_VERTEX_PROPERTY_MAP_GEN_HPP
#define BOOST_GRAPH_DETAIL_VERTEX_PROPERTY_MAP_GEN_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/smart_ptr/scoped_array.hpp>
#include <boost/mpl/if.hpp>
#include <algorithm>

namespace boost { namespace detail {

    template <typename Graph, typename IndexMap, typename Value>
    class vertex_property_map_generator
    {
        const Graph& _g;
        const IndexMap& _index_map;
        boost::scoped_array<Value>& _array_holder;

     public:
        typedef typename mpl::if_<
            is_vertex_list_graph<Graph>,
            boost::iterator_property_map<Value*, IndexMap>,
            boost::vector_property_map<Value, IndexMap>
        >::type result_type;
        typedef result_type type;

        inline vertex_property_map_generator(
            const Graph& g,
            const IndexMap& index_map,
            boost::scoped_array<Value>& array_holder
        ) : _g(g), _index_map(index_map), _array_holder(array_holder)
        {
        }

     private:
        inline result_type eval(mpl::true_) const
        {
            this->_array_holder.reset(new Value[num_vertices(this->_g)]);
            std::fill(
                this->_array_holder.get(),
                this->_array_holder.get() + num_vertices(this->_g),
                Value()
            );
            return make_iterator_property_map(
                this->_array_holder.get(),
                this->_index_map
            );
        }

        inline result_type eval(mpl::false_) const
        {
            return boost::make_vector_property_map<Value>(this->_index_map);
        }

     public:
        inline result_type operator()() const
        {
            return this->eval(is_vertex_list_graph<Graph>());
        }
    };
}}

#endif  // include guard

