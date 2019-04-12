//============================================================================
// Copyright 2019 Cromwell D. Enage
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#ifndef BOOST_GRAPH_DETAIL_SHARED_ARRAY_PROPERTY_MAP_GEN_HPP
#define BOOST_GRAPH_DETAIL_SHARED_ARRAY_PROPERTY_MAP_GEN_HPP

#include <boost/property_map/shared_array_property_map.hpp>

namespace boost { namespace detail {

    template <typename Graph, typename IndexMap, typename Value>
    class shared_array_property_map_generator
    {
        const Graph& _g;
        const IndexMap& _index_map;
        Value _zero_value;

     public:
        typedef boost::shared_array_property_map<Value, IndexMap> result_type;
        typedef result_type type;

        inline shared_array_property_map_generator(
            const Graph& g,
            const IndexMap& index_map,
            const Value& zero_value
        ) : _g(g), _index_map(index_map), _zero_value(zero_value)
        {
        }

        inline result_type operator()() const
        {
            return make_shared_array_property_map(
                num_vertices(this->_g),
                this->_zero_value,
                this->_index_map
            );
        }
    };

    template <typename Graph, typename IndexMap, typename Value>
    inline shared_array_property_map_generator<Graph, IndexMap, Value>
    make_shared_array_property_map_generator(
        const Graph& g,
        const IndexMap& index_map,
        const Value& zero_value
    )
    {
        return shared_array_property_map_generator<Graph, IndexMap, Value>(
            g, index_map, zero_value
        );
    }
}}

#endif  // include guard

