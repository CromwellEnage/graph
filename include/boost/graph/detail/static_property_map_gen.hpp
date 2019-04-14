//============================================================================
// Copyright 2019 Cromwell D. Enage
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#ifndef BOOST_GRAPH_DETAIL_STATIC_PROPERTY_MAP_GEN_HPP
#define BOOST_GRAPH_DETAIL_STATIC_PROPERTY_MAP_GEN_HPP

#include <boost/property_map/property_map.hpp>

namespace boost { namespace detail {

    template <typename Value, typename Key = void>
    class static_property_map_generator
    {
        Value _value;

     public:
        typedef boost::static_property_map<Value,Key> result_type;
        typedef result_type type;

        inline explicit static_property_map_generator(Value v) : _value(v)
        {
        }

        inline result_type operator()() const
        {
            return result_type(this->_value);
        }
    };
}}

#endif  // include guard

