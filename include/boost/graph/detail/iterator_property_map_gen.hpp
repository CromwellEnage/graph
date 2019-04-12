//============================================================================
// Copyright 2019 Cromwell D. Enage
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#ifndef BOOST_GRAPH_DETAIL_ITERATOR_PROPERTY_MAP_GEN_HPP
#define BOOST_GRAPH_DETAIL_ITERATOR_PROPERTY_MAP_GEN_HPP

#include <boost/property_map/property_map.hpp>
#include <iterator>

namespace boost { namespace detail {

    template <
        typename RAIter,
        typename OffsetMap,
        typename T = typename std::iterator_traits<RAIter>::value_type,
        typename R = typename std::iterator_traits<RAIter>::reference
    >
    class iterator_property_map_generator
    {
        RAIter _itr;
        OffsetMap _offset_map;

     public:
        typedef boost::iterator_property_map<
            RAIter, OffsetMap, T, R
        > result_type;
        typedef result_type type;

        inline iterator_property_map_generator(RAIter itr, OffsetMap offset_map)
          : _itr(itr), _offset_map(offset_map)
        {
        }

        inline result_type operator()() const
        {
            return result_type(this->_itr, this->_offset_map);
        }
    };

    template <typename RAIter, typename OffsetMap>
    inline iterator_property_map_generator<RAIter, OffsetMap>
    make_iterator_property_map_generator(RAIter itr, OffsetMap offset_map)
    {
        return iterator_property_map_generator<RAIter, OffsetMap>(
            itr, offset_map
        );
    }
}}

#endif  // include guard

