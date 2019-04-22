//============================================================================
// Copyright 2009 Trustees of Indiana University.
// Authors: Michael Hansen, Andrew Lumsdaine
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#ifndef BOOST_GRAPH_DETAIL_FUNCTIONAL_HPP
#define BOOST_GRAPH_DETAIL_FUNCTIONAL_HPP

namespace boost {

    // Binary function object that returns true if the values for item1
    // in property_map1 and item2 in property_map2 are equivalent.
    template <typename PropertyMapFirst, typename PropertyMapSecond>
    class property_map_equivalent
    {
        const PropertyMapFirst m_property_map1;
        const PropertyMapSecond m_property_map2;

    public:
        typedef bool result_type;

        inline property_map_equivalent(
            const PropertyMapFirst property_map1,
            const PropertyMapSecond property_map2
        ) : m_property_map1(property_map1), m_property_map2(property_map2)
        {
        }

        template <typename ItemFirst, typename ItemSecond>
        inline bool
        operator()(const ItemFirst item1, const ItemSecond item2) const
        {
            return (
                get(this->m_property_map1, item1) ==
                get(this->m_property_map2, item2)
            );
        }
    };

    // Returns a property_map_equivalent object that compares the values
    // of property_map1 and property_map2.
    template <typename PropertyMapFirst, typename PropertyMapSecond>
    property_map_equivalent<PropertyMapFirst,PropertyMapSecond>
    make_property_map_equivalent(
        const PropertyMapFirst property_map1,
        const PropertyMapSecond property_map2
    )
    {
        return property_map_equivalent<PropertyMapFirst,PropertyMapSecond>(
            property_map1, property_map2
        );
    }

    // Binary function object that always returns true.  Used when
    // vertices or edges are always equivalent (i.e. have no labels).
    struct always_equivalent
    {
        typedef bool result_type;

        template <typename ItemFirst, typename ItemSecond>
        inline bool operator()(const ItemFirst&, const ItemSecond&) const
        {
            return true;
        }
    };
}

#endif  // include guard

