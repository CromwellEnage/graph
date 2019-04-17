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

#ifndef BOOST_GRAPH_DETAIL_OUT_DEGREE_PROPERTY_MAP_HPP
#define BOOST_GRAPH_DETAIL_OUT_DEGREE_PROPERTY_MAP_HPP

#include <boost/graph/graph_traits.hpp>
#include <boost/property_map/property_map.hpp>

namespace boost {

    template <typename Graph>
    class out_degree_property_map
        : public put_get_helper<
            typename graph_traits<Graph>::degree_size_type,
            out_degree_property_map<Graph>
        >                  
    {
        const Graph& m_g;

    public:
        typedef typename graph_traits<Graph>::vertex_descriptor key_type;
        typedef typename graph_traits<Graph>::degree_size_type value_type;
        typedef value_type reference;
        typedef readable_property_map_tag category;

        inline out_degree_property_map(const Graph& g) : m_g(g)
        {
        }

        inline value_type operator[](const key_type& v) const
        {
            return out_degree(v, this->m_g);
        }
    };

    template <typename Graph>
    inline out_degree_property_map<Graph>
    make_out_degree_map(const Graph& g)
    {
        return out_degree_property_map<Graph>(g);
    }
} // namespace boost

namespace boost { namespace detail {

    template <typename Graph>
    class out_degree_property_map_generator
    {
        const Graph& _g;

     public:
        typedef out_degree_property_map<Graph> result_type;
        typedef result_type type;

        inline explicit out_degree_property_map_generator(const Graph& g)
          : _g(g)
        {
        }

        inline result_type operator()() const
        {
            return result_type(this->_g);
        }
    };

    template <typename Graph>
    inline out_degree_property_map_generator<Graph>
    make_out_degree_map_generator(const Graph& g)
    {
        return out_degree_property_map_generator<Graph>(g);
    }
}}

#endif  // include guard

