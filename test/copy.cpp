// Copyright (C) Vladimir Prus 2003.
// Copyright (C) Cromwell D. Enage 2019.
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>

typedef boost::adjacency_list<
    boost::vecS, boost::vecS, boost::directedS,
    boost::property<boost::vertex_rank_t, int>
> G1;
typedef boost::graph_traits<G1>::vertex_descriptor V1;
typedef boost::adjacency_list<
    boost::vecS, boost::setS, boost::directedS,
    boost::property<boost::vertex_index_t, std::size_t>
> G2;
typedef boost::graph_traits<G2>::vertex_descriptor V2;
typedef boost::graph_traits<G2>::vertex_iterator V2Itr;

#include <boost/graph/properties.hpp>

class copier
{
    const G1& _g1;
    G2& _g2;

public:
    copier(const G1& g1, G2& g2) : _g1(g1), _g2(g2)
    {
    }

    void operator()(const V1& v1, const V2& v2) const
    {
        put(
            get(boost::vertex_index, this->_g2), v2, (
                num_vertices(this->_g1) - 1 -
                get(get(boost::vertex_index, this->_g1), v1)
            )
        );
    }
};

#include <boost/graph/copy.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/core/lightweight_test.hpp>

int main()
{
    G1 g1_1, g1_2;
    G2 g2;
    V1 v1_1, v1_2;

    add_edge(v1_1 = add_vertex(g1_1), v1_2 = add_vertex(g1_1), g1_1);
    put(get(boost::vertex_rank, g1_1), v1_1, 4);
    put(get(boost::vertex_rank, g1_1), v1_2, 8);
    add_edge(v1_1 = add_vertex(g1_1), v1_2 = add_vertex(g1_1), g1_1);
    put(get(boost::vertex_rank, g1_1), v1_1, 15);
    put(get(boost::vertex_rank, g1_1), v1_2, 16);
    add_edge(v1_1 = add_vertex(g1_1), v1_2 = add_vertex(g1_1), g1_1);
    put(get(boost::vertex_rank, g1_1), v1_1, 23);
    put(get(boost::vertex_rank, g1_1), v1_2, 42);
    add_edge(vertex(1, g1_1), vertex(4, g1_1), g1_1);
    add_edge(vertex(3, g1_1), vertex(0, g1_1), g1_1);
    add_edge(vertex(5, g1_1), vertex(2, g1_1), g1_1);
    boost::copy_graph(g1_1, g1_2);
    BOOST_TEST_EQ(num_vertices(g1_1), num_vertices(g1_2));
    for (std::size_t i = 0; i < num_vertices(g1_2); ++i)
        BOOST_TEST_EQ(
            get(get(boost::vertex_rank, g1_1), vertex(i, g1_1)),
            get(get(boost::vertex_rank, g1_2), vertex(i, g1_2))
        );
    BOOST_TEST(is_adjacent(g1_2, vertex(0, g1_2), vertex(1, g1_2)));
    BOOST_TEST(is_adjacent(g1_2, vertex(1, g1_2), vertex(4, g1_2)));
    BOOST_TEST(is_adjacent(g1_2, vertex(2, g1_2), vertex(3, g1_2)));
    BOOST_TEST(is_adjacent(g1_2, vertex(3, g1_2), vertex(0, g1_2)));
    BOOST_TEST(is_adjacent(g1_2, vertex(4, g1_2), vertex(5, g1_2)));
    BOOST_TEST(is_adjacent(g1_2, vertex(5, g1_2), vertex(2, g1_2)));
    copier c(g1_1, g2);
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_UNNAMED_ARGUMENTS)
    boost::copy_graph(g1_1, g2, c);
#elif defined(BOOST_GRAPH_CONFIG_CAN_NAME_ARGUMENTS)
    boost::copy_graph(g1_1, g2, boost::graph::keywords::_vertex_copy = c);
#else
    boost::copy_graph(g1_1, g2, boost::vertex_copy(c));
#endif

    V2Itr v2_begin, v2_end;
    boost::tie(v2_begin, v2_end) = vertices(g2);
    V2Itr v2_i = v2_begin;

    for (std::size_t i = num_vertices(g2); v2_i != v2_end; ++v2_i)
    {
        --i;
        BOOST_TEST_EQ(i, get(get(boost::vertex_index, g2), *v2_i));
    }

    v2_i = v2_end = v2_begin; ++v2_end;
    BOOST_TEST(is_adjacent(g2, *v2_i, *v2_end));

    ++v2_i; ++v2_i; ++v2_end; ++v2_end;
    BOOST_TEST(is_adjacent(g2, *v2_i, *v2_end));

    ++v2_i; ++v2_i; ++v2_end; ++v2_end;
    BOOST_TEST(is_adjacent(g2, *v2_i, *v2_end));

    v2_i = v2_end = v2_begin; ++v2_i; ++v2_end; ++v2_end; ++v2_end; ++v2_end;
    BOOST_TEST(is_adjacent(g2, *v2_i, *v2_end));

    v2_end = v2_i = v2_begin; ++v2_i; ++v2_i; ++v2_i;
    BOOST_TEST(is_adjacent(g2, *v2_i, *v2_end));

    ++v2_i; ++v2_i; ++v2_end; ++v2_end;
    BOOST_TEST(is_adjacent(g2, *v2_i, *v2_end));

    return boost::report_errors();
}

