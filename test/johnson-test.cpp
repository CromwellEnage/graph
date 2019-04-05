//============================================================================
// Copyright 2002 Indiana University.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//============================================================================

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/johnson_all_pairs_shortest.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/core/lightweight_test.hpp>
#include <vector>

int main()
{
    using namespace boost;
    typedef adjacency_list<vecS, vecS, undirectedS, no_property,
        property< edge_weight_t, int, 
        property< edge_weight2_t, int > >
    > Graph;
    const int V = 10;
    typedef std::pair < int, int >Edge;
    Edge edge_array[] ={
        Edge(0,1), Edge(1,2), Edge(2,3), Edge(1,4), Edge(2,5), Edge(3,6),
        Edge(4,5), Edge(5,6), Edge(4,7), Edge(5,8), Edge(6,9), Edge(7,8),
        Edge(8,9)
    };

    const std::size_t E = sizeof(edge_array) / sizeof(Edge);

    Graph g(edge_array, edge_array + E, V);

    property_map < Graph, edge_weight_t >::type w = get(edge_weight, g);
    int weights[] = { 99, 12, 12, 4, 99, 12, 4, 99, 2, 4, 2, 99, 12  };
    int *wp = weights;

    graph_traits < Graph >::edge_iterator e, e_end;
    for (boost::tie(e, e_end) = edges(g); e != e_end; ++e)
        w[*e] = *wp++;

    std::vector < int >d(V, (std::numeric_limits < int >::max)());
    int D[V][V];
    johnson_all_pairs_shortest_paths(
        g,
        D,
        boost::graph::keywords::_distance_map =
        boost::make_iterator_property_map(
            d.begin(),
            get(boost::vertex_index, g)
        )
    );

/* Expected Output
         0    1    2    3    4    5    6    7    8    9
    0    0   99  111  123  103  107  125  105  111  123
    1   99    0   12   24    4    8   26    6   12   24
    2  111   12    0   12   16   20   24   18   24   26
    3  123   24   12    0   28   30   12   30   26   14
    4  103    4   16   28    0    4   22    2    8   20
    5  107    8   20   30    4    0   18    6    4   16
    6  125   26   24   12   22   18    0   24   14    2
    7  105    6   18   30    2    6   24    0   10   22
    8  111   12   24   26    8    4   14   10    0   12
    9  123   24   26   14   20   16    2   22   12    0
*/
    for (int i = 0; i < 10; ++i)
    {
        BOOST_TEST_EQ(0, D[i][i]);

        for (int j = i + 1; j < 10; ++j)
        {
            BOOST_TEST_EQ(D[i][j], D[j][i]);
        }
    }

    BOOST_TEST_EQ( 99, D[0][1]);
    BOOST_TEST_EQ(111, D[0][2]);
    BOOST_TEST_EQ(123, D[0][3]);
    BOOST_TEST_EQ(103, D[0][4]);
    BOOST_TEST_EQ(107, D[0][5]);
    BOOST_TEST_EQ(125, D[0][6]);
    BOOST_TEST_EQ(105, D[0][7]);
    BOOST_TEST_EQ(111, D[0][8]);
    BOOST_TEST_EQ(123, D[0][9]);
    BOOST_TEST_EQ( 12, D[1][2]);
    BOOST_TEST_EQ( 24, D[1][3]);
    BOOST_TEST_EQ(  4, D[1][4]);
    BOOST_TEST_EQ(  8, D[1][5]);
    BOOST_TEST_EQ( 26, D[1][6]);
    BOOST_TEST_EQ(  6, D[1][7]);
    BOOST_TEST_EQ( 12, D[1][8]);
    BOOST_TEST_EQ( 24, D[1][9]);
    BOOST_TEST_EQ( 12, D[2][3]);
    BOOST_TEST_EQ( 16, D[2][4]);
    BOOST_TEST_EQ( 20, D[2][5]);
    BOOST_TEST_EQ( 24, D[2][6]);
    BOOST_TEST_EQ( 18, D[2][7]);
    BOOST_TEST_EQ( 24, D[2][8]);
    BOOST_TEST_EQ( 26, D[2][9]);
    BOOST_TEST_EQ( 28, D[3][4]);
    BOOST_TEST_EQ( 30, D[3][5]);
    BOOST_TEST_EQ( 12, D[3][6]);
    BOOST_TEST_EQ( 30, D[3][7]);
    BOOST_TEST_EQ( 26, D[3][8]);
    BOOST_TEST_EQ( 14, D[3][9]);
    BOOST_TEST_EQ(  4, D[4][5]);
    BOOST_TEST_EQ( 22, D[4][6]);
    BOOST_TEST_EQ(  2, D[4][7]);
    BOOST_TEST_EQ(  8, D[4][8]);
    BOOST_TEST_EQ( 20, D[4][9]);
    BOOST_TEST_EQ( 18, D[5][6]);
    BOOST_TEST_EQ(  6, D[5][7]);
    BOOST_TEST_EQ(  4, D[5][8]);
    BOOST_TEST_EQ( 16, D[5][9]);
    BOOST_TEST_EQ( 24, D[6][7]);
    BOOST_TEST_EQ( 14, D[6][8]);
    BOOST_TEST_EQ(  2, D[6][9]);
    BOOST_TEST_EQ( 10, D[7][8]);
    BOOST_TEST_EQ( 22, D[7][9]);
    BOOST_TEST_EQ( 12, D[8][9]);
    return boost::report_errors();
}
