//=======================================================================
// Copyright 2013 Cromwell D. Enage
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#ifndef BOOST_GRAPH_DETAIL_TRAITS_HPP
#define BOOST_GRAPH_DETAIL_TRAITS_HPP

#include <boost/config.hpp>

#if defined(BOOST_NO_CXX11_NULLPTR)
#define BOOST_GRAPH_DETAIL_NULLPTR 0
#else
#define BOOST_GRAPH_DETAIL_NULLPTR nullptr
#endif

namespace boost { namespace detail {
    typedef char graph_yes_tag;
    typedef char (&graph_no_tag)[2];
}}

#include <boost/mpl/apply.hpp>
#include <boost/type_traits/declval.hpp>

#if defined(BOOST_NO_CXX11_DECLTYPE)
#include <boost/typeof/typeof.hpp>
#endif

namespace boost { namespace detail {

    template <typename T, typename ResultPlaceholderExpr>
    struct is_logically_negatable_impl
        : mpl::apply1<
            ResultPlaceholderExpr,
#if defined(BOOST_NO_CXX11_DECLTYPE)
            BOOST_TYPEOF_KEYWORD(!boost::declval<T>())
#else
            decltype(!boost::declval<T>())
#endif
        >::type
    {
    };

    template <
        typename T
      , typename FirstArgument
      , typename SecondArgument
      , typename ResultPlaceholderExpr
    >
    struct is_binary_function_impl
        : mpl::apply1<
            ResultPlaceholderExpr,
#if defined(BOOST_NO_CXX11_DECLTYPE)
            BOOST_TYPEOF_KEYWORD((
                boost::declval<T>()(
                    boost::declval<FirstArgument>(),
                    boost::declval<SecondArgument>()
                )
            ))
#else
            decltype(
                boost::declval<T>()(
                    boost::declval<FirstArgument>(),
                    boost::declval<SecondArgument>()
                )
            )
#endif
        >::type
    {
    };
}}

#include <boost/mpl/bool.hpp>
#include <boost/mpl/if.hpp>
#include <boost/type_traits/add_pointer.hpp>

namespace boost { namespace detail {

    template <typename T, typename FirstArg, typename SecondArg>
    class is_binary_func
    {
        template <typename B, typename A1, typename A2>
        static graph_yes_tag
            _check(
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD((
                        boost::declval<B>()(
                            boost::declval<A1>(),
                            boost::declval<A2>()
                        )
                    ))
#else
                    decltype(
                        boost::declval<B>()(
                            boost::declval<A1>(),
                            boost::declval<A2>()
                        )
                    )
#endif
                >::type
            );

        template <typename B, typename A1, typename A2>
        static graph_no_tag _check(...);

     public:
        typedef mpl::bool_<
            sizeof(
                is_binary_func<T,FirstArg,SecondArg>::BOOST_NESTED_TEMPLATE
                _check<T,FirstArg,SecondArg>(BOOST_GRAPH_DETAIL_NULLPTR)
            ) == sizeof(graph_yes_tag)
        > type;
    };
}}

#include <boost/mpl/if.hpp>

namespace boost { namespace detail {

    template <typename T, typename ResultPlaceholderExpr>
    class is_logically_negatable_expr
    {
        template <typename B>
        static graph_yes_tag
            _check(
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD(!boost::declval<B>())
#else
                    decltype(!boost::declval<B>())
#endif
                >::type
            );

        template <typename B>
        static graph_no_tag _check(...);

     public:
        typedef typename mpl::if_c<
            sizeof(
                is_logically_negatable_expr<T,ResultPlaceholderExpr>
                ::BOOST_NESTED_TEMPLATE _check<T>(BOOST_GRAPH_DETAIL_NULLPTR)
            ) == sizeof(graph_yes_tag),
            is_logically_negatable_impl<T,ResultPlaceholderExpr>,
            mpl::false_
        >::type type;
    };

    template <
        typename T,
        typename FirstArgument,
        typename SecondArgument,
        typename ResultPlaceholderExpr
    >
    struct is_binary_function
        : mpl::if_<
            typename is_binary_func<T,FirstArgument,SecondArgument>::type,
            is_binary_function_impl<
                T,
                FirstArgument,
                SecondArgument,
                ResultPlaceholderExpr
            >,
            mpl::false_
        >::type
    {
    };
}}

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace boost { namespace detail {

    template <typename T, typename ResultPlaceholderExpr>
    struct is_logically_negatable
        : is_logically_negatable_expr<
            typename boost::add_const<
                typename boost::remove_reference<T>::type
            >::type,
            ResultPlaceholderExpr
        >::type
    {
    };
}}

#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/remove_const.hpp>

namespace boost { namespace detail {

    template <typename T>
    struct is_boolean_expression
        : mpl::if_<
            boost::is_convertible<
                typename boost::remove_const<
                    typename boost::remove_reference<T>::type
                >::type,
                bool
            >,
            mpl::true_,
            is_logically_negatable<T>
        >::type
    {
    };
}}

#include <boost/mpl/has_xxx.hpp>

namespace boost { namespace detail {

    BOOST_MPL_HAS_XXX_TRAIT_DEF(size_type)
    BOOST_MPL_HAS_XXX_TRAIT_DEF(container_type)
    BOOST_MPL_HAS_XXX_TRAIT_DEF(const_reference)
}}

#include <boost/property_map/property_map.hpp>
#include <boost/mpl/eval_if.hpp>

namespace boost { namespace detail {

    template <typename T>
    struct is_buffer
        : mpl::eval_if<
            has_value_type<T>,
            mpl::eval_if<
                has_size_type<T>,
                mpl::eval_if<
                    has_container_type<T>,
                    mpl::if_<
                        has_reference<T>,
                        has_const_reference<T>,
                        mpl::false_
                    >,
                    mpl::false_
                >,
                mpl::false_
            >,
            mpl::false_
        >::type
    { };
}}

#include <boost/type_traits/is_same.hpp>

namespace boost { namespace detail {

    template <typename T>
    struct is_predecessor_map_impl
        : mpl::if_<
            boost::is_same<
                typename property_traits<T>::key_type,
                typename property_traits<T>::value_type
            >,
            mpl::true_,
            mpl::false_
        >::type
    { };
}}

#include <boost/graph/graph_traits.hpp>

namespace boost { namespace detail {

    template <typename T, typename G>
    struct is_vertex_property_map_of_graph_impl
        : mpl::if_<
            boost::is_same<
                typename property_traits<T>::key_type,
                typename graph_traits<T>::vertex_descriptor
            >,
            mpl::true_,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_vertex_property_map_of_graph
        : mpl::eval_if<
            is_property_map<T>,
            mpl::if_<
                is_graph<G>,
                is_vertex_property_map_of_graph_impl<T,G>,
                mpl::false_
            >,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_vertex_predecessor_map_of_graph
        : mpl::if_<
            is_vertex_property_map_of_graph<T,G>,
            is_predecessor_map_impl<T>,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_edge_property_map_of_graph_impl
        : mpl::if_<
            boost::is_same<
                typename property_traits<T>::key_type,
                typename graph_traits<T>::edge_descriptor
            >,
            mpl::true_,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_edge_property_map_of_graph
        : mpl::eval_if<
            is_property_map<T>,
            mpl::if_<
                is_graph<G>,
                is_edge_property_map_of_graph_impl<T,G>,
                mpl::false_
            >,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_edge_predecessor_map_of_graph
        : mpl::if_<
            is_edge_property_map_of_graph<T,G>,
            is_predecessor_map_impl<T>,
            mpl::false_
        >::type
    { };
}}

#include <boost/graph/properties.hpp>

namespace boost { namespace detail {

    template <typename T>
    struct is_color_map_impl
        : mpl::if_<
            // This is required by the ColorValue concept.
            boost::is_same<
                typename property_traits<T>::value_type,
#if defined(BOOST_NO_CXX11_DECLTYPE)
                BOOST_TYPEOF_KEYWORD(
                    color_traits<
                        typename property_traits<T>::value_type
                    >::white()
                )
#else
                decltype(
                    color_traits<
                        typename property_traits<T>::value_type
                    >::white()
                )
#endif
            >,
            mpl::true_,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_vertex_color_map_of_graph
        : mpl::if_<
            is_vertex_property_map_of_graph<T,G>,
            is_color_map_impl<T>,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_edge_color_map_of_graph
        : mpl::if_<
            is_edge_property_map_of_graph<T,G>,
            is_color_map_impl<T>,
            mpl::false_
        >::type
    { };
}}

#include <boost/type_traits/is_integral.hpp>

namespace boost { namespace detail {

    template <typename T>
    struct is_index_map_impl
        : mpl::eval_if<
            // Ensure that color maps are not mistaken for index maps during
            // type deduction. -- Cromwell D. Enage
            is_color_map_impl<T>,
            mpl::false_,
            mpl::if_<
                boost::is_integral<typename property_traits<T>::value_type>,
                mpl::true_,
                mpl::false_
            >
        >::type
    { };

    template <typename T, typename G>
    struct is_vertex_index_map_of_graph
        : mpl::if_<
            is_vertex_property_map_of_graph<T,G>,
            is_index_map_impl<T>,
            mpl::false_
        >::type
    { };

    template <typename T, typename G>
    struct is_edge_index_map_of_graph
        : mpl::if_<
            is_edge_property_map_of_graph<T,G>,
            is_index_map_impl<T>,
            mpl::false_
        >::type
    { };
}}

#include <boost/tti/has_member_function.hpp>

namespace boost { namespace detail {

    BOOST_TTI_HAS_MEMBER_FUNCTION(initialize_vertex)
    BOOST_TTI_HAS_MEMBER_FUNCTION(start_vertex)
    BOOST_TTI_HAS_MEMBER_FUNCTION(discover_vertex)
    BOOST_TTI_HAS_MEMBER_FUNCTION(examine_vertex)
    BOOST_TTI_HAS_MEMBER_FUNCTION(examine_edge)
    BOOST_TTI_HAS_MEMBER_FUNCTION(tree_edge)
    BOOST_TTI_HAS_MEMBER_FUNCTION(back_edge)
    BOOST_TTI_HAS_MEMBER_FUNCTION(forward_or_cross_edge)
    BOOST_TTI_HAS_MEMBER_FUNCTION(non_tree_edge)
    BOOST_TTI_HAS_MEMBER_FUNCTION(gray_target)
    BOOST_TTI_HAS_MEMBER_FUNCTION(black_target)
    BOOST_TTI_HAS_MEMBER_FUNCTION(finish_vertex)
    BOOST_TTI_HAS_MEMBER_FUNCTION(finish_edge)
}}

#endif  // include guard

