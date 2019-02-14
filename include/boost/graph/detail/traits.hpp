//=======================================================================
// Copyright 2013 Cromwell D. Enage
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================

#ifndef BOOST_GRAPH_DETAIL_TRAITS_HPP
#define BOOST_GRAPH_DETAIL_TRAITS_HPP

// Many of these metafunctions deserve to be in a separate library.  Until
// such a library materializes, however, this header file shall serve as
// their home. -- Cromwell D. Enage

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

#include <boost/type_traits/add_lvalue_reference.hpp>

namespace boost { namespace detail {

    template <typename T>
    typename boost::add_lvalue_reference<T>::type declref() BOOST_NOEXCEPT;
}}

#include <boost/type_traits/add_const.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace boost { namespace detail {

    template <typename T>
    typename boost::add_lvalue_reference<
        typename boost::add_const<
            typename boost::remove_reference<T>::type
        >::type
    >::type declcref() BOOST_NOEXCEPT;
}}

#include <boost/graph/named_function_params.hpp>
#include <boost/mpl/bool.hpp>
#include <boost/mpl/if.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/type_traits/declval.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#if defined(BOOST_NO_CXX11_DECLTYPE)
#include <boost/typeof/typeof.hpp>
#endif

namespace boost { namespace detail {

    template <typename T>
    struct is_logically_negatable_impl
    {
        typedef typename mpl::if_<
            boost::is_convertible<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                BOOST_TYPEOF_TPL(!boost::declval<T>()),
#else
                decltype(!boost::declval<T>()),
#endif
                bool
            >,
            mpl::true_,
            mpl::false_
        >::type type;
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/type_traits/is_same.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename T>
    struct has_const_member_function_size_impl
    {
        typedef typename mpl::if_<
            boost::is_same<
                typename T::size_type,
#if defined(BOOST_NO_CXX11_DECLTYPE)
                BOOST_TYPEOF_TPL(boost::detail::declcref<T>().size())
#else
                decltype(boost::detail::declcref<T>().size())
#endif
            >,
            mpl::true_,
            mpl::false_
        >::type type;
    };

    template <typename T>
    struct has_const_member_function_top_impl
    {
        typedef typename mpl::if_<
            boost::is_same<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                typename T::value_type,
                BOOST_TYPEOF_TPL(boost::detail::declcref<T>().top())
#else
                typename T::value_type const&,
                decltype(boost::detail::declcref<T>().top())
#endif
            >,
            mpl::true_,
            mpl::false_
        >::type type;
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/mpl/apply.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <
        typename T,
        typename FirstArgument,
        typename SecondArgument,
        typename ResultPlaceholderExpr
    >
    struct is_binary_function_impl
    {
        typedef typename mpl::apply1<
            ResultPlaceholderExpr,
#if defined(BOOST_NO_CXX11_DECLTYPE)
            BOOST_TYPEOF_TPL((
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
        >::type type;
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/mpl/vector.hpp>
#include <boost/type_traits/add_pointer.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename T, typename FirstArg, typename SecondArg>
    class is_binary_func
    {
        template <typename B, typename A1, typename A2>
        static graph_yes_tag
            _check(
                mpl::vector<B,A1,A2>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL((
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
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef mpl::bool_<
            sizeof(
                is_binary_func<T,FirstArg,SecondArg>::_check(
                    static_cast<mpl::vector<T,FirstArg,SecondArg>*>(
                        BOOST_GRAPH_DETAIL_NULLPTR
                    )
                )
            ) == sizeof(graph_yes_tag)
        > type;
    };

    template <typename T>
    class is_iterator_impl
    {
        template <typename B>
        static graph_yes_tag
            _check_d(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL(*boost::detail::declref<B>())
#else
                    decltype(*boost::detail::declref<B>())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_d(...);

        template <typename B>
        static graph_yes_tag
            _check_i(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL(++boost::detail::declref<B>())
#else
                    decltype(++boost::detail::declref<B>())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check_i(...);

     public:
        typedef mpl::bool_<
            (
                sizeof(
                    is_iterator_impl<T>::_check_d(
                        static_cast<mpl::vector<T>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    is_iterator_impl<T>::_check_i(
                        static_cast<mpl::vector<T>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            )
        > type;
    };

#if !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
    template <typename T>
    class has_member_function_pop_expr
    {
        template <typename B>
        static graph_yes_tag
            _check(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_KEYWORD(boost::detail::declref<B>().pop())
#else
                    decltype(boost::detail::declref<B>().pop())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef mpl::bool_<
            sizeof(
                has_member_function_pop_expr<T>::_check(
                    static_cast<mpl::vector<T>*>(BOOST_GRAPH_DETAIL_NULLPTR)
                )
            ) == sizeof(graph_yes_tag)
        > type;
    };
#endif  // !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/mpl/eval_if.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <
        typename T,
        typename FirstArgument,
        typename SecondArgument,
        typename ResultPlaceholderExpr
    >
    struct is_binary_function
        : mpl::eval_if<
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

    template <typename T>
    class is_logically_negatable_expr
    {
        template <typename B>
        static graph_yes_tag
            _check(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL(!boost::declval<B>())
#else
                    decltype(!boost::declval<B>())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef typename mpl::eval_if_c<
            sizeof(
                is_logically_negatable_expr<T>::_check(
                    static_cast<mpl::vector<T>*>(BOOST_GRAPH_DETAIL_NULLPTR)
                )
            ) == sizeof(graph_yes_tag),
            is_logically_negatable_impl<T>,
            mpl::false_
        >::type type;
    };

    template <typename T>
    struct is_logically_negatable
        : is_logically_negatable_expr<
            typename boost::add_const<
                typename boost::remove_reference<T>::type
            >::type
        >::type
    {
    };

    template <typename T>
    struct has_member_function_top_impl
    {
        typedef typename mpl::eval_if<
            boost::is_same<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                typename T::value_type,
                BOOST_TYPEOF_TPL(boost::detail::declref<T>().top())
#else
                typename T::value_type&,
                decltype(boost::detail::declref<T>().top())
#endif
            >,
            has_const_member_function_top_impl<T>,
            mpl::false_
        >::type type;
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/type_traits/remove_const.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
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

    template <typename T>
    struct is_iterator
        : is_iterator_impl<
            typename boost::remove_const<
                typename boost::remove_reference<T>::type
            >::type
        >::type
    {
    };

    template <typename T>
    struct has_const_member_function_empty_impl
    {
        typedef is_boolean_expression<
#if defined(BOOST_NO_CXX11_DECLTYPE)
            BOOST_TYPEOF_TPL(boost::detail::declcref<T>().empty())
#else
            decltype(boost::detail::declcref<T>().empty())
#endif
        > type;
    };

    template <typename T>
    class has_const_member_function_empty_expr
    {
        template <typename B>
        static graph_yes_tag
            _check(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL(boost::detail::declcref<B>().empty())
#else
                    decltype(boost::detail::declcref<B>().empty())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef typename mpl::eval_if_c<
            sizeof(
                has_const_member_function_empty_expr<T>::_check(
                    static_cast<mpl::vector<T>*>(BOOST_GRAPH_DETAIL_NULLPTR)
                )
            ) == sizeof(graph_yes_tag),
            has_const_member_function_empty_impl<T>,
            mpl::false_
        >::type type;
    };

    template <typename T>
    struct has_const_member_function_empty
        : has_const_member_function_empty_expr<
            typename boost::remove_const<
                typename boost::remove_reference<T>::type
            >::type
        >::type
    {
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/property_map/property_map.hpp>

namespace boost { namespace detail {

    template <typename G>
    struct choose_dummy_property_map
    {
        typedef dummy_property_map type;

        inline static type call(const G& g)
        {
            return dummy_property_map();
        }
    };

    template <typename T>
    struct is_property_map_with_same_key_and_value_type_impl
        : mpl::if_<
            boost::is_same<
                typename property_traits<T>::key_type,
                typename property_traits<T>::value_type
            >,
            mpl::true_,
            mpl::false_
        >::type
    {
    };
}}

#include <boost/mpl/eval_if.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename T>
    class has_member_function_top_expr
    {
        template <typename B>
        static graph_yes_tag
            _check(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL(boost::detail::declref<B>().top())
#else
                    decltype(boost::detail::declref<B>().top())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef typename mpl::eval_if_c<
            (
                sizeof(
                    has_member_function_top_expr<T>::_check(
                        static_cast<mpl::vector<T>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ) && (
                sizeof(
                    has_member_function_top_expr<T>::_check(
                        static_cast<mpl::vector<T const>*>(
                            BOOST_GRAPH_DETAIL_NULLPTR
                        )
                    )
                ) == sizeof(graph_yes_tag)
            ),
            mpl::eval_if<
                has_value_type<T>,
                has_member_function_top_impl<T>,
                mpl::false_
            >,
            mpl::false_
        >::type type;
    };

    template <typename T>
    struct has_member_function_top
        : has_member_function_top_expr<
            typename boost::remove_const<
                typename boost::remove_reference<T>::type
            >::type
        >::type
    {
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/range/size.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename T>
    class has_const_member_function_size_expr
    {
        template <typename B>
        static graph_yes_tag
            _check(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL(boost::detail::declcref<B>().size())
#else
                    decltype(boost::detail::declcref<B>().size())
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef typename mpl::eval_if_c<
            sizeof(
                has_const_member_function_size_expr<T>::_check(
                    static_cast<mpl::vector<T>*>(BOOST_GRAPH_DETAIL_NULLPTR)
                )
            ) == sizeof(graph_yes_tag),
            mpl::eval_if<
                has_size_type<T>,
                has_const_member_function_size_impl<T>,
                mpl::false_
            >,
            mpl::false_
        >::type type;
    };

    template <typename T>
    struct has_const_member_function_size
        : has_const_member_function_size_expr<
            typename boost::remove_const<
                typename boost::remove_reference<T>::type
            >::type
        >::type
    {
    };

    template <typename T>
    struct is_buffer
        : mpl::eval_if<
#if !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
            typename has_member_function_pop_expr<
                typename boost::remove_const<
                    typename boost::remove_reference<T>::type
                >::type
            >::type,
            mpl::eval_if<
#endif
                has_member_function_top<T>,
                mpl::if_<
                    has_const_member_function_empty<T>,
                    has_const_member_function_size<T>,
                    mpl::false_
                >,
                mpl::false_
#if !defined(BOOST_NO_CXX11_DECLTYPE) || defined(BOOST_TYPEOF_KEYWORD)
            >,
            mpl::false_
#endif
        >::type
    {
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/mpl/has_xxx.hpp>

namespace boost { namespace graph_detail {

    BOOST_MPL_HAS_XXX_TRAIT_DEF(const_reference)
    BOOST_MPL_HAS_XXX_TRAIT_DEF(reference)
    BOOST_MPL_HAS_XXX_TRAIT_DEF(graph_type)
    BOOST_MPL_HAS_XXX_TRAIT_DEF(graph_tag)
}}

#include <boost/range/has_range_iterator.hpp>

namespace boost { namespace detail {

    template <typename T>
    struct has_container_typedefs_impl
        : mpl::eval_if<
            has_value_type<T>,
            mpl::eval_if<
                has_size_type<T>,
                mpl::eval_if<
                    has_range_iterator<T>,
                    mpl::eval_if<
                        has_range_const_iterator<T>,
                        mpl::if_<
                            graph_detail::has_const_reference<T>,
                            graph_detail::has_reference<T>,
                            mpl::false_
                        >,
                        mpl::false_
                    >,
                    mpl::false_
                >,
                mpl::false_
            >,
            mpl::false_
        >::type
    {
    };

    template <typename T>
    struct has_container_typedefs
        : has_container_typedefs_impl<
            typename boost::remove_const<
                typename boost::remove_reference<T>::type
            >::type
        >::type
    {
    };
}}

#include <boost/graph/graph_traits.hpp>

namespace boost { namespace detail {

    template <typename T, typename G>
    struct is_vertex_property_map_of_graph_impl
        : mpl::if_<
            boost::is_same<
                typename graph_traits<G>::vertex_descriptor,
                typename property_traits<T>::key_type
            >,
            mpl::true_,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_vertex_property_map_of_graph
        : mpl::eval_if<
            is_property_map<T>,
            mpl::if_<
                is_bgl_graph<G>,
                is_vertex_property_map_of_graph_impl<T,G>,
                mpl::false_
            >,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_vertex_to_vertex_map_of_graph
        : mpl::if_<
            is_vertex_property_map_of_graph<T,G>,
            is_property_map_with_same_key_and_value_type_impl<T>,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_edge_property_map_of_graph_impl
        : mpl::if_<
            boost::is_same<
                typename property_traits<T>::key_type,
                typename graph_traits<G>::edge_descriptor
            >,
            mpl::true_,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_edge_property_map_of_graph
        : mpl::eval_if<
            is_property_map<T>,
            mpl::if_<
                is_bgl_graph<G>,
                is_edge_property_map_of_graph_impl<T,G>,
                mpl::false_
            >,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_edge_to_edge_map_of_graph
        : mpl::if_<
            is_edge_property_map_of_graph<T,G>,
            is_property_map_with_same_key_and_value_type_impl<T>,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G2>
    struct is_orig_to_copy_vertex_map_impl
        : mpl::if_<
            boost::is_same<
                typename property_traits<T>::value_type,
                typename graph_traits<G2>::vertex_descriptor
            >,
            mpl::true_,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G1, typename G2>
    struct is_orig_to_copy_vertex_map
        : mpl::if_<
            is_vertex_property_map_of_graph<T,G1>,
            is_orig_to_copy_vertex_map_impl<T,G2>,
            mpl::false_
        >::type
    {
    };
}}

#include <boost/graph/properties.hpp>

namespace boost { namespace detail {

    // The primary template specialization will handle the cases of
    // PropertyType == boost::no_property and
    // PropertyType == user-defined bundled property type
    // -- Cromwell D. Enage
    template <typename PropertyType, typename Tag>
    struct property_type_contains : mpl::false_
    {
    };

    template <typename T, typename Tail, typename Tag>
    struct property_type_contains<boost::property<Tag,T,Tail>,Tag>
      : mpl::true_
    {
    };

    template <typename P, typename T, typename Tail, typename Tag>
    struct property_type_contains<boost::property<P,T,Tail>,Tag>
      : property_type_contains<Tail,Tag>
    {
    };

    template <typename G, typename Tag>
    struct is_graph_with_vertex_property_type_impl
        : property_type_contains<typename G::vertex_property_type,Tag>
    {
    };

    template <typename G, typename Tag>
    struct is_graph_with_vertex_property_type
        : mpl::if_<
            detail::has_vertex_property_type<G>,
            is_graph_with_vertex_property_type_impl<G,Tag>,
            mpl::false_
        >::type
    {
    };

    template <typename G>
    struct choose_vertex_index_map
    {
        typedef typename property_map<G,vertex_index_t>::const_type type;

        inline static type call(const G& g)
        {
            return get(vertex_index, g);
        }
    };
}}

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename T>
    struct is_color_map_impl
    {
        typedef typename mpl::if_<
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
        >::type type;
    };

    template <typename T, typename G>
    struct is_vertex_color_map_of_graph
        : mpl::eval_if<
            is_vertex_property_map_of_graph<T,G>,
            is_color_map_impl<T>,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_edge_color_map_of_graph
        : mpl::eval_if<
            is_edge_property_map_of_graph<T,G>,
            is_color_map_impl<T>,
            mpl::false_
        >::type
    {
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

#include <boost/type_traits/is_integral.hpp>

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename T>
    struct is_integral_value_map_impl
        : mpl::eval_if<
            // Ensure that color maps are not mistaken for index maps during
            // type deduction. -- Cromwell D. Enage
            typename is_color_map_impl<T>::type,
            mpl::false_,
            mpl::if_<
                boost::is_integral<typename property_traits<T>::value_type>,
                mpl::true_,
                mpl::false_
            >
        >::type
    {
    };

    template <typename T, typename G>
    struct is_vertex_to_integer_map_of_graph
        : mpl::if_<
            is_vertex_property_map_of_graph<T,G>,
            is_integral_value_map_impl<T>,
            mpl::false_
        >::type
    {
    };

    template <typename T, typename G>
    struct is_edge_to_integer_map_of_graph
        : mpl::if_<
            is_edge_property_map_of_graph<T,G>,
            is_integral_value_map_impl<T>,
            mpl::false_
        >::type
    {
    };
}}

namespace boost { namespace detail {

    template <template <typename> class UnaryPredicate>
    struct argument_predicate
    {
        template <typename Arg, typename ArgPack>
        struct apply
        {
            typedef UnaryPredicate<
                typename boost::remove_const<
                    typename boost::remove_reference<Arg>::type
                >::type
            > type;
        };
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

namespace boost {

    struct vec_adj_list_tag;
}

namespace boost { namespace detail {

    template <typename G>
    struct has_vec_adj_list_graph_tag
        : boost::is_same<typename G::graph_tag,vec_adj_list_tag>
    {
    };
}}

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename G>
    class has_internal_vertex_index_map_impl
    {
        template <typename B>
        static graph_yes_tag
            _check(
                mpl::vector<B>*,
                typename boost::add_pointer<
#if defined(BOOST_NO_CXX11_DECLTYPE)
                    BOOST_TYPEOF_TPL((
                        get(
                            boost::declval<vertex_index_t>(),
                            boost::detail::declcref<B>()
                        )
                    ))
#else
                    decltype(
                        get(
                            boost::declval<vertex_index_t>(),
                            boost::detail::declcref<B>()
                        )
                    )
#endif
                >::type = BOOST_GRAPH_DETAIL_NULLPTR
            );

        static graph_no_tag _check(...);

     public:
        typedef mpl::bool_<
            sizeof(
                has_internal_vertex_index_map_impl<G>::_check(
                    static_cast<mpl::vector<G>*>(BOOST_GRAPH_DETAIL_NULLPTR)
                )
            ) == sizeof(graph_yes_tag)
        > type;
    };
}}
#endif  // defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)

namespace boost { namespace detail {

    // TODO:
    // Implement a more robust check. -- Cromwell D. Enage
    template <typename G>
    struct has_internal_vertex_index_map_dispatch;

    template <typename G>
    struct has_graph_type_with_internal_vertex_index_map
    {
        typedef typename mpl::if_<
            boost::is_same<typename G::graph_type,G>,
            mpl::false_,
            has_internal_vertex_index_map_dispatch<typename G::graph_type>
        >::type type;
    };

    template <typename G>
    struct has_internal_vertex_index_map_dispatch
        : mpl::eval_if<
            typename mpl::eval_if<  // for adjacency_list
                graph_detail::has_graph_tag<G>,
                has_vec_adj_list_graph_tag<G>,
                mpl::false_
            >::type,
            mpl::true_,
            mpl::eval_if<
                is_adjacency_matrix<G>,  // for adjacency_matrix
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
                has_internal_vertex_index_map_impl<G>,
#else
                mpl::true_,
#endif
                mpl::eval_if<
                    has_container_typedefs<G>,  // for vector_as_graph
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
                    has_internal_vertex_index_map_impl<G>,
#else
                    mpl::true_,
#endif
                    mpl::eval_if<
                        is_graph_with_vertex_property_type<G,vertex_index_t>,
#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
                        has_internal_vertex_index_map_impl<G>,
#else
                        mpl::true_,
#endif
                        mpl::eval_if<
                            graph_detail::has_graph_type<G>,  // for adaptors
                            has_graph_type_with_internal_vertex_index_map<G>,
                            mpl::false_
                        >
                    >
                >
            >
        >::type
    {
    };

    template <typename G>
    struct has_internal_vertex_index_map
        : has_internal_vertex_index_map_dispatch<
            typename boost::remove_const<
                typename boost::remove_reference<G>::type
            >::type
        >
    {
    };

    // Several graph algorithms either require a vertex color map or require a
    // vertex index map with which to build a default vertex color map.  This
    // function allows those algorithms to not require the input graph to hold
    // an internal vertex index map when an input color map is provided.
    // -- Cromwell D. Enage
    template <typename G>
    inline typename mpl::eval_if<
        has_internal_vertex_index_map<G>,
        choose_vertex_index_map<G>,
        choose_dummy_property_map<G>
    >::type
        vertex_index_map_or_dummy_property_map(const G& g)
    {
        typedef typename mpl::if_<
            has_internal_vertex_index_map<G>,
            choose_vertex_index_map<G>,
            choose_dummy_property_map<G>
        >::type impl;
        return impl::call(g);
    }

    template <typename Args, typename Tag>
    struct mutable_value_type
        : boost::remove_const<
            typename boost::remove_reference<
                typename boost::parameter::value_type<Args,Tag>::type
            >::type
        >
    {
    };

    template <typename Args, typename Tag>
    struct property_map_value
    {
        typedef typename property_traits<
            typename mutable_value_type<Args,Tag>::type
        >::value_type type;
    };
}}

#if defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <template <typename, typename> class BinaryPredicate>
    struct argument_with_graph_predicate
    {
        template <typename Arg, typename ArgPack>
        struct apply
        {
            typedef BinaryPredicate<
                typename boost::remove_const<
                    typename boost::remove_reference<Arg>::type
                >::type,
                typename mutable_value_type<
                    ArgPack,
                    boost::graph::keywords::tag::graph
                >::type
            > type;
        };
    };

    struct orig_to_copy_vertex_map_predicate
    {
        template <typename Arg, typename ArgPack>
        struct apply
        {
            typedef is_orig_to_copy_vertex_map<
                typename boost::remove_const<
                    typename boost::remove_reference<Arg>::type
                >::type,
                typename mutable_value_type<
                    ArgPack,
                    boost::graph::keywords::tag::graph
                >::type,
                typename mutable_value_type<
                    ArgPack,
                    boost::graph::keywords::tag::result
                >::type
            > type;
        };
    };
}}

#include <boost/mpl/quote.hpp>

namespace boost { namespace detail {

    struct binary_function_graph_predicate
    {
        template <typename Arg, typename ArgPack>
        struct apply
        {
            typedef is_binary_function<
                typename boost::remove_reference<Arg>::type,
                typename boost::remove_const<
                    typename boost::parameter::value_type<
                        ArgPack,
                        boost::graph::keywords::tag::root_vertex
                    >::type
                >::type,
                typename boost::remove_const<
                    typename boost::parameter::value_type<
                        ArgPack,
                        boost::graph::keywords::tag::graph
                    >::type
                >::type,
                mpl::quote1<is_boolean_expression>
            > type;
        };
    };

#if defined(BOOST_NO_CXX11_DECLTYPE) && !defined(BOOST_TYPEOF_KEYWORD)
    // Without decltype() or BOOST_TYPEOF_KEYWORD(), we can detect a
    // Visitor model only by what it isn't rather than by what it is.
    // -- Cromwell D. Enage
    template <typename T, typename G>
    struct is_visitor_impl
        : mpl::eval_if<
            is_vertex_of_graph<T,G>,
            mpl::false_,
            mpl::eval_if<
                is_vertex_property_map_of_graph<T,G>,
                mpl::false_,
                mpl::eval_if<
                    is_edge_property_map_of_graph<T,G>,
                    mpl::false_,
                    mpl::if_<is_buffer<T>,mpl::false_,mpl::true_>
                >
            >
        >::type
    {
    };
}}

#include <boost/mpl/apply_wrap.hpp>

namespace boost { namespace detail {

    struct visitor_predicate
    {
        template <typename Arg, typename ArgPack>
        struct apply
        {
            typedef typename mpl::eval_if<
                typename mpl::apply_wrap2<
                    binary_function_graph_predicate,
                    Arg,
                    ArgPack
                >::type,
                mpl::false_,
                mpl::apply_wrap2<
                    argument_with_graph_predicate<is_visitor_impl>
                    Arg,
                    ArgPack
                >
            >::type type;
        };
    };
#endif  // defined(BOOST_NO_CXX11_DECLTYPE) && !defined(BOOST_TYPEOF_KEYWORD)
}}
#else   // !defined(BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS)
namespace boost { namespace detail {

    template <typename Args, typename VertexTag, typename GraphTag>
    struct is_vertex_property_map_of_graph_argument_impl
        : is_vertex_property_map_of_graph<
            typename mutable_value_type<Args,VertexTag>::type,
            typename mutable_value_type<Args,GraphTag>::type
        >
    {
    };
}}

#include <boost/type_traits/is_base_of.hpp>

namespace boost { namespace detail {

    template <typename Args, typename Tag>
    struct is_bgl_named_param_argument_impl
        : mpl::if_<
            boost::is_base_of<
                detail::bgl_named_params_base,
                typename detail::mutable_value_type<Args,Tag>::type
            >,
            mpl::true_,
            mpl::false_
        >::type
    {
    };
}}

#include <boost/mpl/has_key.hpp>

namespace boost { namespace detail {

    template <typename Args, typename VertexTag, typename GraphTag>
    struct is_vertex_property_map_of_graph_argument
        : mpl::if_<
            typename mpl::eval_if<
                mpl::has_key<Args,VertexTag>,
                mpl::has_key<Args,GraphTag>,
                mpl::false_
            >::type,
            is_vertex_property_map_of_graph_argument_impl<
                Args,
                VertexTag,
                GraphTag
            >,
            mpl::false_
        >::type
    {
    };

    template <typename Args, typename Tag>
    struct is_bgl_named_param_argument
        : mpl::if_<
            typename mpl::has_key<Args,Tag>::type,
            is_bgl_named_param_argument_impl<Args,Tag>,
            mpl::false_
        >::type
    {
    };
}}
#endif  // BOOST_GRAPH_CONFIG_CAN_DEDUCE_PARAMETERS

#include <cstddef>
#include <utility>

namespace boost { namespace detail {

    template <typename Args, typename Tag>
    struct make_size_t_value_pair
    {
        typedef std::pair<
            std::size_t,
            typename mutable_value_type<Args,Tag>::type
        > type;
    };
}}

#endif  // include guard

