//
//=======================================================================
// Copyright 1997, 1998, 1999, 2000 University of Notre Dame.
// Authors: Andrew Lumsdaine, Lie-Quan Lee, Jeremy G. Siek
//
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//=======================================================================
//
//
// Revision History:
//   13 June 2001: Changed some names for clarity. (Jeremy Siek)
//   01 April 2001: Modified to use new <boost/limits.hpp> header. (JMaddock)
//   28 Feb 2017: change bucket head, fix bug in remove. (Felix Salfelder)
//
#ifndef BOOST_GRAPH_DETAIL_BUCKET_SORTER_HPP
#define BOOST_GRAPH_DETAIL_BUCKET_SORTER_HPP

#include <vector>
#include <cassert>
#include <boost/limits.hpp>
#include <boost/config.hpp>

namespace boost {

  template <class BucketType, class ValueType, class Bucket, 
            class ValueIndexMap>
  class bucket_sorter {
  public:
    typedef BucketType bucket_type;
    typedef ValueType value_type;
    typedef typename std::vector<value_type>::size_type size_type;
    
    bucket_sorter(size_type _length, bucket_type _max_bucket, 
                  const Bucket& _bucket = Bucket(), 
                  const ValueIndexMap& _id = ValueIndexMap()) 
      : head(_max_bucket, invalid_value()),
        next(_length, invalid_value()), 
        prev(_length, invalid_value()),
        id_to_value(_length),
        bucket(_bucket), id(_id) { }
    
    void remove(const value_type& x) {
      const size_type i = get(id, x);
      const size_type& next_node = next[i];
      const size_type& prev_node = prev[i];
    
      //check if i is the end of the bucket list 
      if ( next_node != invalid_value() )
        prev[next_node] = prev_node; 
      //check if i is the begin of the bucket list
      if ( prev_node != invalid_value() )
        next[prev_node] = next_node;
      else //need update head of current bucket list
        head[ bucket[x] ] = next_node;
    }

    void push(const value_type& x) {
      id_to_value[get(id, x)] = x;
      (*this)[bucket[x]].push(x);
    }
    
    void update(const value_type& x) {
      remove(x);
      (*this)[bucket[x]].push(x);
    }
    //  private: 
    //    with KCC, the nested stack class is having access problems
    //    despite the friend decl.
    static size_type invalid_value() {
      return (std::numeric_limits<size_type>::max)();
    }

    friend class stack;

  private:
#if defined(BOOST_CLANG) && (1 == BOOST_CLANG) && defined(__APPLE_CC__)
    typedef std::vector<size_type> SizeVec;
    typedef typename SizeVec::size_type Index;
    typedef std::vector<value_type> ValueVec;
    typedef typename ValueVec::size_type IndexValueIndex;

  public:
    class stack {
      bucket_type     bucket_id;
      SizeVec&        head;
      SizeVec&        next;
      SizeVec&        prev;
      ValueVec&       id_to_value;
      Index           head_index;
      Index           next_index;
      Index           prev_index;
      IndexValueIndex value_index;
      ValueIndexMap   id;

    public:
      stack(
          bucket_type _bucket_id
        , SizeVec& h
        , Index h_i
        , SizeVec& n
        , Index n_i
        , SizeVec& p
        , Index p_i
        , ValueVec& v
        , IndexValueIndex v_i
        , const ValueIndexMap& _id
      ) : bucket_id(_bucket_id)
        , head(h)
        , next(n)
        , prev(p)
        , id_to_value(v)
        , head_index(h_i)
        , next_index(n_i)
        , prev_index(p_i)
        , value_index(v_i)
        , id(_id)
      {
      }

      // Avoid using default arg for ValueIndexMap so that the default
      // constructor of the ValueIndexMap is not required if not used.
      stack(
          bucket_type _bucket_id
        , SizeVec& h
        , Index h_i
        , SizeVec& n
        , Index n_i
        , SizeVec& p
        , Index p_i
        , ValueVec& v
        , IndexValueIndex v_i
      ) : bucket_id(_bucket_id)
        , head(h)
        , next(n)
        , prev(p)
        , id_to_value(v)
        , head_index(h_i)
        , next_index(n_i)
        , prev_index(p_i)
        , value_index(v_i)
      {
      }

      void push(const value_type& x) {
        const size_type new_head = get(id, x);
        const size_type current = (head.begin() + head_index)[bucket_id];
        if ( current != invalid_value() )
          (prev.begin() + prev_index)[current] = new_head;
        (prev.begin() + prev_index)[new_head] = invalid_value();
        (next.begin() + next_index)[new_head] = current;
        (head.begin() + head_index)[bucket_id] = new_head;
      }
      void pop() {
        size_type current = (head.begin() + head_index)[bucket_id];
        size_type next_node = (next.begin() + next_index)[current];
        (head.begin() + head_index)[bucket_id] = next_node;
        if ( next_node != invalid_value() )
          (prev.begin() + prev_index)[next_node] = invalid_value();
      }
      value_type& top() {
        return (id_to_value.begin() + value_index)[ (head.begin() + head_index)[bucket_id] ];
      }
      const value_type& top() const {
        return (id_to_value.begin() + value_index)[ (head.begin() + head_index)[bucket_id] ];
      }
      bool empty() const { return (head.begin() + head_index)[bucket_id] == invalid_value(); }
    };

    stack operator[](const bucket_type& i) {
      assert(i < head.size());
      return stack(i, head, 0, next, 0, prev, 0, id_to_value, 0, id);
    }
#else   // not xcode
    typedef typename std::vector<size_type>::iterator Iter;
    typedef typename std::vector<value_type>::iterator IndexValueMap;

  public:
    class stack {
    public:
      stack(bucket_type _bucket_id, Iter h, Iter n, Iter p, IndexValueMap v,
            const ValueIndexMap& _id)
      : bucket_id(_bucket_id), head(h), next(n), prev(p), value(v), id(_id) {}

      // Avoid using default arg for ValueIndexMap so that the default
      // constructor of the ValueIndexMap is not required if not used.
      stack(bucket_type _bucket_id, Iter h, Iter n, Iter p, IndexValueMap v)
        : bucket_id(_bucket_id), head(h), next(n), prev(p), value(v) {}

      void push(const value_type& x) {
        const size_type new_head = get(id, x);
        const size_type current = head[bucket_id];
        if ( current != invalid_value() )
          prev[current] = new_head;
        prev[new_head] = invalid_value();
        next[new_head] = current;
        head[bucket_id] = new_head;
      }
      void pop() {
        size_type current = head[bucket_id];
        size_type next_node = next[current];
        head[bucket_id] = next_node;
        if ( next_node != invalid_value() )
          prev[next_node] = invalid_value();
      }
      value_type& top() { return value[ head[bucket_id] ]; }
      const value_type& top() const { return value[ head[bucket_id] ]; }
      bool empty() const { return head[bucket_id] == invalid_value(); }
    private:
      bucket_type bucket_id;
      Iter head;
      Iter next;
      Iter prev;
      IndexValueMap value;
      ValueIndexMap id;
    };

    stack operator[](const bucket_type& i) {
      assert(i < head.size());
      return stack(i, head.begin(), next.begin(), prev.begin(),
                   id_to_value.begin(), id);
    }
#endif  // xcode
  protected:
    std::vector<size_type>   head;
    std::vector<size_type>   next;
    std::vector<size_type>   prev;
    std::vector<value_type>  id_to_value;
    Bucket bucket;
    ValueIndexMap id;
  };
  
}

#endif
