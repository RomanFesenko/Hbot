
#ifndef  _path_iterator_
#define  _path_iterator_

#include "property.h"
#include "../Iterator_facade/iterator_facade.hpp"

// Some of the graph algorithms associate
// to each node of the graph an edge, lying on the path to this node,
// path_iterator_t is a way, to iterate over these edges, likes STL iterators

template<class node_t>
class path_iterator_t:public iterator_facade<path_iterator_t<node_t>,
                             std::forward_iterator_tag,
                             typename node_t::iterator>
{
    using iterator=node_t::iterator;
    TProperty<iterator,node_t>& m_prev_iterator;
    const node_t* m_current_node;
    public:
    explicit path_iterator_t(TProperty<iterator,node_t>& prev_iterator,
                             const node_t& node):
    m_prev_iterator(prev_iterator),
    m_current_node(&node){}

    path_iterator_t<node_t>& operator=(const path_iterator_t<node_t>&other)
    {
        m_current_node=other.m_current_node;
        return *this;
    }
    iterator& dereference()const{return m_prev_iterator(*m_current_node);}
    void increment()
    {
        m_current_node=&m_prev_iterator(*m_current_node).from_node();
    }
    bool equal_to(const path_iterator_t&other)const
    {
        return m_current_node==other.m_current_node;
    }
};

template<class node_t>
class reverse_path_t
{
    const node_t* m_start=nullptr;
    const node_t* m_end=nullptr;
    using iterator=node_t::iterator;
    TProperty<iterator,node_t>& m_prev_iterator;
    public:
    explicit reverse_path_t(TProperty<iterator,node_t>&prev_iterator):
    m_prev_iterator(prev_iterator){}
    void set(const node_t& start,const node_t& end)
    {
        m_start=&start;m_end=&end;
    }
    path_iterator_t<node_t> begin()const
    {
        return path_iterator_t<node_t>(m_prev_iterator,*m_end);
    }
    path_iterator_t<node_t> end()const
    {
        return path_iterator_t<node_t>(m_prev_iterator,*m_start);
    }
};


#endif

