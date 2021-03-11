
#ifndef  _depth_search_
#define  _depth_search_

#include <assert.h>

#include <utility>
#include <vector>
#include "property.h"
#include "search_adaptors.h"

const int ds_node_discover=1<<0;
const int ds_node_close=1<<1;
const int ds_tree_edge=1<<2;
const int ds_nontree_edge=1<<3;


template<class node_t,int flag>
struct default_ds_visitor_t
{
    using iterator=node_t::iterator;
    constexpr void node_discover(const node_t&)
    requires (!(flag&ds_node_discover))
    {}
    constexpr void node_close(const node_t&)
    requires (!(flag&ds_node_close))
    {}
    constexpr void tree_edge_process(const iterator&)
    requires (!(flag&ds_tree_edge))
    {}
    constexpr void nontree_edge_process(const iterator&)
    requires (!(flag&ds_nontree_edge))
    {}
};


template<class node_t>
class depth_search
{
    using size_type=std::vector<node_t*>::size_type;
    const std::vector< node_t*>& m_graph_nodes;
    size_type m_num_nodes;
    public:
    enum State:char{Sleep=0,Alive,Dead};
    private:
    TProperty<State,node_t> m_nodes_state;

    int m_timer=0;

    using iterator= typename node_t::iterator;//for traversal adjacent
    //discovere time of node
    TProperty<int,node_t> m_entry_time;
    //close time of node
    TProperty<int,node_t> m_close_time;
    // iterator - edge leading to node
    TProperty<iterator,node_t> m_prev_iterator;//prev_node[node.index()] - previous node
    public:


    explicit depth_search(const std::vector< node_t*>& graph_nodes):
    m_graph_nodes(graph_nodes),
    m_num_nodes(graph_nodes.size()),
    m_nodes_state(m_num_nodes),
    m_entry_time(m_num_nodes),
    m_close_time(m_num_nodes),
    m_prev_iterator(m_num_nodes){}


    template<class _Restr>
    size_type done(const node_t& from,_Restr& restrictor,bool freeze=false);

    int entry_time(const node_t& node) const
    {
        return m_entry_time(node);
    }

    int close_time(const node_t& node) const
    {
        return m_close_time(node);
    }

    const iterator& prev_iterator(const node_t& node) const
    {
        return m_prev_iterator(node);
    }

    const node_t& parent(const node_t& node) const
    {
        return m_prev_iterator(node).from_node();
    }

    bool is_find(const node_t& node)const
    {
        return m_nodes_state(node)==Dead;
    }

    State state(const node_t& node)const
    {
        return m_nodes_state(node);
    }

    const std::vector< node_t*>& nodes()const
    {
        return m_graph_nodes;
    }
};

//depth_search
//return - number nodes,which available from start_node

template<class node_t>
template<class _Restr>
depth_search<node_t>::size_type
depth_search<node_t>::done(const node_t& start_node,_Restr& restrictor,bool freeze_state)
{
    // between calls number of nodes
    // no changed
    assert(m_num_nodes==m_graph_nodes.size());

    // save or not, previous call result
    if (!freeze_state)
    {
        std::fill(m_nodes_state.begin(),m_nodes_state.end(),Sleep);
        m_timer=0;
    }

    std::vector<iterator> stack;
    size_type num_connective_nodes=0;
    auto wake_up_node=[&stack,&num_connective_nodes,&restrictor,this](const node_t&node)
    {
        iterator it;it.set_base(node);
        stack.push_back(it);
        m_entry_time(node)=m_timer++;
        m_nodes_state(node)=Alive;
        num_connective_nodes++;
        restrictor.node_discover(node);
    };

    wake_up_node(start_node);
    while(!stack.empty())
    {
        iterator& current_it=stack.back();
        const node_t& from=current_it.from_node();
        if(current_it.end())
        {
            m_close_time(from)=m_timer++;
            m_nodes_state(from)=Dead;
            restrictor.node_close(from);
            stack.pop_back();
            continue;
        }
        else if(const node_t& to=current_it.to_node();m_nodes_state(to)==Sleep)
        {
            restrictor.tree_edge_process(current_it);
            m_prev_iterator(to)=current_it;
            current_it.next();//!
            wake_up_node(to);
            continue;
        }
        else
        {
            if(&parent(from)!=&to)
            {
                restrictor.nontree_edge_process(current_it);
            }
            current_it.next();
        }
    } //while
    return num_connective_nodes;
}

#endif

