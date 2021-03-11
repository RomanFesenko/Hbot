
#ifndef  _breadth_search_
#define  _breadth_search_

#include <assert.h>

#include <queue>
#include "property.h"
#include "search_adaptors.h"


//The presence of a handler of visited nodes that have
//already been processed,
// need an acyclic check graph
template<class _Restr,class node_t>
concept def_dead_node_process=requires(_Restr& res,const node_t&node)
{
    res.dead_node_process( node);
};

template<class node_t>
class breadth_search
{
    using size_type=std::vector<node_t*>::size_type;
    const std::vector< node_t*>& m_graph_nodes;
    size_type m_num_nodes;
    TProperty<char,node_t> m_nodes_state;
    enum State{Sleep=0,Alive,Dead,Killed};
    public:

    using iterator= typename node_t::iterator;//for traversal adjacent
    //minimal count edge on path to node
    TProperty<size_type,node_t> m_distance;
    // iterator - edge leading to node
    TProperty<iterator,node_t> m_prev_iterator;//prev_node[node.index()] - previous node

    breadth_search(const std::vector< node_t*>& graph_nodes)
    :m_graph_nodes(graph_nodes),m_num_nodes(graph_nodes.size()),
    m_nodes_state(m_num_nodes),m_distance(m_num_nodes),
    m_prev_iterator(m_num_nodes){}


    template<class _Restr>
    size_type done(const node_t& from,_Restr& restrictor,bool freeze=false);

    template<class _Restr,class _Signal>
    void union_parts_process(_Restr&restr,_Signal& signal);

    size_type distance(const node_t& node)const
    {
        return m_distance(node);

    }
    bool is_find(const node_t& node)const
    {
        return m_nodes_state(node)==Dead;
    }
};

//breadth_search
//return - number nodes,which available from start_node

template<class node_t>
template<class _Restr>
breadth_search<node_t>::size_type
breadth_search<node_t>::done(const node_t& start_node,_Restr& restrictor,bool freeze_state)
{

    //insurance against incorrect declaration of visitors in the restrictor-
    // assert will work if there is no visitor if available
    // corresponding flag in _specific_restriction inherited
    // from search_restrictor

    static_assert(((_Restr::_specific_restriction&has_node_discover)!=0)==
                 def_node_discover<_Restr,node_t>);

    static_assert(((_Restr::_specific_restriction&has_unary_edge_process)!=0)==
                 def_unary_edge_process<_Restr,node_t>);


    static_assert(((_Restr::_specific_restriction&has_node_finish)!=0)==
                 def_is_node_finish<_Restr,node_t>);


    static_assert(((_Restr::_specific_restriction&has_dead_node_process)!=0)==
                 def_dead_node_process<_Restr,node_t>);

    // between calls number of nodes
    // no changed
    assert(m_num_nodes==m_graph_nodes.size());

    // save or not previous call result
    if (!freeze_state)
    {
        std::fill(m_nodes_state.begin(),m_nodes_state.end(),Sleep);
    }

    std::queue<const node_t*> queue;
    size_type num_connective_nodes=0;
    auto activate_node=[&queue,&num_connective_nodes,this](const node_t&node,size_type dist)
    {
        queue.push(&node);
        m_distance(node)=dist;
        m_nodes_state(node)=Alive;
        num_connective_nodes++;
    };

    if constexpr(def_node_discover<_Restr,node_t>)
    {
        if(restrictor.node_discover(start_node))
        {
            activate_node(start_node,0);
        }
        else
        {
            m_nodes_state(start_node)=Killed;
        }
    }
    else
    {
         activate_node(start_node,0);
    }
    iterator it;
    while(!queue.empty())
    {
        const node_t& new_dead=*queue.front();
        queue.pop();

        m_nodes_state(new_dead)=Dead;
        if constexpr(def_is_node_finish<_Restr,node_t>)
        {
             if(restrictor.is_node_finish(new_dead)) break;
        }
        it.set_base(new_dead);

        //processing all outgoing edges
        size_type new_dist=m_distance(it.from_node())+1;
        const node_t* parent=&m_prev_iterator(new_dead).from_node();
        for(it.begin();!it.end();it.next())
        {
            // whether this edge can be processed?
            if constexpr(def_unary_edge_process<_Restr,node_t>)
            {
                if(!restrictor.is_edge_process(it)) continue;
            }
            // whether
            const node_t& to_n=it.to_node();

            if(m_nodes_state(to_n)==Sleep)//sleeping node, wake up
            {
                if constexpr(def_node_discover<_Restr,node_t>)
                {
                    // filtering the node
                    if(restrictor.node_discover(to_n))
                    {
                        activate_node(to_n,new_dist);
                        m_prev_iterator(to_n)=it;
                    }
                    else
                    {
                        m_nodes_state(to_n)=Killed;
                    }
                }
                else
                {
                    activate_node(to_n,new_dist);
                    m_prev_iterator(to_n)=it;
                }
            }
            else// dead or killed
            {
                if constexpr(def_dead_node_process<_Restr,node_t>)
                {
                    if(m_nodes_state(to_n)==Dead && (&to_n)!=parent)
                    {
                        restrictor.dead_node_process(to_n);
                    }
                }
            }
        }// for
    } //while

    return num_connective_nodes;
}

// processing of all connected components of an undirected graph

template<class node_t>
template<class _Restr,class _Signal>
void breadth_search<node_t>::union_parts_process(_Restr&restr,_Signal& signal)
{
    std::fill(m_nodes_state.begin(),m_nodes_state.end(),Sleep);

    auto _begin=m_graph_nodes.begin();
    while(true)
    {
        _begin=
        find_if(_begin,
                m_graph_nodes.end(),
                [this](node_t*node)
                {
                    return m_nodes_state(*node)==Sleep;
                });

        if(_begin==m_graph_nodes.end()) return;

        signal();
        done(**_begin,restr,true);
    }
}

#endif

