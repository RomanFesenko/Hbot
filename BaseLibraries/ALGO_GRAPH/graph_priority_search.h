
#ifndef  _priority_graph_search_
#define  _priority_graph_search_

#include <assert.h>

#include <vector>
#include "property.h"
#include "graph_priority_queue.h"
#include "search_adaptors.h"

// node iterator interface:
// it.set_base(const node& nd);
// it.begin()
// it.end()
// it.next()
// const node& it.from_node()
// const node& it.to_node()
// weight_t it.weight()



// in Dijkstra's algorithms, Prima is used
// relaxation procedure with different argument lists

template<class node_t,class _Relax_func_t>
concept unary_relaxer=requires(const node_t::iterator& iter,_Relax_func_t relax)
{
    relax(iter);
};

template<class node_t,class _Relax_func_t>
concept binary_relaxer=requires(const node_t::iterator& iter,
                                node_t::weight_t weight,
                                _Relax_func_t relax)
{
    relax(iter,weight);
};


// generalized search on the graph using a priority queue
// with support for arbitrary edge relaxation procedure
// and edge / node filters

// special cases - alg. Dijkstra, Prima, augmental path search options
// in alg. Ford - Fulkerson

template<class node_t>
class priority_search
{
    protected:
    const std::vector< node_t*>& m_graph_nodes;
    typename std::vector<node_t*>::size_type m_num_nodes;
    TProperty<char,node_t> m_nodes_state;
    enum State{Sleep=0,Alive,Dead,Killed};
    public:
    using ptr_node_t=node_t*;
    using weight_t= typename node_t::weight_t;
    using iterator= typename node_t::iterator;// iterator to traverse adjacent
    TProperty<weight_t,node_t> m_weights;

    // iterator - edge leading from the previous vertex
    TProperty<iterator,node_t> m_prev_iterator;//prev_node[node.index()] - previous for node

    priority_search(const std::vector< node_t*>& graph_nodes)
    :m_graph_nodes(graph_nodes),m_num_nodes(graph_nodes.size()),
    m_nodes_state(m_num_nodes),m_weights(m_num_nodes),
    m_prev_iterator(m_num_nodes){}

    weight_t weight_zero(){return weight_t(0);}

    template<class _Restr,class _Relax_func_t>
    void done(const node_t& from,_Restr& restrictor,_Relax_func_t);
    weight_t weight_result(const node_t& node)const
    {
        return m_weights(node);

    }
    bool is_find(const node_t& node)const
    {
        return m_nodes_state(node)==Dead;
    }
};

/* relaxer - function to relax the edge

 restrictor interface (any member function optional):
 restrictor.node_discover-> bool - node visitor on first visit,
 if it returns false, the node is excluded from the search;
 restrictor.is_edge_process-> bool - edge visitor, if it returns
 false, the edge is excluded from the search;
 restrictor.is_node_finish-> bool node visitor on last visit,
 if it returns true, the search stops;

 restrictor must inherit from search_restrictor for example:
 CRestrictor:public search_restrictor<has_node_discover|
                                      has_node_finish|
                                      ....>

*/

template<class node_t>
template<class _Restr,class _Relax_func_t>
void priority_search<node_t>::done(const node_t& start_node,_Restr& restrictor,
                                        _Relax_func_t relaxer)
{
    //insurance against incorrect declaration of visitors in the restrictor-
    // assert will work if there is no visitor if available
    // corresponding flag in _specific_restriction inherited
    // from search_restrictor

    static_assert(((_Restr::_specific_restriction&has_node_discover)!=0)==
                 def_node_discover<_Restr,node_t>);

    static_assert(((_Restr::_specific_restriction&has_unary_edge_process)!=0)==
                 def_unary_edge_process<_Restr,node_t>);

    static_assert(((_Restr::_specific_restriction&has_binary_edge_process)!=0)==
                 def_binary_edge_process<_Restr,node_t>);


    static_assert(((_Restr::_specific_restriction&has_node_finish)!=0)==
                 def_is_node_finish<_Restr,node_t>);

    //static_assert(_Restr::_specific_restriction!=0);

    //Must define only once from
    static_assert((!def_binary_edge_process<_Restr,node_t>)||
                  (!def_unary_edge_process<_Restr,node_t>));

    // between calls number of nodes
    // no changed
    assert(m_num_nodes==m_graph_nodes.size());
    std::fill(m_nodes_state.begin(),m_nodes_state.end(),Sleep);

    //the next from the queue is the node with the lowest weight
    auto fpriority=[](weight_t a,weight_t b){return a > b;};

    graph_priority_queue<node_t,weight_t, decltype(fpriority)>
    queue(m_weights,fpriority);///

    if constexpr(def_node_discover<_Restr,node_t>)
    {
        if(restrictor.node_discover(start_node))
        {
            queue.push(start_node,weight_zero());
            m_nodes_state(start_node)=Alive;
        }
        else
        {
            m_nodes_state(start_node)=Killed;
        }
    }
    else
    {
         queue.push(start_node,weight_zero());
         m_nodes_state(start_node)=Alive;
    }
    iterator it;
    while(!queue.empty())
    {
        const node_t& new_dead=*queue.top();queue.pop();

        /*cout<< "m_weights(new_dead): "<<m_weights(new_dead)<<endl;
        system("PAUSE");*/

        m_nodes_state(new_dead)=Dead;
        if constexpr(def_is_node_finish<_Restr,node_t>)
        {
             if(restrictor.is_node_finish(new_dead)) break;
        }
        it.set_base(new_dead);
        //processing all outgoing edges
        for(it.begin();!it.end();it.next())
        {
            // relaxation
            weight_t new_weight;
            if constexpr(unary_relaxer<node_t,_Relax_func_t>)
            {
                new_weight=relaxer(it);//prim
            }
            else// Dijkstra,augmental path
            {
                new_weight=relaxer(it,m_weights(it.from_node()));
            }

            // whether this edge can be processed?
            if constexpr(def_unary_edge_process<_Restr,node_t>)
            {
                if(!restrictor.is_edge_process(it)) continue;
            }
            else if constexpr(def_binary_edge_process<_Restr,node_t>)
            {
                if(!restrictor.is_edge_process(it,new_weight))continue;
            }
            else{}
            // whether
            const node_t& to_n=it.to_node();
            if(m_nodes_state(to_n)==Alive)
            {
                if(new_weight<m_weights(to_n))
                {
                    queue.inc_priority(to_n,new_weight);
                    m_prev_iterator(to_n)=it;
                }
            }
            else if(m_nodes_state(to_n)==Sleep) // sleeping node, wake up
            {
                if constexpr(def_node_discover<_Restr,node_t>)
                {
                    // filter node
                    if(restrictor.node_discover(to_n))
                    {
                        queue.push(to_n,new_weight);
                        m_prev_iterator(to_n)=it;
                        m_nodes_state(to_n)=Alive;
                    }
                    else
                    {
                        m_nodes_state(to_n)=Killed;
                    }
                }
                else
                {
                    queue.push(to_n,new_weight);
                    m_prev_iterator(to_n)=it;
                    m_nodes_state(to_n)=Alive;
                }
            }
            else {}// dead or killed
        }// for
    } //while
}

#endif

