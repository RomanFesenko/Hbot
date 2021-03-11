
#ifndef  _prim_
#define  _prim_


#include "graph_priority_search.h"

///for undirected graphs

template<class node_t>
class prim_process:public priority_search<node_t>
{

    public:
    using iterator=typename node_t::iterator;
    using weight_t=typename node_t::weight_t;
    private:
    static weight_t relax(const iterator&iter)
    {
        return iter.weight();
    }

    public:
    prim_process(const std::vector< node_t*>& graph_nodes)
    :priority_search<node_t>(graph_nodes){}

    weight_t build_spanning_tree(const node_t& from);

};

template<class node_t>
typename node_t::weight_t prim_process<node_t>::build_spanning_tree(const node_t& start_node)
{
    search_restrictor<0> not_restr;
    this->done(start_node,not_restr,relax);
    weight_t result=this->weight_zero();
    for(const node_t* node:this->m_graph_nodes)
    {
        if(node==&start_node) continue;
        result+=this->m_prev_iterator(*node).weight();
    }
    return result;
}

#endif

