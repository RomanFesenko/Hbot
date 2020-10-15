
#ifndef  _prim_
#define  _prim_


#include "graph_priority_search.h"

///для ненаправленого графа

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

//TEST
    ///#1
    /*link_vec_t<float> lv={
        {0,1,0.41,id_1_2_edge},
        {0,5,0.29,id_1_2_edge},
        {1,2,0.51,id_1_2_edge},
        {1,4,0.32,id_1_2_edge},
        {2,3,0.5,id_1_2_edge},
        {3,0,0.45,id_1_2_edge},
        {3,5,0.38,id_1_2_edge},
        {4,2,0.32,id_1_2_edge},
        {4,3,0.36,id_1_2_edge},
        {5,1,0.29,id_1_2_edge},
        {5,4,0.21,id_1_2_edge}
    };
    using node_t=CNodeGraph<float>;

    CBaseGraph BaseGraph(lv);
    dijkstra_process<node_t> dp(&(BaseGraph.graphnodes[0]),BaseGraph.NumNodes());
    std::vector<typename node_t::iterator> path;

    int numn=BaseGraph.NumNodes();
    for(int i=0;i<numn;i++)
    {
        for(int j=0;j<numn;j++)
        {
            auto res=dp.find_path(i,j,path);
            printf("%d %s %d %s %f %s",i," ",j,"path:",res,"\n");
        }
    }*/
#endif

