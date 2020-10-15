
#ifndef  _dijkstra_
#define  _dijkstra_

#include <functional>
#include "graph_priority_search.h"

template<class node_t>
class dijkstra_process:public priority_search<node_t>
{
    using weight_t=typename node_t::weight_t;

    using iterator=typename node_t::iterator;

    static weight_t relax(const iterator&iter,weight_t from_weight)
    {
        return iter.weight()+from_weight;
    }

    public:
    dijkstra_process(const std::vector< node_t*>& graph_nodes)
    :priority_search<node_t>(graph_nodes){}


    bool get_path(const node_t& from,const node_t& to,std::vector<iterator>& iters)const;
    weight_t find_path(const node_t& from,const node_t& to,std::vector<iterator>& iters);
    template<class _Restr>
    void restricted_find_path(const node_t& from,_Restr& restrictor);
    bool find_path_with_def_restrictors(const node_t& from,
                                        std::vector<iterator>& iters,
                                        std::function<bool(const node_t&)> goal,
                                        std::function<bool(const node_t&)> node_filter=nullptr,
                                        std::function<bool(const iterator&iter,float)>
                                        edge_filter=nullptr);

};

template<class node_t>
bool dijkstra_process<node_t>::get_path(const node_t& from,const node_t& to,std::vector<iterator>& iters)const
{
    iters.clear();
    const node_t* from_ptr=&from;
    if(this->m_nodes_state(to)!=priority_search<node_t>::Dead) return false;
    const node_t* to_ptr=&to;
    iterator it;
    while(to_ptr!=from_ptr)
    {
        it=this->m_prev_iterator(*to_ptr);
        iters.push_back(it);
        to_ptr=&it.from_node();
    }
    std::reverse(iters.begin(),iters.end());
    return true;
}


template<class node_t>
template<class _Restr>
void dijkstra_process<node_t>::restricted_find_path(const node_t& start_node,_Restr& restrictor)
{
    static_assert(_Restr::_specific_restriction!=0);
    this->done(start_node,restrictor,relax);
}

template<class node_t>
typename node_t::weight_t
dijkstra_process<node_t>::find_path(const node_t& from,const node_t& to,std::vector<iterator>& iters)
{
    struct find_node:public search_restrictor<has_node_finish>
    {
        const node_t& dest;
        public:
        find_node(const node_t&_nd):dest(_nd){}
        bool is_node_finish(const node_t& node)
        {
            return &dest==&node;
        }
    };
    find_node finder(to);
    restricted_find_path(from,finder);

    return get_path(from,to,iters)? this->m_weights(to):-1.0;
}


template<class node_t>
bool dijkstra_process<node_t>::find_path_with_def_restrictors(const node_t& from,
                                    std::vector<iterator>& iters,
                                    std::function<bool(const node_t&)> fn_goal,
                                    std::function<bool(const node_t&)> fn_node_filter,
                                    std::function<bool(const iterator&iter,float)>
                                    fn_edge_filter)
{
    iters.clear();
    const node_t* goal;

    auto finder=search_terminator<node_t>(fn_goal);

    if(fn_node_filter!=nullptr)
    {
        auto nf_finder=finder|
        node_filter<node_t>(fn_node_filter);

        if(fn_edge_filter!=nullptr)
        {
            auto ef_nf_finder=nf_finder|
            edge_filter<node_t>(fn_edge_filter);

            restricted_find_path(from,ef_nf_finder);
            goal=ef_nf_finder.goal_node;
        }
        else
        {
            restricted_find_path(from,nf_finder);
            goal=nf_finder.goal_node;
        }
    }
    else if(fn_edge_filter!=nullptr)
    {
        auto ef_finder=finder|
        edge_filter<node_t>(fn_edge_filter);

        restricted_find_path(from,ef_finder);
        goal=ef_finder.goal_node;
    }
    else
    {
        restricted_find_path(from,finder);
        goal=finder.goal_node;
    }
    if(goal==nullptr) return false;
    bool res=get_path(from,*goal,iters);
    assert(res);
    return true;
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

