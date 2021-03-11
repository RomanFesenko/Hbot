
#ifndef  _edmonds_karp_
#define  _edmonds_karp_

#include <assert.h>
#include <memory>
#include <optional>


#include "graph_priority_search.h"
#include "breadth_search.h"
#include "network_composer.h"
#include "path_iterator.h"

// Every augmental path - path
// with min number of edges in residual graph

template<class flow_t>
flow_t ff_min_path(network_t<flow_t>&network,int isource,int isink)
{
    assert(isource!=isink);
    using node_t=network_t<flow_t>::node_t;
    using edge_t=network_t<flow_t>::edge_t;
    using iterator=node_t::iterator;

    node_t&source=*network.nodes()[isource];
    node_t&sink=*network.nodes()[isink];
    std::unique_ptr<breadth_search<node_t>> bs(new breadth_search<node_t>(network.nodes()));
    reverse_path_t<node_t> aug_path(bs->m_prev_iterator);
    aug_path.set(source,sink);

    auto add_flow=[](edge_t&edge,flow_t flow)
    {
        edge.add_flow(flow);
        if(!edge.reverse_edge().is_in_list())
        {
            edge.adjacent_node().insert(edge.reverse_edge());
        }
    };
    auto fill_edge=[](edge_t&edge)
    {
        edge.fill();
        if(!edge.reverse_edge().is_in_list())
        {
            edge.adjacent_node().insert(edge.reverse_edge());
        }
        edge.erase();
    };
    auto set_augmental_path=[&source,&sink,&aug_path,fill_edge,add_flow]()
    {
        auto min_cap_iter=std::min_element(aug_path.begin(),aug_path.end(),
                          [](const iterator& iter1,const iterator& iter2)
                          {
                                return iter1.capacity()<iter2.capacity();
                          });
        flow_t min_flow=min_cap_iter->capacity();

        for(auto& iter:aug_path)
        {
            add_flow(iter.edge(),min_flow);
        }
        fill_edge((*min_cap_iter).edge());
        return min_flow;
    };

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
    find_node fn(sink);

    flow_t total=flow_t{0};
    while(true)
    {
        bs->done(source,fn);
        if(!bs->is_find(sink)) break;
        total+=set_augmental_path();
    }
    return total;
}

// Every augmental path - path
// with max added flow in residual graph

template<class flow_t>
flow_t ff_max_flow(network_t<flow_t>&network,int isource,int isink)
{
    assert(isource!=isink);
    using node_t=network_t<flow_t>::node_t;
    using edge_t=network_t<flow_t>::edge_t;
    using iterator=node_t::iterator;

    node_t&source=*network.nodes()[isource];
    node_t&sink=*network.nodes()[isink];
    std::unique_ptr<priority_search<node_t>> bs(new priority_search<node_t>(network.nodes()));
    reverse_path_t<node_t> aug_path(bs->m_prev_iterator);
    aug_path.set(source,sink);

    auto relax_function=[](const iterator&iter,flow_t max_flow)
    {
        return std::min(iter.capacity(),max_flow);
    };
    auto add_flow=[](edge_t&edge,flow_t flow)
    {
        edge.add_flow(flow);
        if(!edge.reverse_edge().is_in_list())
        {
            edge.adjacent_node().insert(edge.reverse_edge());
        }
    };
    auto fill_edge=[](edge_t&edge)
    {
        edge.fill();
        if(!edge.reverse_edge().is_in_list())
        {
            edge.adjacent_node().insert(edge.reverse_edge());
        }
        edge.erase();
    };
    auto set_augmental_path=[&source,&sink,&aug_path,fill_edge,add_flow]()
    {
        auto min_cap_iter=std::min_element(aug_path.begin(),aug_path.end(),
                          [](const iterator& iter1,const iterator& iter2)
                          {
                                return iter1.capacity()<iter2.capacity();
                          });
        flow_t min_flow=min_cap_iter->capacity();

        for(auto& iter:aug_path)
        {
            add_flow(iter.edge(),min_flow);
        }
        fill_edge((*min_cap_iter).edge());
        return min_flow;
    };

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
    find_node fn(sink);

    flow_t total=flow_t{0};
    while(true)
    {
        bs->done(source,fn,relax_function);
        if(!bs->is_find(sink)) break;
        total+=set_augmental_path();
    }
    return total;
}

#endif

