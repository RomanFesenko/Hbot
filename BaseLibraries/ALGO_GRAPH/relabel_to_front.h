
#ifndef  _relabel_to_front_
#define  _relabel_to_front_

#include <assert.h>
#include <list>
#include "network_composer.h"
#include "property.h"


template<class flow_t>
flow_t relabel_to_front(network_t<flow_t>&network,int isource,int isink)
{
    assert(isource!=isink);
    using node_t=network_t<flow_t>::node_t;
    using iterator=node_t::iterator;
    const int inf_height=3*network.nodes().size();
    node_t&source=*network.nodes()[isource];
    node_t&sink=*network.nodes()[isink];

    //height - height of the node
    TProperty<int,node_t> height(network.nodes().size());
    // overflow - overflow in the node
    TProperty<flow_t,node_t> overflow(network.nodes().size());
    // current_it - current edge for discharge
    TProperty<iterator,node_t> current_it(network.nodes().size());

    //push_flow - push part of overflow from iter.from_node() to iter.to_node()
    auto push_flow=[&overflow](iterator& iter)
    {
        node_t& from=iter.from_node();
        node_t& to=iter.to_node();
        if(iter.capacity()<overflow(from))
        {
            flow_t min_flow=iter.capacity();
            overflow(from)-=min_flow;
            overflow(to)+=min_flow;
            iter.fill();
        }
        else
        {
            flow_t min_flow=overflow(from);
            overflow(from)=flow_t{0};
            overflow(to)+=min_flow;
            iter.add_flow(min_flow);
        }
    };

    // relabel - minimal increment height of node
    auto relabel=[&height,inf_height](node_t&node)
    {

        int min_height=inf_height;
        for(iterator iter(node);!iter.end();iter.next())
        {
            if(iter.capacity()==0){continue;}

            min_height=std::min(min_height,height(iter.to_node()));
            assert(height(iter.to_node())<inf_height);
        }
        assert(min_height!=inf_height);
        height(node)=min_height+1;
    };

    auto discharge=[&overflow,&height,&current_it,&relabel,&push_flow](node_t&node)
    {
        iterator& iter=current_it(node);
        while(overflow(node)>0)
        {
            if(iter.end())
            {
                relabel(node);
                iter.begin();
            }
            else if((iter.capacity()>flow_t{0})&&
                    //(height(node)==height(iter.to_node())+1))
                    (height(node)>height(iter.to_node())))
            {
                push_flow(iter);
            }
            else
            {
                iter.next();
            }
        }
    };

    //initialization
    for(auto node_ptr:network.nodes())
    {
        height(*node_ptr)=0;
        overflow(*node_ptr)=flow_t{0};
        current_it(*node_ptr)=iterator(*node_ptr);
    }
    height(source)=network.nodes().size();
    for(iterator iter(source);!iter.end();iter.next())
    {
        overflow(source)-=iter.capacity();
        overflow(iter.to_node())=iter.capacity();
        iter.fill();
    }
    //traversal list
    std::list<node_t*> tr_list(network.nodes().begin(),
                               network.nodes().end());
    tr_list.remove(&source); tr_list.remove(&sink);

    //main cycle
    auto sentinel=tr_list.end();
    for(auto temp=tr_list.begin();temp!=sentinel;++temp)
    {
        node_t& current_node=**temp;
        int old_height=height(current_node);
        discharge(current_node);
        if(height(current_node)>old_height)
        {
            tr_list.erase(temp);
            tr_list.push_front(&current_node);
            temp=tr_list.begin();
        }
    }
    return -overflow(source);
}



#endif

