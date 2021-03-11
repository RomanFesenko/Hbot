
#ifndef  _articulation_point_
#define  _articulation_point_

#include <assert.h>

#include "depth_search.h"

template<class node_t,class ap_handler_t>
class ap_finder_t:public default_ds_visitor_t<node_t,ds_node_close
                                                    |ds_node_discover
                                                    |ds_tree_edge
                                                    |ds_nontree_edge>
{
    using iterator=node_t::iterator;
    const depth_search<node_t>& m_ds;
    const node_t& m_source;
    ap_handler_t m_ap_handler;
    // most senior ancestor of node reachable
    // from a node along forward and backward edges
    TProperty <const node_t*,node_t> m_reachable_ancestor;
    // degree of node in depth search tree
    TProperty <int,node_t> m_tree_out_degree;
    public:
    explicit ap_finder_t(const depth_search<node_t>&ds,
                         const node_t& source,
                         ap_handler_t ap_handler):
    m_ds(ds),
    m_source(source),
    m_ap_handler(ap_handler),
    m_reachable_ancestor(ds.nodes().size()),
    m_tree_out_degree(ds.nodes().size())
    {
        std::fill(m_tree_out_degree.begin(),m_tree_out_degree.end(),0);
    }

    void node_discover(const node_t&node)
    {
        m_reachable_ancestor(node)=&node;
    }
    void node_close(const node_t&node)
    {
        if(&node==&m_source)
        {
            if(m_tree_out_degree(node)>1)
            {
                m_ap_handler.articulation(node);
            }
            return;
        }
        bool is_parent_root=(&m_ds.parent(node)==&m_source);
        if(m_reachable_ancestor(node)==&m_ds.parent(node)&&!is_parent_root)
        {
            m_ap_handler.articulation(m_ds.parent(node));
        }
        if(m_reachable_ancestor(node)==&node)
        {
            if(!is_parent_root)
            {
                m_ap_handler.articulation(m_ds.parent(node));
            }
            m_ap_handler.bridge(node,m_ds.parent(node));
            if(m_tree_out_degree(node)>1)
            {
                m_ap_handler.articulation(node);
            }
        }
        const node_t&parent= m_ds.parent(node);
        int time_node=m_ds.entry_time(*m_reachable_ancestor(node));
        int time_parent=m_ds.entry_time(*m_reachable_ancestor(parent));
        if(time_node<time_parent)
        {
            m_reachable_ancestor(parent)=m_reachable_ancestor(node);
        }
    }
    void tree_edge_process(const iterator& iter)
    {
        ++m_tree_out_degree(iter.from_node());
    }
    void nontree_edge_process(const iterator& iter)
    {
        const node_t&from=iter.from_node();
        const node_t&to=iter.to_node();
        if(m_ds.entry_time(to)<
           m_ds.entry_time(*m_reachable_ancestor(from)))
        {
            m_reachable_ancestor(from)=&to;
        }
    }
};

#endif

