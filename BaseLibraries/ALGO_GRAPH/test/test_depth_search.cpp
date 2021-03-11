#include <algorithm>

#include "test_common.h"
#include "test_depth_search.h"
#include "../graph_composer.h"

#include "../articulation_point.h"


#include <iostream>

//#include <assert.h>

namespace dsd{

struct edge_t
{
    int weight()const{return 1;};
};
struct node_t:public CBaseNode<edge_t,node_t>
{};

struct undirected_tag{};
struct directed_tag{};


struct CGraph
{
    CGraphComposer<node_t> m_gc;
    public:
    CGraph(const std::vector<std::pair<int,int>>& lv,undirected_tag ut=undirected_tag{})
    {
       int num=0;
       for(auto&[from,to]:lv )
       {
           num=std::max({num,from,to});
       }
       num++;
       m_gc.create_nodes(num);
       for(auto&[_1,_2]:lv)
       {
           m_gc.set_undirect_edge(*m_gc.nodes()[_1],*m_gc.nodes()[_2],new edge_t());
       }
    }
    CGraph(const std::vector<std::pair<int,int>>& lv,directed_tag)
    {
       int num=0;
       for(auto&[from,to]:lv )
       {
           num=std::max({num,from,to});
       }
       num++;
       m_gc.create_nodes(num);
       for(auto&[_1,_2]:lv)
       {
           m_gc.set_direct_edge(*m_gc.nodes()[_1],*m_gc.nodes()[_2],new edge_t());
       }
    }
    const std::vector<node_t*>& Nodes()const
    {
        return m_gc.nodes();
    }
    void DeleteAllEdges()
    {
        m_gc.delete_all_edges();
    }

   ~CGraph()
   {
       m_gc.delete_graph();
   }
};

void find_articulation(const std::vector<std::pair<int,int>>&adj_lists,
                       std::vector<std::pair<int,int>>&bridges,
                       std::vector<int>& articulations,int isource)
{
    bridges.clear();articulations.clear();
    CGraph graph(adj_lists);
    depth_search ds(graph.Nodes());

    using ipair=std::pair<int,int>;
    struct handler_t
    {
        std::vector<ipair>&m_bridges;
        std::vector<int>& m_articulations;
        handler_t(std::vector<ipair>&_bridges,std::vector<int>& _articulations):
        m_bridges(_bridges),m_articulations(_articulations){}
        void articulation(const node_t&node)
        {
            m_articulations.push_back(node.index());
        }
        void bridge(const node_t&_1,const node_t&_2)
        {
            m_bridges.push_back(std::minmax(_1.index(),_2.index()));
        }
    };
    handler_t handler(bridges,articulations);

    ap_finder_t ap_finder(ds,*graph.Nodes()[isource],handler);
    ds.done(*graph.Nodes()[isource],ap_finder);
    std::sort(articulations.begin(),articulations.end());
    std::sort(bridges.begin(),bridges.end());
}

bool is_acyclic(const std::vector<std::pair<int,int>>&adj_lists)
{
    CGraph graph(adj_lists,directed_tag{});
    depth_search ds(graph.Nodes());

    struct cycle_finder_t:public default_ds_visitor_t<node_t,
                                                      ds_nontree_edge>
    {
        bool is_cycle=false;
        const depth_search<node_t>&m_ds;
        explicit cycle_finder_t(const depth_search<node_t>&ds):
        m_ds(ds){}

        void nontree_edge_process(const iterator&iter)
        {
            if(is_cycle) return;
            is_cycle=m_ds.state(iter.to_node())==depth_search<node_t>::Alive;
        }
    };
    cycle_finder_t cf(ds);
    ds.done(*graph.Nodes()[0],cf);
    return !cf.is_cycle;
}

bool topological_sort(const std::vector<std::pair<int,int>>&adj_lists,
                      std::vector<int>&sorted)
{
    if(!is_acyclic(adj_lists)) return false;
    sorted.clear();

    CGraph graph(adj_lists,directed_tag{});
    depth_search ds(graph.Nodes());

    struct top_sorter_t:public default_ds_visitor_t<node_t,
                                                      ds_node_close>
    {
        std::vector<int>&m_sorted;
        explicit top_sorter_t(std::vector<int>& vec):
        m_sorted(vec)
        {}
        void node_close(const node_t&node)
        {
            m_sorted.push_back(node.index());
        }
    };
    top_sorter_t ts(sorted);
    ds.done(*graph.Nodes()[0],ts);
    std::reverse(sorted.begin(),sorted.end());
    return true;
}

/// Check articulation

// Skiena 195
void test_articulation()
{
    using ipair=std::pair<int,int>;
    const std::vector<std::pair<int,int>> lv=
    {
        {0,1},{0,4},{0,5},{1,2},{1,4},{2,3},{3,4}
    };
    std::vector<ipair> bridges;
    std::vector<int> articulations;
    const std::vector<ipair> true_bridges={{0,5}};
    const std::vector<int> true_articulations={0};
    //asserts
    for(int i=0;i<6;++i)
    {
        find_articulation(lv,bridges,articulations,i);
        TEST(true_articulations==articulations);
        TEST(true_bridges==bridges);
    }
}

//Okulov 155
void test_articulation2()
{
    using ipair=std::pair<int,int>;
    const std::vector<std::pair<int,int>> lv=
    {
        {0,1},{0,2},
        {1,3},
        {2,3},
        {3,4},{3,5},{3,6},
        {4,5},{4,7},
        {5,6},
        {6,8}
    };
    std::vector<ipair> bridges;
    std::vector<int> articulations;

    const std::vector<ipair> true_bridges={{4,7},{6,8}};
    const std::vector<int> true_articulations={3,4,6};

    //asserts
    for(int i=0;i<9;++i)
    {
        find_articulation(lv,bridges,articulations,i);
        TEST(true_articulations==articulations);
        TEST(true_bridges==bridges);
    }
}

/// Check acyclic

// Sedjvic 199
void test_acyclic()
{
    const std::vector<std::pair<int,int>> ac_lv=
    {
        {0,1},{0,2},{0,3},{0,6},{0,8},
        {2,3},
        {3,7},{3,8},
        {4,5},
        {5,6},
        {6,7},{6,9},
        {7,9},
        {9,10},{9,11},{9,12},
        {11,12}
    };
    const std::vector<std::pair<int,int>> c_lv=
    {
        {0,1},
        {1,2},
        {2,3},
        {3,0}
    };
    TEST(is_acyclic(ac_lv));
    TEST(!is_acyclic(c_lv));
}

/// Check topological sort

void test_topological_sort()
{
    const std::vector<std::pair<int,int>> ac_lv=
    {
        {0,1},{0,2},
        {1,3},
        {2,1},{2,3}
    };
    const std::vector<int> true_top_sort={0,2,1,3};
    std::vector<int> top_sort;
    TEST(topological_sort(ac_lv,top_sort));
    TEST(top_sort==true_top_sort);
}
}// dsd
