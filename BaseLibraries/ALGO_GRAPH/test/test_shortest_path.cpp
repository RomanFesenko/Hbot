
#include "test_common.h"
#include "test_shortest_path.h"
#include "../graph_composer.h"
#include "../dijkstra.h"
#include "../floyd_warshall.h"
#include "../bellman_ford.h"
#include "../graph_generator.h"


#include <iostream>
#include <iterator>
#include <vector>
#include <array>

//#include <assert.h>
namespace spd{

class edge_t
{
    int m_weight;
    public:
    explicit edge_t(int weight):
    m_weight(weight)
    {}
    int weight()const{return m_weight;};
};
struct some_node_t:public CBaseNode<edge_t,some_node_t>
{};

struct undirected_tag{};
struct directed_tag{};


struct CGraph
{
    CGraphComposer<some_node_t> m_gc;
    public:
    template<class link_t>
    CGraph(const std::vector<link_t>& lv,undirected_tag)
    {
       int num=0;
       for(const auto&[from,to,weight]:lv )
       {
           num=std::max({from,to,num});
       }
       num++;
       m_gc.create_nodes(num);
       for(const auto&[from,to,weight]:lv )
       {
           m_gc.set_undirect_edge(*m_gc.nodes()[from],*m_gc.nodes()[to],new edge_t(weight));
       }
    }
    template<class link_t>
    CGraph(const std::vector<link_t>& lv,directed_tag)
    {
       int num=0;
       for(const auto&[from,to,weight]:lv )
       {
           num=std::max({from,to,num});
       }
       num++;
       m_gc.create_nodes(num);
       for(const auto&[from,to,weight]:lv )
       {
           m_gc.set_direct_edge(*m_gc.nodes()[from],*m_gc.nodes()[to],new edge_t(weight));
       }
    }
    const std::vector<some_node_t*>& Nodes()const
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

void test_shortest_path_impl(const std::vector<std::array<int,3>>&lv,
                             const std::vector<std::vector<int>>& sp)
{
    CGraph  graph(lv,directed_tag{});
    const int num_nodes=graph.Nodes().size();
    //test dijkstra
    dijkstra_process<some_node_t> dp(graph.Nodes());
    struct find_all_t:public search_restrictor<has_node_finish>
    {
        bool is_node_finish(const some_node_t& n){return false;}
    };
    find_all_t find_all{};
    //test floyd-warshall
    floyd_warshall_process<some_node_t> fwp(graph.Nodes());
    fwp.done();
    // test bellman-ford
    bellman_ford_process<some_node_t> bfp(graph.Nodes());
    for(int i=0;i<num_nodes;++i)
    {
        const some_node_t&from=*graph.Nodes()[i];
        dp.restricted_find_path(from,find_all);
        TEST(bfp.done(from));
        for(int j=0;j<num_nodes;++j)
        {
            const some_node_t&to=*graph.Nodes()[j];
            TEST(dp.is_find(to));
            TEST(fwp.is_path(from,to));
            TEST(bfp.is_find(to));
            TEST(sp[i][j]==dp.weight_result(to));
            TEST(sp[i][j]==fwp.weight_result(from,to));
            TEST(sp[i][j]==bfp.weight_result(to));
        }
    }
}

void test_shortest_path_impl(const std::vector<std::array<int,3>>&lv)
{
    CGraph  graph(lv,directed_tag{});
    const int num_nodes=graph.Nodes().size();
    //test dijkstra
    dijkstra_process<some_node_t> dp(graph.Nodes());
    struct find_all_t:public search_restrictor<has_node_finish>
    {
        bool is_node_finish(const some_node_t& n){return false;}
    };
    find_all_t find_all{};
    //test floyd-warshall
    floyd_warshall_process<some_node_t> fwp(graph.Nodes());
    fwp.done();
    // test bellman-ford
    bellman_ford_process<some_node_t> bfp(graph.Nodes());
    for(int i=0;i<num_nodes;++i)
    {
        const some_node_t&from=*graph.Nodes()[i];
        dp.restricted_find_path(from,find_all);
        TEST(bfp.done(from));
        for(int j=0;j<num_nodes;++j)
        {
            const some_node_t&to=*graph.Nodes()[j];
            TEST(dp.is_find(to));
            TEST(fwp.is_path(from,to));
            TEST(bfp.is_find(to));
            TEST(fwp.weight_result(from,to)==dp.weight_result(to));
            TEST(fwp.weight_result(from,to)==bfp.weight_result(to));
        }
    }
}

void test_bellman_ford(const std::vector<std::array<int,3>>&lv,
                       const std::vector<std::vector<int>>&sp)
{
    CGraph  graph(lv,directed_tag{});
    const int num_nodes=graph.Nodes().size();

    bellman_ford_process<some_node_t> bfp(graph.Nodes());
    for(int i=0;i<num_nodes;++i)
    {
        const some_node_t&from=*graph.Nodes()[i];
        TEST(bfp.done(from));
        for(int j=0;j<num_nodes;++j)
        {
            const some_node_t&to=*graph.Nodes()[j];
            TEST(bfp.is_find(to));
            TEST(sp[i][j]==bfp.weight_result(to));
        }
    }
}

void test_shortest_path()
{
    // Sedjvik 292
    const std::vector<std::array<int,3>> lv=
    {
        {0,1,41},
        {0,5,29},
        {1,2,51},
        {1,4,32},
        {2,3,50},
        {3,0,45},
        {3,5,38},
        {4,2,32},
        {4,3,36},
        {5,1,29},
        {5,4,21}
    };
    const std::vector<std::vector<int>> sp=
    {
        {0,41,82,86,50,29},
        {113,0,51,68,32,106},
        {95,117,0,50,109,88},
        {45,67,91,0,59,38},
        {81,103,32,36,0,74},
        {102,29,53,57,21,0}
    };
    test_shortest_path_impl(lv,sp);

    // Sedjvik 341
    const std::vector<std::array<int,3>> lv_neg=
    {
        {0,1,41},
        {0,5,29},
        {1,2,51},
        {1,4,32},
        {2,3,50},
        {3,0,45},
        {3,5,-38},//
        {4,2,32},
        {4,3,36},
        {5,1,-29},//
        {5,4,21}
    };
    const std::vector<std::vector<int>> sp_neg=
    {
        {0,0,51,68,32,29},
        {113,0,51,68,32,30},
        {95,-17,0,50,15,12},
        {45,-67,-16,0,-35,-38},
        {81,-31,20,36,0,-2},
        {84,-29,22,39,3,0}
    };
    test_bellman_ford(lv_neg,sp_neg);

    //test for random graph
    std::vector<std::array<int,3>> random_lv;
    graph_traits_t<int> gt;
    gt.nodes=50;
    gt.edges=500;
    gt.min_weight=1;
    gt.max_weight=100;
    generate_weighted_graph(gt,std::back_inserter(random_lv));
    test_shortest_path_impl(random_lv);
}

}//spd
