
#include "test_common.h"
#include "test_min_spanning_tree.h"
#include "../graph_composer.h"
#include "../graph_generator.h"
#include "../kruskal.h"
#include "../prim.h"


#include <iostream>
#include <iterator>
#include <vector>
#include <array>

//#include <assert.h>
namespace mstd{

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

int mst_kruskal(const std::vector<std::array<int,3>>&lv)
{
    CGraph graph(lv,undirected_tag{});
    return kruskal(graph.Nodes());
}

int mst_prim(const std::vector<std::array<int,3>>&lv)
{
    CGraph graph(lv,undirected_tag{});
    prim_process<some_node_t> pp(graph.Nodes());
    return pp.build_spanning_tree(*graph.Nodes()[0]);
}

void test_min_spanning_tree()
{
    //Kormen 645
    const std::vector<std::array<int,3>> lv=
    {
        {0,1,4},
        {0,2,8},
        {1,2,11},
        {1,4,8},
        {2,3,7},
        {2,5,1},
        {3,4,2},
        {3,5,6},
        {4,6,7},
        {4,7,4},
        {5,7,2},
        {6,7,14},
        {6,8,9},
        {7,8,10}
    };
    TEST(mst_kruskal(lv)==37);
    TEST(mst_prim(lv)==37);
    //test random graph
    std::vector<std::array<int,3>> random_lv;
    graph_traits_t<int> gt;
    gt.nodes=50;
    gt.edges=500;
    gt.min_weight=1;
    gt.max_weight=100;
    generate_weighted_graph(gt,std::back_inserter(random_lv));
    int rg_mst=mst_kruskal(random_lv);
    TEST(rg_mst!=0);
    TEST(rg_mst==mst_prim(random_lv));
}

}//mstd
