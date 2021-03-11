
#ifndef  _graph_generator_
#define  _graph_generator_
#include <assert.h>


#include <utility>
#include <algorithm>
#include <type_traits>
#include <vector>
#include <optional>
#include <random>


template<class weight_t>
auto _get_random_generator(weight_t min,weight_t max,
                          std::default_random_engine&dre)
{
    if constexpr(std::is_floating_point_v<weight_t>)
    {
        std::uniform_real_distribution<weight_t> urd(min,max);
        return [urd,&dre]()mutable{return urd(dre);};
    }
    else
    {
        std::uniform_int_distribution<weight_t> urd(min,max);
        return [urd,&dre]()mutable{return urd(dre);};
    }
}

// weighted graph traits
template<class weight_t>
struct graph_traits_t
{
    int nodes;
    int edges;
    weight_t min_weight;
    weight_t max_weight;
};

// Generation sparse connected undirected weighted graph
// with graph_traits_t conditions
template<class weight_t,class out_iterator>
void generate_weighted_graph(const graph_traits_t<weight_t>& gt,out_iterator out)
{
    struct link_t
    {
        int from;
        int to;
        weight_t weight;
    };
    std::vector<link_t> temp_links;temp_links.reserve(gt.edges);
    int max_edge=gt.nodes*(gt.nodes-1)/2;
    assert(gt.edges<=max_edge);
    assert(gt.edges>=gt.nodes-1);

    std::default_random_engine dre;
    auto rg=_get_random_generator(gt.min_weight,gt.max_weight,dre);
    // Build exactly connected graph
    // with creation hamilton cycle
    int edge_for_build=gt.edges;
    for(int i=0;i<gt.nodes-1;++i)
    {
        temp_links.push_back({i,i+1,rg()});
    } edge_for_build-=(gt.nodes-1);

    if(edge_for_build>0)
    {
        temp_links.push_back({gt.nodes-1,0,rg()});
        edge_for_build--;
    }
    // Create residual number edges,
    // some from that maybe equal
    for(std::uniform_int_distribution<int> edge_inds(0,gt.nodes-1);
        edge_for_build>0;edge_for_build--)
    {
        int imin=edge_inds(dre);int imax =edge_inds(dre);
        if(imin==imax) continue;
        temp_links.push_back({imin,imax,rg()});
    }
    // Sorted out and remove dublicate
    auto link_compare=[](const link_t&_1,const link_t&_2)
    {
        auto _1_minmax=std::minmax(_1.from,_1.to);
        auto _2_minmax=std::minmax(_2.from,_2.to);
        return (_1_minmax.first!=_2_minmax.first)?
                _1_minmax.first<_2_minmax.first  :
                _1_minmax.second<_2_minmax.second;
    };
    auto link_equal=[](const link_t&_1,const link_t&_2)
    {
        auto _1_minmax=std::minmax(_1.from,_1.to);
        auto _2_minmax=std::minmax(_2.from,_2.to);
        return (_1_minmax.first==_2_minmax.first)&&
                _1_minmax.second==_2_minmax.second;
    };

    std::sort(temp_links.begin(),temp_links.end(),link_compare);
    //nodes pair maybe connected not greater by one edge
    auto new_end=std::unique(temp_links.begin(),temp_links.end(),link_equal);
    temp_links.erase(new_end,temp_links.end());
    for(auto&link:temp_links)
    {
        *out={link.from,link.to,link.weight};
        ++out;
    }
}

#endif

