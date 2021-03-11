
#include "test_common.h"
#include "test_flow.h"
#include "../edmonds_karp.h"
#include "../relabel_to_front.h"

#include <iostream>

//#include <assert.h>

//Kormen
void test_flow()
{
    using ipair=std::pair<int ,int>;
    struct pipe_t
    {
        int from,to,cap_1_2,cap_2_1;
    };
    const int num_nodes=6;
    std::vector<pipe_t> pipes=
    {
        {0,1,16,0},
        {0,2,13,0},
        {1,2,10,4},
        {1,3,12,0},
        {2,3,0,9},
        {2,4,14,0},
        {3,4,0,7},
        {3,5,20,0},
        {4,5,4,0}
    };
    network_t<int > network{};
    network.build(pipes.begin(),pipes.end(),num_nodes);

    TEST(ff_min_path(network,0,5)==23); network.to_empty();
    TEST(ff_max_flow(network,0,5)==23); network.to_empty();
    TEST(relabel_to_front(network,0,5)==23); network.to_empty();

    TEST(network.residual_capacities(0,1)==ipair(16,0));
    TEST(network.residual_capacities(0,2)==ipair(13,0));
    TEST(network.residual_capacities(4,5)==ipair(4,0));

    for(int i=0;i<num_nodes;++i)
    {
        for(int j=i+1;j<num_nodes;++j)
        {
            TEST(network.flow(i,j)==0);
        }
    }
    network.destroy();
}


//Kristofides
void test_flow2()
{
    struct pipe_t
    {
        int from,to,cap_1_2,cap_2_1;
    };
    std::vector<pipe_t> pipes=
    {
        {0,1,8,8},
        {0,4,7,7},
        {0,5,11,11},

        {1,2,12,12},
        {1,3,10,10},
        {1,4,3,3},

        {2,3,9,9},
        {2,5,2,2},

        {3,4,5,5},
        {3,5,6,6},

        {4,5,4,4},
    };
    struct flows_t
    {
        int from,to,flow;
    };
    const std::vector<flows_t> max_flows=
    {
        {0,1,24},
        {0,2,23},
        {0,3,24},
        {0,4,19},
        {0,5,23},
        {1,2,23},
        {1,3,30},
        {1,4,19},
        {1,5,23},
        {2,3,23},
        {2,4,19},
        {2,5,23},
        {3,4,19},
        {3,5,23},
        {4,5,19}
    };
    network_t<int > network{};
    network.build(pipes.begin(),pipes.end(),6);

    for(const auto&mf:max_flows)
    {
        TEST(ff_min_path(network,mf.from,mf.to)==mf.flow);
        network.to_empty();

        TEST(relabel_to_front(network,mf.from,mf.to)==mf.flow);
        network.to_empty();

        TEST(ff_max_flow(network,mf.from,mf.to)==mf.flow);
        network.to_empty();
    }
    network.destroy();
}
