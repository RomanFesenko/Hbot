
#ifndef  _test_min_spanning_tree_
#define  _test_min_spanning_tree_

#include <vector>
#include <array>


namespace mstd{

int mst_kruskal(const std::vector<std::array<int,3>>&);

int mst_prim(const std::vector<std::array<int,3>>&);

void test_min_spanning_tree();
}
#endif

