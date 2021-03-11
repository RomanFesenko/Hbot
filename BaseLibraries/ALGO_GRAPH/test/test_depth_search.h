
#ifndef  _test_depth_search_
#define  _test_depth_search_

#include <utility>
#include <vector>

namespace dsd{

void find_articulation(const std::vector<std::pair<int,int>>&adj_lists,
                       std::vector<std::pair<int,int>>&bridges,
                       std::vector<int>& point,int isource);

bool is_acyclic(const std::vector<std::pair<int,int>>&adj_lists);
bool topological_sort(const std::vector<std::pair<int,int>>&adj_lists,
                      std::vector<int>&sorted);


void test_articulation();
void test_articulation2();
void test_acyclic();
void test_topological_sort();
}

#endif

