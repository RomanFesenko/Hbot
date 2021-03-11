
#ifndef  _test_shortest_path_
#define  _test_shortest_path_

#include <vector>
#include <array>


namespace spd{

void test_shortest_path_impl(const std::vector<std::array<int,3>>&,
                             const std::vector<std::vector<int>>&);

void test_shortest_path_impl(const std::vector<std::array<int,3>>&);

void test_bellman_ford(const std::vector<std::array<int,3>>&,
                             const std::vector<std::vector<int>>&);

void test_shortest_path();
}
#endif

