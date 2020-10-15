#ifndef _parse_config_
#define _parse_config_

#include <vector>
#include <utility>
#include <string>

using game_config_t=std::vector<std::tuple<std::string,float,float>>;


bool FileToString(const std::string&,std::string&);
void ParseConfig(game_config_t&);

#endif






