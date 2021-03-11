#ifndef _commands_
#define _commands_

#include<map>
#include<string>

#include<functional>
#include "util.h"

void Cmd_time(const player_t&player);
void Cmd_my_data(const player_t&player);



std::string CheckGraphEdit(const player_t& player);
void Cmd_create_node(const player_t&player);
void Cmd_mark_node(const player_t&player);
void Cmd_delete_node(const player_t&player);
void Cmd_mark_edge(const player_t&player);
void Cmd_delete_edge(const player_t&player);

void Cmd_map_data(const player_t&player);
void Cmd_graph_data(const player_t&player);
void Cmd_save_graph(const player_t&player);

void Cmd_debug_bot(const player_t&player);
void Cmd_follow_me(const player_t&player);

void Cmd_add_bot(const player_t&player);

void Cmd_delete_item_point(const player_t&player);
void Cmd_info_item_point(const player_t&player);
void Cmd_link_item_points(const player_t&player);

using s_vvf_t=std::pair<const std::string,std::function<void(const player_t&)> >;
#define STRING_TO_COMMAND(commandstring) s_vvf_t(#commandstring,Cmd_ ## commandstring)

const std::map<const std::string,std::function<void(const player_t&)>> glCommandsMap=
{
    STRING_TO_COMMAND(time),
    STRING_TO_COMMAND(my_data),

    STRING_TO_COMMAND(create_node),
    STRING_TO_COMMAND(mark_node),
    STRING_TO_COMMAND(delete_node),
    STRING_TO_COMMAND(mark_edge),
    STRING_TO_COMMAND(delete_edge),

    STRING_TO_COMMAND(map_data),
    STRING_TO_COMMAND(graph_data),
    STRING_TO_COMMAND(save_graph),

    STRING_TO_COMMAND(debug_bot),
    STRING_TO_COMMAND(follow_me),

    STRING_TO_COMMAND(add_bot),

    STRING_TO_COMMAND(delete_item_point),
    STRING_TO_COMMAND(info_item_point),
    STRING_TO_COMMAND(link_item_points)
};

inline player_t* glMe=nullptr;

void Cmd_set_direct_edge(const player_t&player,const std::vector<int>&);
void Cmd_set_bidirect_edge(const player_t&player,const std::vector<int>&);
void Cmd_graph_edit(const player_t&player,const std::vector<int>&);
void Cmd_graph_build(const player_t&player,const std::vector<int>&);
void Cmd_test_path(const player_t&player,const std::vector<int>&);
void Cmd_test_path_to_corner(const player_t&player,const std::vector<int>&);
void Cmd_create_item_point(const player_t&player,const std::vector<int>&);

void Cmd_graph_test(const player_t&player,const std::vector<int>&);

void Cmd_set_visible(const player_t&player,const std::vector<int>&);
void Cmd_set_difficult(const player_t&player,const std::vector<int>&);


using s_vintf_t=std::pair<const std::string,std::function<void(const player_t&,std::vector<int>&)> >;
#define PARSE_TO_COMMAND(commandstring) s_vintf_t(#commandstring,Cmd_ ## commandstring)

const std::map<const std::string,std::function<void(const player_t&,std::vector<int>&)>> glVarCommandsMap=
{
    PARSE_TO_COMMAND(set_direct_edge),
    PARSE_TO_COMMAND(set_bidirect_edge),
    PARSE_TO_COMMAND(graph_edit),
    PARSE_TO_COMMAND(graph_build),
    PARSE_TO_COMMAND(test_path),
    PARSE_TO_COMMAND(test_path_to_corner),
    PARSE_TO_COMMAND(create_item_point),
    PARSE_TO_COMMAND(graph_test),
    PARSE_TO_COMMAND(set_visible),
    PARSE_TO_COMMAND(set_difficult)
};

/*enum GlobalStates
{
    id_dormant_st,
    id_graph_edit_st,
    id_graph_test_st,
    id_bot_run_st
};
inline GlobalStates glState;*/


bool ParseCommand(const std::string&str_server);

#endif //
