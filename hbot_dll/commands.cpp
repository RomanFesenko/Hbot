#include "commands.h"
#include "graph_editor.h"
#include "bot_fight.h"
#include "items_data.h"


#include <regex>

using std::string;
using std::regex;
using std::regex_search;
using std::regex_match;
using std::smatch;
using std::sregex_token_iterator;

CBot*glDebugBot=nullptr;

void BotThink()
{
    if constexpr(BOT_DEBUG)
    {
        if(glDebugBot) {glDebugBot->Think();}
    }
    glBots.DoThink();
}

string report(const string& _rep)
{
    return (_rep=="")? "success":_rep;
}

const auto glNumberReg=regex("[[:digit:]]+");

// longest sequence of letters and _
// ending in _

const auto glPrefixCmdReg=regex("[[:alpha:]_]+_");

void ServerDeactivate()
{
    ReportPrint("Server deactivate...\n");
    glMapGraphViewer.HideAll();

    if constexpr(BOT_DEBUG)
    {
        if(glDebugBot) delete glDebugBot;
    }

    glBots.Clear();
    glMapGraphEditor.Close();
    glMapGraph.Destroy();
    if(glMe!=nullptr)
    {
        delete glMe;
        glMe=nullptr;
    }
    assert(glCallbacks.size()==0);
}

bool ServerCommand(const char*from_server)
{
    if(glMe==nullptr)
    {
        std::vector<player_t> plrs;
        PlayersInGame(plrs);
        glMe=new player_t(plrs[0]);
        SetCallBack(0.1,CallbacksRun);
    }
    auto iter=glCommandsMap.find(from_server);
    if(iter!=glCommandsMap.end())
    {
        (iter->second)(*glMe);
        return true;
    }
    return ParseCommand(from_server);
}

void Cmd_time(const player_t&player)
{
    ServerPrint("Time: "+std::to_string(Time())+"\n");
}

void Cmd_my_data(const player_t&player)
{
    ServerPrint("Center:"+ToString(player.Center())+'\n');
    ServerPrint("Eye:"+ToString(player.Eyes())+'\n');
    ServerPrint("View angles:"+ToString(player.ViewAngle())+'\n');
    ServerPrint("Health"+std::to_string(player.Health())+'\n');
    ServerPrint("Armour"+std::to_string(player.Armour())+'\n');
    ServerPrint("Active item: "+get_string(player.ActiveItem())+'\n');

    ServerPrint("Weapons:\n");
    generate_flags weap(player.AllItems()&Weapon);
    for(weap.begin();!weap.end();weap.next())
    {
        ServerPrint(get_string(weap)+'\n');
    }
    ServerPrint("Ammo:\n");
    generate_flags ammo(Ammunition);
    for(ammo.begin();!ammo.end();ammo.next())
    {
        ServerPrint(get_string(ammo)+": "+
                    std::to_string(player.Ammo(ammo))+
                    '\n');
    }
}


void Cmd_create_node(const player_t&player)
{
    string rep=glMapGraphEditor.CreateWP(player.Center());
    ServerPrint("Create node: "+report(rep)+'\n');
}

void Cmd_mark_node(const player_t&player)
{
    string rep=glMapGraphEditor.MarkWP(player.Center());
    ServerPrint("Mark node: "+report(rep)+'\n');

}

void Cmd_delete_node(const player_t&player)
{
    string rep=glMapGraphEditor.DeleteMarkedWP();
    ServerPrint("Delete mark node: "+report(rep)+'\n');
}

void Cmd_delete_item_point(const player_t&player)
{
    string rep=glMapGraphEditor.DeleteItemPoint(player.Center());
    ServerPrint("Delete item point: "+report(rep)+'\n');
}

void Cmd_mark_edge(const player_t&player)
{
    string rep=glMapGraphEditor.MarkLink(player.Center());
    ServerPrint("Mark edge: "+report(rep)+'\n');
}

void Cmd_delete_edge(const player_t&player)
{
    string rep=glMapGraphEditor.DeleteMarkedEdge();
    ServerPrint("Delete mark edge: "+report(rep)+'\n');
}

void Cmd_map_data(const player_t&player)
{
    ServerPrint("Name: "+MapName()+"\n");

    ServerPrint("-------ITEMS-------- \n");
    std::vector<std::pair<int,Vector>> items;
    AllItemsInMap(items);
    for(const auto& [id,vec]:items)
    {
        ServerPrint(get_string(id)+"\n");
    }
}


void Cmd_graph_data(const player_t&player)
{
    if(!glMapGraph.IsInit())
    {
        ServerPrint("graph not valid \n");
        return;
    }
    ServerPrint("Number nodes: "+
                std::to_string(glMapGraph.NumWP())+"\n");

    ServerPrint("Number edges: "+
                std::to_string(glMapGraph.Composer.all_edges())+"\n");
    ServerPrint("-------LINKED  ITEMS-------- \n");
    for(const auto& item:glMapGraph.ItemPoints())
    {
        ServerPrint(get_string(item->Type())+"\n");
    }
}

void Cmd_save_graph(const player_t&player)
{
    string rep=glMapGraphEditor.SaveGraph();
    ServerPrint("Save graph: "+report(rep)+'\n');
}

void Cmd_debug_bot(const player_t&player)
{
    if(glCallbacks.size()!=0)
    {
        ServerPrint("Impossible run bot while other theme running \n");
        return;
    }
    if(!glMapGraph.IsInit())
    {
        if(!LoadMapGraph(glMapGraph,MapName()))
        {
            ServerPrint("Not valid way points file \n");
            return;
        }
    }
    if(glDebugBot==nullptr)
    {
        glDebugBot=new CBot("Debug_bot",0,0);
        glCallbacks.AddCallback(CCallbacks::id_bot_run_cb,
        {
            0.05,
            BotThink
        });
    }
    else
    {
        ServerPrint("Bot already running \n");
        return;
    }
}

void Cmd_follow_me(const player_t&player)
{
    if(glDebugBot==nullptr)
    {
        ServerPrint("Not debug bot \n");
        return;
    }
    CBaseRoute* base_route=new CBaseRoute;
    auto route_res=RouteToVector(*glDebugBot,glDebugBot->Center(),glMe->Center());
    base_route->Set(route_res);
    glDebugBot->SetExecuteTask(base_route);
}


void Cmd_info_item_point(const player_t&player)
{
    for(const auto&item:glMapGraph.ItemPoints())
    {
        if(distance_3D(item->Coord()-player.Center())<200.0f)
        {
            ServerPrint("Item: "+get_string(item->Type())+'\n');
        }
    }
}

void Cmd_link_item_points(const player_t&player)
{
    string rep=glMapGraphEditor.LinkItemPoints();
    ServerPrint("Link item points: "+report(rep)+'\n');
}

// parsing strings like "_str_ng_234_344_9_34 _..."
// extracting all numbers and strings - commands

bool ParseCommand(const string&str_server)
{
    smatch matcher;
    if(!regex_search(str_server,matcher,glPrefixCmdReg))
    {
        ServerPrint("Parse error in command \n");
        return false;
    }

    string pref=matcher[0];pref.pop_back();

    string suffix=string(str_server.begin()+pref.size()+1,
                         str_server.end());

    if(suffix.empty())
    {
        ServerPrint("Void arguments list \n");
        return false;
    }

    regex sepr("_+");
    sregex_token_iterator parse(suffix.begin(),suffix.end(),
                                sepr,-1);
    sregex_token_iterator end_iter;
    std::vector<int> args;
    for(;parse!=end_iter;++parse)
    {
        if(!regex_match(string(*parse),glNumberReg))
        {
            ServerPrint("Parse error in argument \n");
            return false;
        }
        args.push_back(std::stoi(*parse));
    }

    auto iter=glVarCommandsMap.find(pref);
    if(iter!=glVarCommandsMap.end())
    {
        (iter->second)(*glMe,args);
        return true;
    }
    return false;
}

void Cmd_create_item_point(const player_t&player,const std::vector<int>&args)
{
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments \n");
        return;
    }
    const string& item_name=get_string(args[0]);
    if(item_name==glVoidString)
    {
        ServerPrint("Unknown item identifier \n");
        return;
    }
    string rep=glMapGraphEditor.CreateItemPoint(player.Center(),
                                               player.ViewAngle(),
                                               args[0]);
    ServerPrint("Create "+item_name+": "+report(rep)+'\n');
}

void Cmd_set_direct_edge(const player_t&player,const std::vector<int>&args)
{
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments \n");
        return;
    }
    string rep=glMapGraphEditor.SetEdge(args[0],-1);
    ServerPrint("Set direct edge: "+report(rep)+'\n');
}


void Cmd_set_bidirect_edge(const player_t&player,const std::vector<int>&args)
{
    string rep;
    if(args.size()==1)
    {
        rep=glMapGraphEditor.SetEdge(args[0],args[0]);
    }
    else if(args.size()==2)
    {
        rep=glMapGraphEditor.SetEdge(args[0],args[1]);
    }
    else
    {
        ServerPrint("Wrong arguments \n");
        return;
    }
    ServerPrint("Set bidirect edge: "+report(rep)+'\n');
}


void Cmd_graph_edit(const player_t&player,const std::vector<int>& args)
{
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments \n");
        return;
    }
    string rep;
    if(args[0]!=0)
    {
        rep=glMapGraphEditor.Run(static_cast<float>(args[0]),player);
        ServerPrint("Graph run: "+report(rep)+'\n');
    }
    else
    {
        rep=glMapGraphEditor.Close();
        ServerPrint("Graph close: "+report(rep)+'\n');
    }

}

void Cmd_graph_build(const player_t&player,const std::vector<int>&args)
{
    if(args.size()!=3)
    {
        ServerPrint("Wrong arguments \n");
        return;
    }
    int max_adj=(args[1]>10)? 10:args[1];

    string rest=glMapGraphEditor.
    BuildDefaultGraph(player,args[0],max_adj,to_radian(args[2]));
    if(rest=="")
    {
        ServerPrint("graph build with parameters: \n");
        ServerPrint("Max distance: "+std::to_string(args[0])+'\n');
        ServerPrint("Max adjacent: "+std::to_string(max_adj)+'\n');
        ServerPrint("Min angle: "+std::to_string(args[2])+'\n');
    }
    else
    {
        ServerPrint(rest+'\n');
    }
}

void Cmd_test_path(const player_t&player,const std::vector<int>&args)
{

    if(glCallbacks.size()!=0&&(!glCallbacks.IsCallback(CCallbacks::id_path_tracker_cb)))
    {
        ServerPrint("Other theme running \n");
        return;
    }
    if(!glMapGraph.IsInit())
    {
        if(!LoadMapGraph(glMapGraph,MapName()))
        {
            ServerPrint("No valid graph for test \n");
            return;
        }
    }
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments list \n");
        return;
    }

    glCallbacks.RemoveCallback(CCallbacks::id_path_tracker_cb);
    if(args[0]>0)
    {
        static CSearchInTransposed tr_searcher;
        const CWayPoint&dest=glMapGraph.NearSuitableWP(player.Center(),player);
        tr_searcher.RestrictedFindPathes(dest);
        CPathTrackerToWP Tracker(player,tr_searcher);

        glCallbacks.AddCallback(CCallbacks::id_path_tracker_cb,
         {
             1.5f,
             Tracker
         });
    }
}

void Cmd_test_path_to_corner(const player_t&player,const std::vector<int>&args)
{
    if(glCallbacks.size()!=0&&(!glCallbacks.IsCallback(CCallbacks::id_path_tracker_cb)))
    {
        ServerPrint("Other them running \n \n");
        return;
    }
    if(!glMapGraph.IsInit())
    {
        if(!LoadMapGraph(glMapGraph,MapName()))
        {
            ServerPrint("No valid graph for test \n");
            return;
        }
    }

    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments list \n");
        return;
    }

    glCallbacks.RemoveCallback(CCallbacks::id_path_tracker_cb);
    if(args[0]>0)
    {
        CPathTracker Tracker(player,UnvisibleNodeFinder(player.Center(),player));

        glCallbacks.AddCallback(CCallbacks::id_path_tracker_cb,
         {
             1.5f,
             Tracker
         });
    }
}

void Cmd_graph_test(const player_t&player,const std::vector<int>&args)
{
    if(!glMapGraph.IsInit())
    {
        if(!LoadMapGraph(glMapGraph,MapName()))
        {
            ServerPrint("No valid graph for test \n");
            return;
        }
    }
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments list \n");
        return;
    }
    glCallbacks.RemoveCallback(CCallbacks::id_graph_test);
    if(args[0]<=0) return;

    auto test=[cycles=args[0],&player]()
    {
        TestGraph(player,cycles);
    };
    glCallbacks.AddCallback(CCallbacks::id_graph_test,
    {
        0.05,
        test
    });
}

void Cmd_add_bot(const player_t&player)
{
    if(glCallbacks.size()!=0&&!glCallbacks.IsCallback(CCallbacks::id_bot_run_cb))
    {
        ServerPrint("Impossible run bot while other theme running \n");
        return;
    }
    if(!glMapGraph.IsInit())
    {
        if(!LoadMapGraph(glMapGraph,MapName()))
        {
            ServerPrint(string("Not valid way points file for this map\n")+
                        "Use next map:\n"+
                        "crossfire\n"+
                        "boot_camp\n"+
                        "lambda_bunker\n"+
                        "datacore\n");
            return;
        }
    }
    glBots.Add();
}


void Cmd_set_visible(const player_t&player,const std::vector<int>&args)
{
    if(glBots.Size()==0)
    {
        ServerPrint("Bots not running \n");
        return;
    }
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments list \n");
        return;
    }
    if(args[0]==0)
    {
        glBots.SetMeVisible(false);
    }
    else
    {
        glBots.SetMeVisible(true);
    }
}

void Cmd_set_difficult(const player_t&player,const std::vector<int>&args)
{
    if(glBots.Size()==0)
    {
        ServerPrint("Bots not running \n");
        return;
    }
    if(args.size()!=1)
    {
        ServerPrint("Wrong arguments list \n");
        return;
    }
    glBots.SetAccuracy(args[0]);

}
