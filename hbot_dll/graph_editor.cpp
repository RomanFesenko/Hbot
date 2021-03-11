#include "graph_editor.h"
#include <algorithm>
#include "../../BaseLibraries/GEOMETRY/primitives_3D.h"
#include "../../BaseLibraries/ALGO_GRAPH/kruskal.h"
#include "../../BaseLibraries/ALGO_EXTEND/output_iterator.hpp"

// Viewing of the graph

///CMapGraphViewer

// CMapGraphViewer :: ShowNode-drawing the graph node

void CMapGraphViewer::ShowNode(const CWayPoint& wp,int id_style)
{
    // define the drawing style by ID
    if(id_style==id_native_style) id_style=m_def_node_style;
    auto iter_style=m_styles.find(id_style);
    assert(iter_style!=m_styles.end());
    const auto&[color,width,height]=iter_style->second;

    // define what needs to be done: draw, redraw, or nothing
    auto iter=m_visible_nodes.find(&wp);

    if(iter==m_visible_nodes.end())// рисуем
    {
        m_visible_nodes.insert
        ({&wp,make_node_beam(wp,iter_style->second,id_style)});
        return;
    }
    if(iter->second.b_style==id_style) return;
    // redraw with a other style
    iter->second.b_style=id_style;
    iter->second.SetColor(color);
    iter->second.SetWidth(width);
    iter->second.SetEnds(wp.Coord(),wp.Coord()+Vector(0,0,height));
}

void CMapGraphViewer::HideNode(const CWayPoint& wp)
{
    m_visible_nodes.erase(&wp);
}

// CMapGraphViewer :: ShowNode-drawing the edge of the graph
void CMapGraphViewer::ShowEdge(const wl_iterator& wlink,int id_style)
{
    if(id_style==id_native_style)
    {
        switch(wlink.type())
        {
            case id_1_2_edge: id_style=m_def_diredge_style;
            break;
            case id_undirect_edge: id_style=m_def_undiredge_style;
            break;
            case id_bidirect_edge: id_style=m_def_multiedge_style;
            break;
            default: assert(false);
        }
    }
    // define the drawing style by ID
    auto iter_style=m_styles.find(id_style);
    assert(iter_style!=m_styles.end());
    const auto&[color,width,height]=iter_style->second;
    // define what needs to be done: draw, redraw, or nothing
    pair_nodes_t nodes_inds=to_pair(wlink);
    auto iter=m_visible_edges.find(nodes_inds);

    if(iter==m_visible_edges.end())// рисуем
    {
        m_visible_edges.insert
        ({nodes_inds,make_edge_beam(wlink,iter_style->second,id_style)});
        return;
    }
    if(iter->second.b_style==id_style) return;
    // redraw with a other style
    iter->second.b_style=id_style;
    iter->second.SetColor(color);
    iter->second.SetWidth(width);
    iter->second.SetEnds(wlink.from_node().Coord(),
    wlink.to_node().Coord());
}

void CMapGraphViewer::HideEdge(const wl_iterator& wlink)
{
    m_visible_edges.erase(to_pair(wlink));
}

// CMapGraphViewer :: ShowItem - item indication
void CMapGraphViewer::ShowItem(const CItemPoint&item,int id_style)
{
    if(id_style==id_native_style) id_style=m_def_item_style;
    auto iter_style=m_styles.find(id_style);
    assert(iter_style!=m_styles.end());

    if(m_visible_items.find(&item)==m_visible_items.end())// рисуем
    {
        m_visible_items.insert
        ({&item,make_item_beam(item,iter_style->second,id_style)});
        return;
    }
}

void CMapGraphViewer::HideItem(const CItemPoint*item)
{
    m_visible_items.erase(item);
}

void CMapGraphViewer::Show(const CMapGraph& graph)
{
    // draw edges
    drawer temp(*this);
    unique_edge_visit(graph.WayPoints(),temp);
    // draw nodes
    for(const auto& ptr_node:graph.WayPoints())
    {
        ShowNode(*ptr_node);
    }
}

// Set the type of drawing style for nodes and edges of different types

int CMapGraphViewer::SetNodeStyle(int _st)
{
    int prev=m_def_node_style;
    m_def_node_style=_st;
    return prev;
}

int CMapGraphViewer::SetDirectEdgeStyle(int _st)
{
    int prev=m_def_diredge_style;
    m_def_diredge_style=_st;
    return prev;
}

int CMapGraphViewer::SetUndirectEdgeStyle(int _st)
{
    int prev=m_def_undiredge_style;
    m_def_undiredge_style=_st;
    return prev;
}

int CMapGraphViewer::SetMultiEdgeStyle(int _st)
{
    int prev=m_def_multiedge_style;
    m_def_multiedge_style=_st;
    return prev;
}

int CMapGraphViewer::SetItemsStyle(int _st)
{
    int prev=m_def_item_style;
    m_def_item_style=_st;
    return prev;
}

void CMapGraphViewer::HideAll()
{
    m_visible_nodes.clear();
    m_visible_edges.clear();
    m_visible_items.clear();
}

// CGraphRenderer - a callback for periodic redrawing
// nodes filtered by a specific attribute.

// Needed, because HL does not allow rendering completely
// rather large graph due to restrictions on the number of Entities

void CGraphRenderer::operator()()
{
    glMapGraphViewer.HideAll();
    Rendering();
}

void CGraphRenderer::DrawNode(const CWayPoint& wpt,int style)
{
    glMapGraphViewer.ShowNode(wpt,style);
}

void CGraphRenderer::DrawEdge(const iterator& wpt,int style)
{
    glMapGraphViewer.ShowEdge(wpt,style);
}

void CGraphRenderer::DrawItem(const CItemPoint&item,int style)
{
    glMapGraphViewer.ShowItem(item,style);
}

CGraphRenderer::~CGraphRenderer()
{
    glMapGraphViewer.HideAll();
}
////////////////////////////////////////////
//   CMapGraphEditor
////////////////////////////////////////////

CMapGraphEditor::CMapGraphEditor()
{
    bool reg_res=glMapGraphViewer.RegisterStyle
    (id_first_marked_st,{Vector(200,200,200),120,100});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_second_marked_st,{Vector(200,200,200),30,100});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_marked_edge_st,{Vector(200,200,200),120,100});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_direct_edge_st,{Vector(0,255,0),30,60});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_undirect_edge_st,{Vector(255,0,0),30,60});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_bidirect_edge_st,{Vector(0,0,255),30,60});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_node_st,{Vector(255,0,0),30,60});

    reg_res=reg_res&&glMapGraphViewer.RegisterStyle
    (id_item_st,{Vector(0,0,255),30,30});

    assert(reg_res);

    glMapGraphViewer.SetNodeStyle(id_node_st);
    glMapGraphViewer.SetDirectEdgeStyle(id_direct_edge_st);
    glMapGraphViewer.SetUndirectEdgeStyle(id_undirect_edge_st);
    glMapGraphViewer.SetMultiEdgeStyle(id_bidirect_edge_st);
    glMapGraphViewer.SetItemsStyle(id_item_st);
}


std::string CMapGraphEditor::CreateWP(const Vector& pos)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }

    CWayPoint* new_wp=new CWayPoint(pos);
    glMapGraph.Composer.add_node(new_wp);
    glMapGraphViewer.ShowNode(*new_wp,id_node_st);
    glMapGraph.RemoveDijkstra();
    m_is_graph_valid=false;
    return "";
}

std::string CMapGraphEditor::CreateItemPoint(const Vector&pos,
                            const Vector&angle,
                            int type)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }
    CItemPoint* new_item=new CItemPoint(pos,type,angle);
    AddItemPoint(new_item);
    glMapGraphViewer.ShowItem(*new_item);
    return "";
}

std::string CMapGraphEditor::DeleteItemPoint(const Vector&nearest)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }
    item_ptr_t for_delete;
    auto nearest_lam=[&nearest](const item_ptr_t&current)
    {
        return distance_3D(nearest-current->Coord());
    };

    auto iter_near=ext::min_element(glMapGraph.ItemPoints().begin(),
                                    glMapGraph.ItemPoints().end(),
                                    nearest_lam);

    if(distance_3D(nearest-(*iter_near)->Coord())>100.0f)
     return "Big distance to nearest item \n";
    glMapGraphViewer.HideItem((*iter_near).get());
    RemoveItemPoint(*iter_near);
    return "";
}

std::string CMapGraphEditor::LinkItemPoints()
{
    if(!m_is_run)
    {
        return "Editor not run";
    }
    glMapGraphViewer.HideAll();
    ::LinkItemPoints();
    return "";
}

std::string CMapGraphEditor::DeleteMarkedWP()
{
    if(!m_is_run)
    {
        return "Editor not run";
    }
    if(m_first_marked==nullptr) return "No first marked";
    glMapGraphViewer.HideNode(*m_first_marked);
    iterator adj_iter;
    // hide adjacent reachable
    for(adj_iter.set_base(*m_first_marked);!adj_iter.end();adj_iter.next())
    {
        glMapGraphViewer.HideEdge(adj_iter);
    }
    // hide adjacent unreachable ones
    for(const CWayPoint* node:m_first_marked->not_availables())
    {
        auto iter=node->get_iterator_to(*m_first_marked);
        assert(iter);
        glMapGraphViewer.HideEdge(iter.value());
    }
    // delete the node itself
    glMapGraph.RemoveDijkstra();
    glMapGraph.Composer.delete_node(*m_first_marked);
    m_first_marked=m_second_marked;
    m_second_marked=nullptr;
    if(m_first_marked!=nullptr)
    {
        glMapGraphViewer.ShowNode(*m_first_marked,id_first_marked_st);
    }
    m_is_graph_valid=false;
    return "";
}

std::string CMapGraphEditor::DeleteMarkedEdge()
{
    if(!m_is_run)
    {
        return "Editor not run";
    }

    if(!m_edge_marked) return "No marked edge";

    glMapGraphViewer.HideEdge(*m_edge_marked);
    glMapGraph.RemoveDijkstra();
    glMapGraph.Composer.
    delete_edge(m_edge_marked->from_node(),m_edge_marked->to_node());

    m_edge_marked={};
    m_is_graph_valid=false;
    return "";
}

std::string CMapGraphEditor::MarkWP(const Vector& pos)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }
    auto& near_node=glMapGraph.NearWP(pos);
    if(Distance(near_node,pos)>100) return "Big distance to near node";
    if(m_first_marked==&near_node) return "";

    if(m_first_marked==nullptr){}
    else if(m_second_marked==nullptr)
    {
        m_second_marked=m_first_marked;
        glMapGraphViewer.ShowNode(*m_second_marked,id_second_marked_st);
    }
    // 2 nodes were marked
    else
    {
        glMapGraphViewer.HideNode(*m_second_marked);
        m_second_marked=m_first_marked;
        glMapGraphViewer.ShowNode(*m_second_marked,id_second_marked_st);
    }
    m_first_marked=&near_node;
    glMapGraphViewer.ShowNode(*m_first_marked,id_first_marked_st);
    return "";
}

std::string CMapGraphEditor::MarkLink(const Vector& pos)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }

    auto near_edge=glMapGraph.NearestEdge(pos,30.0f);
    if(!near_edge) return "Big distance to near edge";
    if(m_edge_marked)
    {
        glMapGraphViewer.ShowEdge(*m_edge_marked);
    }
    m_edge_marked=near_edge;
    glMapGraphViewer.ShowEdge(*m_edge_marked,id_marked_edge_st);
    return "";
}

// establishing a connection of this type between marked nodes

std::string CMapGraphEditor::SetEdge(int type_12,int type_21)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }

    if(m_first_marked==nullptr||m_second_marked==nullptr)
    return "Need two marked nodes";
    glMapGraph.RemoveDijkstra();
    auto iter=m_first_marked->get_iterator_to(*m_second_marked);
    auto rev_iter=m_second_marked->get_iterator_to(*m_first_marked);
    // hide the old edge if there was one
    if(iter)
    {
        glMapGraphViewer.HideEdge(iter.value());
    }
    else if(rev_iter)
    {
        glMapGraphViewer.HideEdge(rev_iter.value());
    }
    else{}
    if(type_12<0)
    {
        glMapGraph.SetEdge(*m_second_marked,*m_first_marked,type_21,type_12);
        glMapGraphViewer.ShowEdge(m_second_marked->get_iterator_to(*m_first_marked).value());
    }
    else
    {
        glMapGraph.SetEdge(*m_first_marked,*m_second_marked,type_12,type_21);
        glMapGraphViewer.ShowEdge(m_first_marked->get_iterator_to(*m_second_marked).value());
    }
    m_is_graph_valid=false;
    return "";
}

std::string CMapGraphEditor::Run(float max_beams,const player_t&player)
{
    if(m_is_run)
    {
        return "Editor already run";
    }
    if(glCallbacks.size()!=0)
    {
        return "Other them running";
    }
    if(!m_is_graph_valid)
    {
        m_is_graph_valid=LoadMapGraph(glMapGraph,MapName());
    }

    max_beams= std::max(max_beams,10.0f);

    CNearestRenderer redrawer(player,max_beams);
    bool add_res=glCallbacks.AddCallback(CCallbacks::id_graph_view_cb,
    {
        1.0f,// redraw with a period of 1.0 sec.
        redrawer
    });
    assert(add_res);

    m_is_run=true;
    return "";
}

std::string CMapGraphEditor::Close()
{
    if(!m_is_run)
    {
        return "Editor already close";
    }

    m_is_run=false;
    bool rem_res=glCallbacks.RemoveCallback(CCallbacks::id_graph_view_cb);
    assert(rem_res);
    glMapGraphViewer.HideAll();
    m_first_marked=nullptr;
    m_second_marked=nullptr;
    m_edge_marked={};

    if(!m_is_graph_valid)
    {
        return "Close editor without saving...";
    }
    return "";
}

std::string CMapGraphEditor::SaveGraph()
{
    if(!m_is_run)
    {
        return "Editor not run";
    }
    if(!glMapGraph.CheckConnectivity())
    {
        return "Graph not connectivity";
    }
    m_is_graph_valid=true;
    glMapGraph.InitDijkstra();

    ReportPrint("Number nodes: "+
                std::to_string(glMapGraph.NumWP())+"\n");

    ReportPrint("Number edges: "+
                std::to_string(glMapGraph.Composer.all_edges())+"\n");

    SaveMapGraph(glMapGraph,MapName());
    return "";
}

/* BuildDefaultGraph-for a given set of N nodes and constraints
   for connectivity builds a graph by default.
 Algorithm:
 1) A graph Gv is built on N - a connection is established between Ni and Nj
  if there is a line of sight between Ni and Nj.
 2) The minimal spanning tree MST is sought on the graph Gv, all
 edges not belonging to MST are removed -> we get GvMST.
 3) GvMST is saturated with edges with CMapGraph :: UpgradeAdjacency function
*/

std::string CMapGraphEditor::BuildDefaultGraph(const player_t&player,
                                        float max_dist,
                                        int max_adj,
                                        float min_angle)
{
    if(!m_is_run)
    {
        return "Editor not run";
    }

    m_first_marked=nullptr;
    m_second_marked=nullptr;
    m_edge_marked={};

    m_is_graph_valid=false;

    glMapGraph.DeleteAllEdges();
    glMapGraph.UpgradeAdjacency(fn_vis_filter(player));//Gv
    if(!glMapGraph.CheckConnectivity())
    {
        glMapGraph.DeleteAllEdges();
        return "Graph can not be connectivity";
    }

    // minimal spanning tree guarantees connectivity
    // the resulting graph
    std::vector<std::pair< CWayPoint*, CWayPoint*>> edges;
    edges.reserve(glMapGraph.NumWP());
    edges.clear();
    // save the result of the Kruskal algorithm as
    // pairs of linked nodes
    auto edge_saver=[&edges](const iterator& iter)
    {
        edges.push_back({&iter.from_node(),
                         &iter.to_node()});
    };
    kruskal(glMapGraph.WayPoints(),ext::fn_output_iterator(edge_saver));
    assert(edges.size()==glMapGraph.NumWP()-1);
    //
    glMapGraph.DeleteAllEdges();
    assert(glMapGraph.Composer.all_edges()==0);

    for(auto [from_n,to_n]:edges)
    {
        glMapGraph.SetEdge(*from_n,*to_n,0,0);
    }
    // glMapGraph==GvMST
    assert(glMapGraph.Composer.all_edges()==glMapGraph.NumWP()-1);

    // saturate the resulting spanning tree with valid edges
    auto adj_filter=[fn_vis=fn_vis_filter(player),
                     fn_dist=fn_dist_filter(max_dist),
                     fn_max_adj=fn_max_adj_filter(max_adj),
                     fn_angle=fn_angle_filter(min_angle)]

    (const CWayPoint&wp1,const CWayPoint&wp2)
    {
        return fn_vis(wp1,wp2)&&fn_dist(wp1,wp2)&&
               fn_max_adj(wp1,wp2)&&fn_angle(wp1,wp2);
    };

    glMapGraph.UpgradeAdjacency(adj_filter);

    return "";
}

/// CNearestRenderer :: Rendering
// dynamic rendering of the graph fragment nearest to the player
// using no more than max_beams beams

void CNearestRenderer::Rendering()
{
    std::vector<const CWayPoint*> nearest;
    glMapGraph.FewNearVisibleWP(m_player.Center(),
                               m_player,
                               m_max_beams,nearest);

    int temp_beams=0;
    iterator temp_iter;
    for(const CWayPoint* wpt:nearest)
    {
        if(temp_beams>=m_max_beams) break;
        DrawNode(*wpt);
        temp_beams++;

        temp_iter.set_base(*wpt);
        for(temp_iter.begin();!temp_iter.end();temp_iter.next())
        {
            if(temp_beams>=m_max_beams) break;
            DrawEdge(temp_iter);
            temp_beams++;
        }
    }

    if(glMapGraphEditor.m_first_marked!=nullptr)
    {
        DrawNode(*glMapGraphEditor.m_first_marked,CMapGraphEditor::id_first_marked_st);
    }

    if(glMapGraphEditor.m_second_marked!=nullptr)
    {
        DrawNode(*glMapGraphEditor.m_second_marked,CMapGraphEditor::id_second_marked_st);
    }
    if(glMapGraphEditor.m_edge_marked)
    {
        DrawEdge(*glMapGraphEditor.m_edge_marked,CMapGraphEditor::id_marked_edge_st);
    }
    // draw the nearest items
    for(auto&item:glMapGraph.ItemPoints())
    {
        if(distance_3D(m_player.Center()-item->Coord())<200.0f)
        {
            DrawItem(*item);
        }
    }
    if(!glMapGraph.IsInit()) return;

    // draw beam indicating the direction of view for a valid graph
    m_view_dir_1_2=nullptr;
    m_view_dir_2_1=nullptr;
    auto near_edge=glMapGraph.NearestEdge(m_player.Center(),30.0f);
    if(!near_edge) return;
    auto view_dir=glMapGraph.VisibleData().
                            GetViewing(edge_path_t(*near_edge));
    if(view_dir)
    {
        m_view_dir_1_2.reset(m_ViewDirBeam(
                        near_edge->to_node(),*view_dir));
    }

    auto reverse_edge=near_edge->get_reverse_iterator();
    if(reverse_edge)
    {
        view_dir=glMapGraph.VisibleData().
                           GetViewing(edge_path_t(*reverse_edge));
        if(view_dir)
        {
            m_view_dir_2_1.reset(m_ViewDirBeam(
                           reverse_edge->to_node(),*view_dir));
        }
    }
}

beam_t* CNearestRenderer::m_ViewDirBeam(const CWayPoint&wp_vis,
                                        const Vector&angle_dir)
{
    Vector source=wp_vis.Coord()+Vector(0,0,Eye_body_dist);
    Vector delta=60.0f*AnglesToVec(angle_dir);
    return new beam_t(source,source+delta,Vector(0,0,250),30.0f);
}

// dynamically draw the path to the goal from
// location of the  player with restriction
// node_filter, edge_filter

void CPathTracker::Rendering()
{
    const CWayPoint&from_n=glMapGraph.NearSuitableWP(m_player.Center(),m_player);

    std::vector<iterator> edges_redraw;
    bool rest=glMapGraph.RestrictedFindPath
    (from_n,edges_redraw,m_goal,m_node_filter,m_edge_filter);

    if(!rest)
    {
        ServerPrint("PathTracker: path not find \n");
        return;
    }
    for(auto iter:edges_redraw)
    {
        DrawEdge(iter);
        DrawNode(iter.from_node());
    }
    if(!edges_redraw.empty())
        DrawNode(edges_redraw.back().to_node(),
                 CMapGraphEditor::id_second_marked_st);

    // mark the start node:
    DrawNode(from_n,CMapGraphEditor::id_first_marked_st);
}

///CPathTrackerToWP
CPathTrackerToWP::CPathTrackerToWP(const player_t&player,
                                   const CSearchInTransposed&searcher):
CGraphRenderer(player),
m_path_gen(searcher)
{
    m_path_gen.UpdateDestination();
}


void CPathTrackerToWP::Rendering()
{
    const CWayPoint&from_n=glMapGraph.NearSuitableWP(m_player.Center(),m_player);

    if(!m_path_gen.SetBeginning(from_n))
    {
        ServerPrint("CPathTrackerToWP: path not find.\n");
        return;
    }
    edge_path_t cur_edge;
    for(;!m_path_gen.empty();m_path_gen.next())
    {
        cur_edge=m_path_gen.get();
        auto iter=cur_edge.from_node->get_iterator_to(*cur_edge.to_node);
        assert(iter);
        DrawEdge(*iter);
        DrawNode(*cur_edge.from_node);
    }
    if(cur_edge.to_node!=nullptr)
        DrawNode(*cur_edge.to_node,
                 CMapGraphEditor::id_second_marked_st);

    // mark the start node:
    DrawNode(from_n,CMapGraphEditor::id_first_marked_st);
}
