#ifndef _graph_editor_
#define _graph_editor_

#include <map>
#include <tuple>
#include <optional>

#include "mapgraph.h"
#include "util.h"
#include "../../BaseLibraries/ALGO_GRAPH/edge_visitors.h"


const int id_default_style=-1;
const int id_native_style=-2;

class CMapGraphViewer
{
   public:

   using wl_iterator=typename CWayPoint::iterator;
   using edge_t=typename CWayPoint::edge_t;
   using style_t=std::tuple<Vector,int,float>;// цвет,толщина,высота
   private:
   const style_t default_style={Vector(255,0,0),30,60};
   std::map<int,style_t> m_styles;
   int m_def_node_style=id_default_style;
   int m_def_diredge_style=id_default_style;
   int m_def_undiredge_style=id_default_style;
   int m_def_multiedge_style=id_default_style;
   int m_def_item_style=id_default_style;

   struct style_beam_t:public beam_t
   {
       int b_style;
       style_beam_t(const Vector& from,const Vector& to,
                    const style_t& style,int id_style):
       beam_t(from,to,std::get<0>(style),std::get<1>(style)),
       b_style(id_style){}
       //style_beam_t(style_beam_t&&)=default;
   } ;
   style_beam_t make_node_beam(const CWayPoint& wp,const style_t& st,int st_id)
   {
       return style_beam_t(wp.Coord(),
                           wp.Coord()+Vector(0,0,std::get<2>(st)),
                           st,st_id);
   }
   style_beam_t make_edge_beam(const wl_iterator& wl,const style_t& st,int st_id)
   {
       return style_beam_t(wl.from_node().Coord(),
                           wl.to_node().Coord(),
                           st,st_id);
   }
   style_beam_t make_item_beam(const CItemPoint&item,const style_t& st,int st_id)
   {
       return style_beam_t(item.Coord(),
                           item.Coord()+Vector(0,0,std::get<2>(st)),
                           st,st_id);
   }

   std::map<const CWayPoint*,style_beam_t> m_visible_nodes;

   using pair_nodes_t=std::pair<const CWayPoint*,const CWayPoint*>;
   pair_nodes_t to_pair(const wl_iterator& wlink)
   {
       return std::minmax(&wlink.from_node(),&wlink.to_node());
   }

   std::map<pair_nodes_t,style_beam_t> m_visible_edges;
   struct drawer:public unique_edge_visitor<has_direct_edge_process|
                                            has_undirect_edge_process|
                                            has_bidirect_edge_process>
   {
        CMapGraphViewer& viewer;
        drawer( CMapGraphViewer& _viewer):viewer(_viewer){}
        void direct_edge_process(const wl_iterator& wp1)
        {
            viewer.ShowEdge(wp1,viewer.m_def_diredge_style);
        }
        void undirect_edge_process(const wl_iterator& wp1)
        {
            viewer.ShowEdge(wp1,viewer.m_def_undiredge_style);
        }
        void bidirect_edge_process(const wl_iterator& wp1,const wl_iterator& wp2)
        {
            viewer.ShowEdge(wp1,viewer.m_def_multiedge_style);
        }
   };
   friend class drawer;

   std::map<const CItemPoint*,style_beam_t> m_visible_items;

   public:
   CMapGraphViewer()
   {
       m_styles.insert({id_default_style,default_style});
   }

   int NumVisNodes()const{return m_visible_nodes.size();}
   int NumVisEdges()const{return m_visible_edges.size();}
   int NumVisBeams()const{return m_visible_nodes.size()+m_visible_edges.size();}

   void ShowNode(const CWayPoint&,int style=id_native_style);
   void ShowEdge(const wl_iterator&,int style=id_native_style);
   void ShowItem(const CItemPoint&item,int style=id_native_style);

   void HideNode(const CWayPoint& wp);
   void HideEdge(const wl_iterator& wlink);
   void HideItem(const CItemPoint*item);

   void Show(const CMapGraph&);
   bool RegisterStyle(int iden,const style_t& style)
   {
       return m_styles.insert({iden,style}).second;
   }

   int SetNodeStyle(int _st);
   int SetDirectEdgeStyle(int _st);
   int SetUndirectEdgeStyle(int _st);
   int SetMultiEdgeStyle(int _st);
   int SetItemsStyle(int _st);
   void HideAll();

   ~CMapGraphViewer(){HideAll();}
};

inline CMapGraphViewer glMapGraphViewer;

class CGraphRenderer
{
    protected:
    using iterator=typename CWayPoint::iterator;
    const player_t& m_player;
    void DrawNode(const CWayPoint& node,int style=id_native_style);
    void DrawEdge(const iterator&iter,int style=id_native_style);
    void DrawItem(const CItemPoint&item,int style=id_native_style);

    public:
    explicit CGraphRenderer(const player_t& pl):
    m_player(pl)
    {
    }
    void operator()();
    virtual void Rendering()=0;
    virtual ~CGraphRenderer();
};

class CNearestRenderer:public CGraphRenderer
{
    const int m_max_beams;
    std::shared_ptr<beam_t> m_view_dir_1_2;
    std::shared_ptr<beam_t> m_view_dir_2_1;

    beam_t* m_ViewDirBeam(const CWayPoint&wp_vis,const Vector&angle_dir);

    public:
    explicit CNearestRenderer(const player_t& player,int max_beam):
    CGraphRenderer(player),m_max_beams(max_beam){}
    virtual void Rendering()override;

    virtual ~CNearestRenderer(){}
};

class CPathTracker:public CGraphRenderer
{
    fn_b_wpref_t m_goal;
    fn_b_wpref_t m_node_filter;
    edge_filter_t m_edge_filter;

    public:
    explicit CPathTracker(const player_t& player,
                          fn_b_wpref_t goal,
                          fn_b_wpref_t node_filter=nullptr,
                          edge_filter_t edge_filter=nullptr):
    CGraphRenderer(player),
    m_goal(goal),
    m_node_filter(node_filter),
    m_edge_filter(edge_filter){}

    virtual void Rendering()override;
    virtual ~CPathTracker(){}
};

class CPathTrackerToWP:public CGraphRenderer
{
    CSearchInTransposed::path_generator_t m_path_gen;
    public:
    CPathTrackerToWP(const player_t&player,
                     const CSearchInTransposed&);

    virtual void Rendering()override;
    virtual ~CPathTrackerToWP(){}
};

// CMapGraphEditor - обрабатывает пользовательский ввод на создание и
// модификацию элементов графа
class CMapGraphEditor
{
   public:
   enum id_styles
   {
       id_first_marked_st=0,
       id_second_marked_st,
       id_marked_edge_st,
       id_direct_edge_st,
       id_undirect_edge_st,
       id_bidirect_edge_st,
       id_node_st,
       id_item_st
    };
   using iterator=typename CWayPoint::iterator;
   private:
   CWayPoint* m_first_marked=nullptr;
   CWayPoint* m_second_marked=nullptr;
   std::optional<iterator> m_edge_marked={};
   bool m_is_run=false;
   bool m_is_graph_valid=false;
   public:
   bool IsRun()const{return m_is_run;}
   using style_t=typename CMapGraphViewer::style_t;

   CMapGraphEditor( );
   std::string Run(float,const player_t&player);
   std::string CreateWP(const Vector& pos);
   std::string DeleteMarkedWP();
   std::string DeleteMarkedEdge();
   std::string MarkWP(const Vector& pos);
   std::string MarkLink(const Vector& pos);
   std::string SetEdge(int type_12,int type_21);
   std::string BuildDefaultGraph(
                                const player_t&player,
                                float max_dist,
                                int max_adj,
                                float min_angle);

   std::string CreateItemPoint(const Vector&pos,
                               const Vector&angle,
                               int type);

   std::string LinkItemPoints();
   std::string DeleteItemPoint(const Vector&nearest);

   std::string SaveGraph();
   std::string  Close();
   virtual ~CMapGraphEditor(){}
   friend class CNearestRenderer;
};

inline CMapGraphEditor glMapGraphEditor;


#endif
