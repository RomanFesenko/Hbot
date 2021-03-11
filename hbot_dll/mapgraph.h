
#ifndef  _mapgraph_
#define  _mapgraph_
#include "items.h"
#include "util.h"
#include "../../BaseLibraries/ALGO_GRAPH/graph_composer.h"
#include "../../BaseLibraries/GEOMETRY/primitives.h"
#include "../../BaseLibraries/ALGO_EXTEND/algo_extend.h"

#include "../../BaseLibraries/ALGO_GRAPH/dijkstra.h"


inline const float Eye_body_dist=30.0f;
inline const float View_cos=cos(85.0f*Degree);

class CWayPoint;
class CItemPoint
{
   public:
   struct item_data_t
   {
       Vector coord;
       Vector view_angle;
       uint_t type;
   };
   private:
   item_data_t m_data;
   const CWayPoint* m_owner=nullptr;
   public:
   CItemPoint(const Vector&pos,uint_t flag,const Vector&ang=Vector(0,0,0)):
   m_data({pos,ang,flag})
   {}

   CItemPoint(const item_data_t&item):m_data(item){}

   const Vector&Coord()const{return m_data.coord;}
   const Vector&ViewAngle()const{return m_data.view_angle;}
   uint_t Type() const{return m_data.type;}
   const item_data_t&Data()const{return m_data;}
   const CWayPoint& NodeOwner()const
   {
       assert(m_owner!=nullptr);
       return *m_owner;
   }
   friend void AddItemPoint(CItemPoint* ip);
   friend void RemoveItemPoint(std::shared_ptr<CItemPoint>);
};

enum TypeMoving
{
   id_duck_move=1<<0,
   id_jump_move=1<<1,
   id_stair_move=1<<2
};

class CWayLink
{
    int m_type_moving;
    float  m_lenght;
    public:
    CWayLink(int _t_m,float lenght):
    m_type_moving(_t_m),m_lenght(lenght)
    {}
    float weight()const{return m_lenght;}
    int type_moving()const{return m_type_moving;}
    unsigned short us_type_moving()const
    {
        return static_cast<unsigned short>(m_type_moving);
    }
};

using item_ptr_t=std::shared_ptr<CItemPoint>;

void LinkItemPoints();
class CWayPoint:public CBaseNode<CWayLink,CWayPoint>
{
    mutable std::vector<item_ptr_t> m_item_points;
    mutable uint_t m_items_flag=0;
    Vector m_coord;
    public:
    explicit CWayPoint(const Vector& _vec):m_coord(_vec){}
    const Vector& Coord()const{return m_coord;}

    uint_t ItemsFlag()const{return m_items_flag;}
    const CItemPoint* ItemPoint(uint_t flag)const;

    friend void LinkItemPoints();
    friend void AddItemPoint(CItemPoint* ip);
    friend void RemoveItemPoint(item_ptr_t);
};

using fn_b_wpref_t=std::function<bool(const CWayPoint&)>;
using edge_filter_t=std::function<bool(const typename CWayPoint::iterator&iter,float)>;


fn_b_wpref_t  ItemFinder(int flag);
fn_b_wpref_t  NodeFinder(const CWayPoint&);
fn_b_wpref_t  UnvisibleNodeFinder(const Vector& view_vec,
                                  const player_t&player);

edge_filter_t  DistanceLimiter(float max_dist);



inline float Distance(const CWayPoint& nod,const Vector& b)
{
    return distance_3D(nod.Coord()-b);
}

inline float Distance(const CWayPoint& nod,const CWayPoint& nod2)
{
    return distance_3D(nod.Coord()-nod2.Coord());
}

inline bool Visible(const Vector&a, const CWayPoint&nod, const player_t&pl)
{
    return Visible(a,nod.Coord(),pl);
}

struct edge_path_t
{
    const CWayPoint* from_node=nullptr;
    const CWayPoint* to_node=nullptr;
    const CWayLink*  edge=nullptr;
    edge_path_t(){}
    edge_path_t(const CWayPoint*,const CWayPoint*,const CWayLink*);
    explicit edge_path_t(const CWayPoint::iterator&);
};

using graph_path_t=std::vector<typename CWayPoint::iterator>;

class CSearchInTransposed;
class CVisibleData;
class CMapGraph
{
    dijkstra_process<CWayPoint>* m_dijkstra=nullptr;
    std::vector<item_ptr_t> m_item_points;
    std::unique_ptr<CVisibleData> m_visible_data;
    mutable uint_t m_items_flag=0;
    public:
    struct edge_transposed_graph_t
    {
        const CWayLink* m_source;
        float weight()const{return m_source->weight();}
    };
    struct node_transposed_graph_t:public CBaseNode<edge_transposed_graph_t,node_transposed_graph_t>
    {
        const CWayPoint*m_source;
        explicit node_transposed_graph_t(const CWayPoint&sce):
        m_source(&sce){}
    };
    private:
    CGraphComposer<node_transposed_graph_t> m_transpose_composer;
    std::unique_ptr<TProperty<node_transposed_graph_t*,CWayPoint>> m_tr_node;
    public:
    using iterator=typename CWayPoint::iterator;
    CGraphComposer<CWayPoint> Composer;

    CMapGraph()
    {
    }
    bool IsInit()const{return m_dijkstra!=nullptr;}

    int NumWP()const{return Composer.nodes().size();}

    const std::vector<CWayPoint*>& WayPoints()const
    {
        return Composer.nodes();
    }
    const std::vector<node_transposed_graph_t*>& TransposedGraph()const
    {
        return m_transpose_composer.nodes();
    }
    node_transposed_graph_t* TransposedNode(const CWayPoint&source)const
    {
        return (*m_tr_node)(source);
    }
    void SetEdge(CWayPoint&from_n,CWayPoint&to_n,int t_12,int t_21);
    void DeleteAllEdges()
    {
        Composer.delete_all_edges();
        RemoveDijkstra();
    }

    void UpgradeAdjacency(std::function<bool(const CWayPoint&,
                                            const CWayPoint&)>);

    CWayPoint& NearWP(const Vector& vect)
    {
        auto dist=[&vect]( CWayPoint* wp)
        {
            return Distance(*wp,vect);
        };
        return **(ext::min_element(WayPoints().begin(),WayPoints().end(),dist));
    }

    CWayPoint* NearVisibleWP(const Vector&near,const player_t&,
                             fn_b_wpref_t node_filter=nullptr);

    CWayPoint& NearSuitableWP(const Vector&near,const player_t&,
                              fn_b_wpref_t node_filter=nullptr);

    std::optional<iterator> NearestEdge(const Vector&pos,float prec)const;

    void FewNearVisibleWP(const Vector&near,const player_t&,int,
                           std::vector<const CWayPoint*>&);

    bool GetPath(const CWayPoint& wp1,const CWayPoint& wp2,std::vector<iterator>& iters)
    {
        assert(m_dijkstra!=nullptr);
        return m_dijkstra->get_path(wp1,wp2,iters);
    }

    bool RestrictedFindPath(const CWayPoint&wp,std::vector<iterator>& iters,
                            fn_b_wpref_t goal,
                            fn_b_wpref_t node_filter=nullptr,
                            edge_filter_t edge_filter=nullptr);

    bool FindPath(const CWayPoint& wp1,const CWayPoint& wp2,std::vector<iterator>& iters)
    {
        assert(m_dijkstra!=nullptr);
        return m_dijkstra->find_path(wp1,wp2,iters)>=0.0f;
    }
    float DistanceTo(const CWayPoint& wp1)const
    {
        return m_dijkstra->weight_result(wp1);
    }
    // Called only after building the graph by calling LoadMapGraph
    void InitDijkstra();
    void RemoveDijkstra();

    bool CheckConnectivity()const;
    void Destroy();
    const std::vector<item_ptr_t>& ItemPoints()const{return m_item_points;}
    ~CMapGraph()
    {
        if(m_dijkstra!=nullptr)
        {
            delete m_dijkstra;
        }
    }
    uint_t ItemsFlag()const{return m_items_flag;}
    const CVisibleData&VisibleData()const;
    friend void LinkItemPoints();
    friend void AddItemPoint(CItemPoint* ip);
    friend void RemoveItemPoint(item_ptr_t);
};
inline CMapGraph glMapGraph;

class CPathGenerator
{
    public:
    virtual void begin()=0;
    virtual edge_path_t get()const=0;
    virtual bool empty()const=0;
    virtual void next()=0;
    virtual~CPathGenerator(){}
};

class CFixedPath:public CPathGenerator
{
    graph_path_t m_path;
    typename graph_path_t::iterator m_current;
    public:
    CFixedPath(){}
    graph_path_t&Data();
    virtual edge_path_t get()const override;
    virtual bool empty()const override;
    virtual void next() override;
    virtual void begin()override;
};

class CSearchInTransposed
{
    using tr_node_t=CMapGraph::node_transposed_graph_t;
    using tr_edge_t=CMapGraph::edge_transposed_graph_t;
    using tr_iterator=tr_node_t::iterator;
    dijkstra_process<tr_node_t> m_dijkstra;
    const CWayPoint*m_destination=nullptr;
    public:
    CSearchInTransposed();
    const CWayPoint*Destination()const;
    void RestrictedFindPathes(const CWayPoint&destination,
                             fn_b_wpref_t node_filter=nullptr,
                             edge_filter_t edge_filter=nullptr);
    bool IsPathFrom(const CWayPoint&start)const;
    std::optional<float> DistancePathFrom(const CWayPoint&start)const;

    class path_generator_t:public CPathGenerator
    {
        const tr_node_t* m_current=nullptr;
        const CWayPoint*m_destination=nullptr;
        const TProperty<tr_iterator,tr_node_t>& m_prev_iterator;
        const CSearchInTransposed& m_tr_search;
        public:
        path_generator_t(const CSearchInTransposed&);
        void UpdateDestination();
        virtual void begin()override;
        virtual edge_path_t get()const override;
        virtual bool empty()const override;
        virtual void next()override;
        bool SetBeginning(const CWayPoint&);
    };
};

class CVisibleData
{
    using vis_nodes_t=std::vector<const CWayPoint*>;
    using gr_iterator=CMapGraph::iterator;
    std::vector<vis_nodes_t> m_vis_nodes_vec;
    TProperty<std::vector<vis_nodes_t>::iterator,CWayPoint> m_vis_nodes;
    struct diff_vis_nodes_t
    {
        const CWayPoint* from_node;
        const CWayPoint* to_node;
        vis_nodes_t difference;
        Vector m_priority_vis_direct;
    };
    std::vector<diff_vis_nodes_t> m_diff_vis_nodes;

    static bool m_CompareDiff(const diff_vis_nodes_t& fst,
                              const diff_vis_nodes_t& snd)
    {
        return (fst.from_node!=snd.from_node)?
            (fst.from_node<snd.from_node):
            (fst.to_node<snd.to_node);
    }
    static bool m_EqualDiff(const diff_vis_nodes_t& fst,
                            const diff_vis_nodes_t& snd)
    {
        return (fst.from_node==snd.from_node)&&
                (fst.to_node==snd.to_node);
    }

    public:
    CVisibleData(const std::vector<CWayPoint*>&,const player_t&);
    std::optional<Vector> GetViewing(const edge_path_t&)const;
};


using linker_func_t=std::function<bool(const CWayPoint&,const CWayPoint&)>;

linker_func_t fn_dist_filter(float max_dist);
linker_func_t fn_vis_filter(const player_t& player);
linker_func_t fn_max_adj_filter(CWayPoint::adj_size_type max_adj);
linker_func_t fn_angle_filter(float min_angle);

struct SLinkInfo
{
    int from;
    int to;
    int moving_1st_2nd;
    int moving_2nd_1st;// - -1 if communication is one-way
};


void SaveMapGraph(const CMapGraph& graph,const std::string&);
bool LoadMapGraph(CMapGraph& graph,const std::string&);
void TestGraph(const player_t&,int );
#endif

