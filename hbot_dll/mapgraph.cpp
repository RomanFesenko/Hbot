#include "mapgraph.h"
#include <algorithm>
#include "../../BaseLibraries/ALGO_GRAPH/breadth_search.h"
#include "../../BaseLibraries/ALGO_GRAPH/edge_visitors.h"
#include"../../BaseLibraries/ALGO_GRAPH/transpose_graph.h"
#include "../../BaseLibraries/GEOMETRY/primitives_3D.h"

extern player_t* glMe;

//CMapGraph::SetEdge установление ребра между from_n и to_n
// с задаными способом перемещения (IN_JUMP и/или IN_DUCK)

void CMapGraph::SetEdge(CWayPoint&from_n,CWayPoint&to_n,int t_12,int t_21)
{
    assert(t_12>=0);
    float dist=Distance(from_n,to_n);
    CWayLink* wl=new CWayLink(t_12,dist);
    if(t_12==t_21)
    {
        Composer.set_undirect_edge(from_n,to_n,wl);
    }
    else if(t_21<0)
    {
        Composer.set_direct_edge(from_n,to_n,wl);
    }
    else
    {
        CWayLink* wl2=new CWayLink(t_21,dist);
        Composer.set_bidirect_edge(from_n,to_n,wl,wl2);
    }
}

// возвращает CItemPoint по флагу айтема

const CItemPoint* CWayPoint::ItemPoint(uint_t flag)const
{
    for(const auto& point:m_item_points)
    {
        if(flag&point->Type()) return point.get();
    }
    return nullptr;
}

//CMapGraph::UpgradeAdjacency - насыщает граф допустимыми ребрами
//Аргумент- предикат разрешает(true)/запрещает(false)
//связь между заданой парой нодов

void CMapGraph::UpgradeAdjacency(std::function<bool(const CWayPoint&,
                                           const CWayPoint&)> func)
{
    assert(NumWP()>1);

    for(auto from_it=WayPoints().begin();
        from_it!=WayPoints().end();++from_it)
    {

        for(auto to_it=from_it+1;
            to_it!=WayPoints().end();++to_it)
            {

                if(edge_type(**from_it,**to_it)!=id_no_edge) continue;

                if(!func(**from_it,**to_it)) continue;

                CWayLink* wl=new CWayLink(0,Distance(**from_it,**to_it));
                int type_prev=Composer.set_undirect_edge(**from_it,**to_it,wl);
                //ReportPrint("Set edge \n");
                assert(type_prev==id_no_edge);
            }
    }
}

//CMapGraph::NearVisibleWP поиск ближайшего к near видимого для player_t нода

CWayPoint* CMapGraph::NearVisibleWP(const Vector&near,const player_t& player,
                                   fn_b_wpref_t node_filter)
{
    if(node_filter==nullptr)
        {node_filter=[](const CWayPoint&wpt){return true;};}
    auto aux_transform=[&player,&near,node_filter](const CWayPoint* wp1)
    {
        if(!Visible(near,*wp1,player)||
           !node_filter(*wp1)) return 9999.0f;

        return Distance(*wp1,near);
    };
    CWayPoint* candidate=*(ext::min_element(WayPoints().begin(),
                                           WayPoints().end(),
                                           aux_transform));
    return (Visible(near,*candidate,player)&&
            node_filter(*candidate))?
            candidate:nullptr;
}

// Поиск не более чем num_WP ближайших видимых к near нодов

void CMapGraph::FewNearVisibleWP(const Vector&near,
                                const player_t&player,
                                int num_WP,
                                std::vector<const CWayPoint*>&res)

{
    if(num_WP>NumWP()) num_WP=NumWP();
    res.resize(num_WP);

    auto aux_transform=[&player,&near](const CWayPoint* wp1,
                                       const CWayPoint* wp2)
    {
        bool vis_1=Visible(near,*wp1,player);
        bool vis_2=Visible(near,*wp2,player);

        if(vis_1!=vis_2) return vis_1;

        return Distance(*wp1,near)<Distance(*wp2,near);
    };

    std::partial_sort_copy(WayPoints().begin(),WayPoints().end(),
                           res.begin(),res.begin()+num_WP,
                           aux_transform);

    auto vis_node=[&player,&near](const CWayPoint* wp1)
    {
        return !Visible(near,*wp1,player);
    };

    auto first_unvis=std::find_if(res.begin(),res.end(),vis_node);
    res.erase(first_unvis,res.end());
}

//CMapGraph::NearSuitableWP - возвращает ближайший видимый нод
// если такой есть иначе просто ближайший

CWayPoint& CMapGraph::NearSuitableWP(const Vector&near,const player_t&player,
                                    fn_b_wpref_t node_filter)
{
    CWayPoint*nwp=NearVisibleWP(near,player,node_filter);
    if constexpr(BOT_DEBUG)
    {
        if(nwp==nullptr&&node_filter==nullptr)
        {
            WarningPrint("Invisible position: "+ToString(near)+'\n');
        }
    }
    return (nwp!=nullptr)? *nwp :NearWP(near);
}

//CMapGraph::NearestEdge - возвращает ближайшее к pos ребро графа
std::optional<CMapGraph::iterator> CMapGraph::NearestEdge(const Vector&pos,float prec)const
{
    struct find_nearest_edge_t:public unique_edge_visitor
                                 <has_direct_edge_process|
                                has_bidirect_edge_process|
                                has_undirect_edge_process>
    {
        std::optional<iterator> near_edge={};
        float min_dist=9999.0f;
        Vector point;
        float precision;
        find_nearest_edge_t(const Vector& _point,float prec):
        point(_point),precision(prec)
        {}
        void edge_process(const iterator& wp1)
        {
            Vector dist=distance_3D
            (line_3D_t(wp1.from_node().Coord(),wp1.to_node().Coord()),
            point);
            if(dist[0]>precision||dist[1]<0||dist[1]>wp1.edge().weight()) return;
            if(dist[0]<min_dist)
            {
                min_dist=dist[0];
                near_edge=wp1;
            }
        }
        void direct_edge_process(const iterator& wp1)
        {
            edge_process(wp1);
        }
        void undirect_edge_process(const iterator& wp1)
        {
            edge_process(wp1);
        }
        void bidirect_edge_process(const iterator& wp1,const iterator&wp2)
        {
            edge_process(wp1);
        }
    };
    find_nearest_edge_t find_nearest_edge(pos,prec);
    unique_edge_visit(WayPoints(),find_nearest_edge);
    return find_nearest_edge.near_edge;
}

///edge_path_t

edge_path_t::edge_path_t(const CWayPoint*from_,const CWayPoint*to_,const CWayLink*wl_):
from_node(from_),to_node(to_),edge(wl_)
{

}

edge_path_t::edge_path_t(const CWayPoint::iterator&iter)
{
    from_node=&iter.from_node();
    to_node=&iter.to_node();
    edge=&iter.edge();
}

// Визиторы для алгоритма Дейкстры, прекращающие
// поиск при нахождении нода удовлетворяющему предикату

// поиск айтемов по набору флагов
std::function<bool(const CWayPoint&)> ItemFinder(int flag)
{
    return [flag](const CWayPoint& wp)
    {
        return (wp.ItemsFlag()&flag)!=0;
    };
}

// поиск заданого нода
std::function<bool(const CWayPoint&)>  NodeFinder(const CWayPoint&wpt)
{
    return [&wpt](const CWayPoint& discovered)
    {
        return &wpt==&discovered;
    };
}

// поиск нода невидимого из view_vec
std::function<bool(const CWayPoint&)>  UnvisibleNodeFinder(const Vector& view_vec,
                                                           const player_t&player)
{
    return [view_vec,&player](const CWayPoint& discovered)
    {
        return !Visible(view_vec,discovered,player);
    };
}

// Визитор ребер в алгоритме Дейкстры прекращающий
// поиск если превышен лимит длины пути

std::function<bool(const typename CWayPoint::iterator&iter,float)> DistanceLimiter(float max_dist)
{
    return [max_dist](const typename CWayPoint::iterator&iter,float dist)
    {
        return dist<max_dist;
    };
}

// RestrictedFindPath - 4-кратное инстанцирование
// шаблона m_dijkstra->restricted_find_path
// для всех вариантов наличия/отсутсвия фильтров вершин/ребер графа

// Альтернатива - вызов 2-х виртуальных функций {return true;}
// в ран-тайме на каждой итерации алгоритма.

bool CMapGraph::RestrictedFindPath(const CWayPoint&wp,std::vector<iterator>& iters,
                            std::function<bool(const CWayPoint&)> fn_goal,
                            std::function<bool(const CWayPoint&)> fn_node_filter,
                            std::function<bool(const typename CWayPoint::iterator&iter,float)>
                            fn_edge_filter)
{
    assert(m_dijkstra!=nullptr);
    return m_dijkstra->find_path_with_def_restrictors
    (wp,iters,fn_goal,fn_node_filter,fn_edge_filter);
}

//проверка связности - запуском поиска в ширину  из каждого нода;
//не лучший вариант,но для небольших графов норм
bool CMapGraph::CheckConnectivity()const
{
    struct counter:public search_restrictor<has_node_discover>
    {
        int m_count=0;
        counter(){}
        bool node_discover(const CWayPoint& new_disc)
        {
            m_count++;
            return true;
        }
    };

    breadth_search<CWayPoint> in_breadth(glMapGraph.WayPoints());
    counter temp_counter;

    for(CWayPoint*from_wp:glMapGraph.WayPoints())
    {
        in_breadth.done(*from_wp,temp_counter);
        assert(temp_counter.m_count<=glMapGraph.NumWP());
        if(temp_counter.m_count!=glMapGraph.NumWP()) return false;
        temp_counter.m_count=0;
    }
    return true;
}

void CMapGraph::RemoveDijkstra()
{
    if(m_dijkstra!=nullptr)
    {
        delete m_dijkstra;
        m_dijkstra=nullptr;
        m_tr_node=nullptr;
        m_visible_data=nullptr;
        m_transpose_composer.delete_graph();
    }
}

void CMapGraph::InitDijkstra()
{
    RemoveDijkstra();
    m_tr_node.reset(new TProperty<node_transposed_graph_t*,CWayPoint>(Composer.nodes().size()));
    m_dijkstra=new dijkstra_process<CWayPoint>(Composer.nodes());

    auto make_transposed_node=[this](const CWayPoint&source_node)
    {
        node_transposed_graph_t* for_ret=new node_transposed_graph_t(source_node);
        (*m_tr_node)(source_node)=for_ret;
        return for_ret;
    };
    auto make_transposed_edge=[](const CWayLink&source_edge)
    {
        return new edge_transposed_graph_t{&source_edge};
    };
    make_transpose(WayPoints(),make_transposed_node,
                               make_transposed_edge,
                               m_transpose_composer);
    m_visible_data.reset(new CVisibleData(WayPoints(),*glMe));
}

void CMapGraph::Destroy()
{
    RemoveDijkstra();
    Composer.delete_graph();
    m_visible_data=nullptr;
    m_transpose_composer.delete_graph();
    m_tr_node=nullptr;
}

const CVisibleData& CMapGraph::VisibleData()const
{
    assert(m_visible_data!=nullptr);
    return *(m_visible_data.get());
}


/// CFixedPath
graph_path_t&CFixedPath::Data()
{
    return m_path;
}

edge_path_t CFixedPath::get()const
{
    return edge_path_t(*m_current);
}

bool CFixedPath::empty()const
{
    return m_current==m_path.end();
}

void CFixedPath::next()
{
    ++m_current;
}

void CFixedPath::begin()
{
    m_current=m_path.begin();
}


///CSearchInTransposed
// поиск кратчайшего пути в обращенном графе
// позволяет находить все кратчайшие пути В заданый нод

CSearchInTransposed::CSearchInTransposed():
m_dijkstra(glMapGraph.TransposedGraph())
{
    assert(glMapGraph.IsInit());
}

const CWayPoint*CSearchInTransposed::Destination()const
{
    return m_destination;
}

void CSearchInTransposed::RestrictedFindPathes(
                          const CWayPoint&destination,
                          fn_b_wpref_t node_filter,
                          edge_filter_t edge_filter)
{
    using fn_b_trref_t=std::function<bool(const tr_node_t&)>;
    using tr_edge_filter_t=
    std::function<bool(const tr_node_t::iterator&iter,float)>;

    auto tr_search_finalizer=[](const tr_node_t&node)
    {
        return false;
    };
    auto tr_node_filter=[](const tr_node_t&node)
    {
        return true;
        //return node_filter(*node.m_source);
    };
    auto tr_edge_filter=[edge_filter](const tr_node_t::iterator&iter,float dist)
    {
        return true;
        //return edge_filter(*iter.edge().m_source,dist);
    };
    fn_b_trref_t adapt_node_filter=tr_node_filter;
    if(node_filter==nullptr)
    {
        adapt_node_filter=nullptr;
    }

    tr_edge_filter_t adapt_edge_filter=tr_edge_filter;
    if(edge_filter==nullptr)
    {
        adapt_edge_filter=nullptr;
    }

    std::vector<tr_node_t::iterator> tr_path;
    bool res=m_dijkstra.find_path_with_def_restrictors(
             *glMapGraph.TransposedNode(destination),tr_path,tr_search_finalizer,
             tr_node_filter,tr_edge_filter);
    assert(!res);
    m_destination=&destination;
}

bool CSearchInTransposed::IsPathFrom(const CWayPoint&start)const
{
    assert(m_destination!=nullptr);
    return m_dijkstra.is_find(*glMapGraph.TransposedNode(start));
}

std::optional<float> CSearchInTransposed::DistancePathFrom(const CWayPoint&start)const
{
    assert(m_destination!=nullptr);
    if(IsPathFrom(start))
    {
        return m_dijkstra.weight_result(*glMapGraph.TransposedNode(start));
    }
    else
    {
        return {};
    }
}

///CSearchInTransposed::path_generator_t

CSearchInTransposed::
path_generator_t::path_generator_t(const CSearchInTransposed& tr_search):
m_prev_iterator(tr_search.m_dijkstra.m_prev_iterator),
m_tr_search(tr_search)
{

}

void CSearchInTransposed::path_generator_t::UpdateDestination()
{
    m_destination=m_tr_search.m_destination;
    assert(m_destination!=nullptr);
}

bool CSearchInTransposed::path_generator_t::empty()const
{
    return m_current->m_source==m_destination;
}

edge_path_t CSearchInTransposed::path_generator_t::get()const
{
    auto tr_iter=m_prev_iterator(*m_current);
    return {m_current->m_source,
            tr_iter.from_node().m_source,// именно from_node
            tr_iter.edge().m_source};
}

void CSearchInTransposed::path_generator_t::next()
{
    m_current=&m_prev_iterator(*m_current).from_node();
}

bool CSearchInTransposed::path_generator_t::SetBeginning(const CWayPoint&beg)
{
    if(!m_tr_search.IsPathFrom(beg)) return false;
    m_current=glMapGraph.TransposedNode(beg);
    return true;
}

void CSearchInTransposed::path_generator_t::begin()
{
    assert(m_destination!=nullptr);
    assert(m_current!=nullptr);
}

///linker_func_t-s
// получение предикатов фильтрующих допустимые ребра
// для использования в  UpgradeAdjacency

// по длине ребра
linker_func_t fn_dist_filter(float max_dist)
{
    return [max_dist](const CWayPoint& wp1,const CWayPoint& wp2)
    {
        return Distance(wp1,wp2)<max_dist;
    };
}

// по наличию видимости между смежными нодами
linker_func_t fn_vis_filter(const player_t& player)
{
    return [&player](const CWayPoint& wp1,const CWayPoint& wp2)
    {
        return Visible(wp1.Coord(),wp2,player);
    };
}

// ограничение степеней вершин
linker_func_t fn_max_adj_filter(CWayPoint::adj_size_type max_adj)
{
    return [max_adj](const CWayPoint& wp1,const CWayPoint& wp2)
    {
        return wp1.size_adjacents()<=max_adj&&
               wp2.size_adjacents()<=max_adj;
    };
}

// допускает связь лишь если отсутсвуют
// слишком острые углы между исходящими ребрами
linker_func_t fn_angle_filter(float min_angle)
{
    auto get_max_cos=[](const Vector&new_dir,const CWayPoint&wpt)
    {
        float _res=-1.0f;
        typename CWayPoint::iterator iter;
        for(iter.set_base(wpt);!iter.end();iter.next())
        {
            Vector delta=iter.to_node().Coord()-wpt.Coord();
            _res=std::max(_res,cos_angle_3D(delta,new_dir));
        }
        return _res;
    };

    float cos_min_ang=cos(min_angle);
    return [cos_min_ang,get_max_cos](const CWayPoint& wp1,const CWayPoint& wp2)
    {
        Vector delta=wp2.Coord()-wp1.Coord();

        return get_max_cos(delta,wp1)<cos_min_ang&&
               get_max_cos(-delta,wp2)<cos_min_ang;
    };
}

///CVisibleData
/* структура для определения лучшего обзора при
 движении вдоль ребра A->B графа.
 Алгоритм:
 1) Для каждого нода N определяется множество V(N) нодов видимых из N;
 2) Для каждого ребра A->B определяется разность множеств V(B)-V(A);
 3) Наиболее выгодное направление взгляда из B определяется из условия
 максимизации числа нодов, принадлежащих V(B)-V(A) и попадающих
 в конус обзора с вершиной в B.
*/

CVisibleData::CVisibleData(const std::vector<CWayPoint*>&graph_vec,
                           const player_t& player):
m_vis_nodes(graph_vec.size())
{
    ReportPrint("Stuff CVisibleData\n");
    m_vis_nodes_vec.clear();
    m_vis_nodes_vec.reserve(graph_vec.size());
    for(const CWayPoint* wpt_from:graph_vec)
    {
        m_vis_nodes_vec.push_back(vis_nodes_t());
        vis_nodes_t&current=m_vis_nodes_vec.back();
        for(const CWayPoint* wpt_to:graph_vec)
        {
            if(Visible(wpt_from->Coord()+Vector(0,0,Eye_body_dist),
                       *wpt_to,player))
            {
                current.push_back(wpt_to);// находим  V(A)
            }
        }
        if constexpr(BOT_DEBUG)
        {
            if(current.empty())
            {
                WarningPrint("Void visible nodes list:\n"+
                             ToString(wpt_from->Coord())+'\n');
            }
        }
        // сортируем V(A) чтобы была возможность применить
        // линейный алгоритм нахождения разности множеств
        std::sort(current.begin(),current.end());
        m_vis_nodes(*wpt_from)=m_vis_nodes_vec.end()-1;
    }
    m_diff_vis_nodes.clear();
    auto set_difference=[this](const CWayPoint&from_n,
                           const CWayPoint&to_n)
    {
        auto vis_from=m_vis_nodes(from_n);
        auto vis_to=m_vis_nodes(to_n);
        diff_vis_nodes_t diff_vis;
        diff_vis.from_node=&from_n;
        diff_vis.to_node=&to_n;

        assert(is_sorted(vis_to->begin(),vis_to->end()));
        assert(is_sorted(vis_from->begin(),vis_from->end()));

        // находим V(B)-V(A)
        std::set_difference(vis_to->begin(),vis_to->end(),
                            vis_from->begin(),vis_from->end(),
                            std::back_inserter(diff_vis.difference));
        if(diff_vis.difference.empty())
        {
            m_diff_vis_nodes.push_back(diff_vis);
            return;
        }

        // число нодов попадающих в конус обзора
        auto count_in_view_con=[](const Vector&vis_pos,
                                  const CWayPoint&vis_wpt,
                                  const vis_nodes_t& vis)
        {
            Vector view_dir=vis_wpt.Coord()-vis_pos;
            return std::count_if(vis.begin(),vis.end(),
                   [&vis_pos,&view_dir](const CWayPoint*wpt)
                   {
                        return dot_product_3D(wpt->Coord()-vis_pos,view_dir)>View_cos;
                   });
        };

        Vector view_from=to_n.Coord()+Vector(0,0,Eye_body_dist);
        auto best_for_view=ext::max_element(
                        diff_vis.difference.begin(),diff_vis.difference.end(),
                        [count_in_view_con,view_from,&diff_vis](const CWayPoint*cur_view)
                        {
                            return count_in_view_con(view_from,*cur_view,diff_vis.difference);
                        });

       diff_vis.m_priority_vis_direct=
       VecToAngles((*best_for_view)->Coord()-view_from);

       m_diff_vis_nodes.push_back(diff_vis);
    };


    struct edge_vis_setter_t:public unique_edge_visitor
                                                <has_undirect_edge_process|
                                                has_direct_edge_process|
                                                has_bidirect_edge_process>
    {
        using edge_process_t=decltype(set_difference);
        edge_process_t m_set_difference;
        edge_vis_setter_t(edge_process_t _sdiff):
        m_set_difference(_sdiff){}
        void undirect_edge_process(const gr_iterator& iter)
        {
            m_set_difference(iter.from_node(),iter.to_node());
            m_set_difference(iter.to_node(),iter.from_node());
        }
        void bidirect_edge_process(const gr_iterator& iter1_2,
                                   const gr_iterator& iter2_1)
        {
            m_set_difference(iter1_2.from_node(),iter1_2.to_node());
            m_set_difference(iter1_2.to_node(),iter1_2.from_node());
        }
        void direct_edge_process(const gr_iterator& iter)
        {
            m_set_difference(iter.from_node(),iter.to_node());
        }
    };
    edge_vis_setter_t edge_vis_setter(set_difference);
    unique_edge_visit(graph_vec,edge_vis_setter);

    std::sort(m_diff_vis_nodes.begin(),m_diff_vis_nodes.end(),
              m_CompareDiff);
    assert(adjacent_find(m_diff_vis_nodes.begin(),m_diff_vis_nodes.end(),
                         m_EqualDiff)==m_diff_vis_nodes.end());
}

///CVisibleData::GetViewing
// по заданому ребру A->B пути возвращаем выгоднейший угол обзора
// или {}, если  V(B)-V(A)==0

std::optional<Vector> CVisibleData::GetViewing(const edge_path_t&edge)const
{
    diff_vis_nodes_t for_find;
    for_find.from_node=edge.from_node;
    for_find.to_node=edge.to_node;
    auto iter=std::lower_bound(m_diff_vis_nodes.begin(),m_diff_vis_nodes.end(),
                               for_find,m_CompareDiff);
    assert(iter!=m_diff_vis_nodes.end());
    assert(m_EqualDiff(*iter,for_find));

    if(!iter->difference.empty())
    {
        return iter->m_priority_vis_direct;
    }
    else
    {
        return {};
    }
}

///SaveMapGraph
// Последовательность записи данных вэйпоинта в файл:
// 1) Вектор - позиция (для всех нодов);
// 2) Число  ребер;
// 3) Все ребра;
// 4) Данные о айтемах

void SaveMapGraph(const CMapGraph& graph,const std::string&name)
{

   class CGraphWriter:public unique_edge_visitor<has_undirect_edge_process|
                                            has_direct_edge_process|
                                            has_bidirect_edge_process>
    {
    using iterator=CWayPoint::iterator;
    FILE* m_fl;
    SLinkInfo m_link_info;
    public:
    explicit CGraphWriter(FILE* _fl):m_fl(_fl)
    {
    }
    void direct_edge_process(const iterator& wp_1)
    {
        m_link_info.from=wp_1.from_node().index();
        m_link_info.to=wp_1.to_node().index();
        m_link_info.moving_1st_2nd=wp_1.edge().type_moving();
        m_link_info.moving_2nd_1st=-1;
        fwrite(&m_link_info,sizeof(SLinkInfo),1,m_fl);
    }
    void undirect_edge_process(const iterator& wp_1)
    {
        m_link_info.from=wp_1.from_node().index();
        m_link_info.to=wp_1.to_node().index();
        m_link_info.moving_1st_2nd=wp_1.edge().type_moving();
        m_link_info.moving_2nd_1st=wp_1.edge().type_moving();
        assert(m_link_info.moving_1st_2nd==m_link_info.moving_2nd_1st);
        fwrite(&m_link_info,sizeof(SLinkInfo),1,m_fl);
    }
    void bidirect_edge_process
    (const iterator& wp_1,const iterator& wp_2)
    {
        m_link_info.from=wp_1.from_node().index();
        m_link_info.to=wp_2.to_node().index();
        m_link_info.moving_1st_2nd=wp_1.edge().type_moving();
        m_link_info.moving_2nd_1st=wp_2.edge().type_moving();
        assert(m_link_info.moving_1st_2nd!=m_link_info.moving_2nd_1st);
        fwrite(&m_link_info,sizeof(SLinkInfo),1,m_fl);
    }
   };

   int nn=graph.NumWP();
   std::string name_ext=BotDirectory()+name+".dat";
   FILE* fl_inst=fopen(name_ext.c_str(),"wb");
   assert(fl_inst!=nullptr);
   CGraphWriter _Writer(fl_inst);
   fwrite(&nn,sizeof(int),1,fl_inst);

   // Пишем все векторы - позиции:
   Vector temp;
   for(const auto& curwp:graph.WayPoints())
   {
      temp=curwp->Coord();
      fwrite(&temp,sizeof(Vector),1,fl_inst);
   }
   // Пишем число связей:
   int num_links=graph.Composer.all_edges();

   fwrite(&num_links,sizeof(int),1,fl_inst);
   // Пишем структуры SLinkInfo так что from < to
   unique_edge_visit(graph.WayPoints(),_Writer);
   // запись данных о айтемах
   int num_items=glMapGraph.ItemPoints().size();
   fwrite(&num_items,sizeof(int),1,fl_inst);
   for(auto item:glMapGraph.ItemPoints())
   {
       fwrite(&item->Data(),sizeof(CItemPoint::item_data_t),1,fl_inst);
   }

   fclose(fl_inst);
}

///LoadMapGraph
//1) Число нодов N
//2) N векторов соотв координатам нодов
//3) Число связей E
//4) E связей типа [int from,int to,int moving from->to,int moving to->from]
//5) Данные о айтемах если таковые есть иначе автоматическое
//   их формирование

bool LoadMapGraph(CMapGraph& graph,const std::string&name)
{
   graph.Destroy();
   FILE*fl=fopen((BotDirectory()+name+".dat").c_str(),"rb");
   if(fl==0) return false;
   int nn,nl,num_point;
   fread(&nn,sizeof(nn),1,fl);
   // Считываем позиции нодов и грузим их в граф
   for(int i=0; i<nn;i++)
   {
      Vector temp_vec;
      fread(&temp_vec,sizeof(Vector),1,fl);
      graph.Composer.add_node(new CWayPoint(temp_vec));
   }
   // Устанавливаем связи
   SLinkInfo curlinkinfo;
   fread(&nl,sizeof(nl),1,fl);
   const std::vector<CWayPoint*>& wps=graph.WayPoints();
   for(int i=0;i<nl;i++)
   {
      int result =fread(&curlinkinfo,sizeof(curlinkinfo),1,fl);
      assert(result!=0);

      const auto& [n_1,n_2,t_12,t_21]=curlinkinfo;
      graph.SetEdge(*wps[n_1],*wps[n_2],t_12,t_21);
   }

   // загрузка данных о айтемах
   int num_read=fread(&num_point,sizeof(num_point),1,fl);
   if(num_read==1)
   {
       if constexpr(BOT_DEBUG)
       {
           ReportPrint("-----------Read items data----------------\n");
       }
       CItemPoint::item_data_t item_data;
       for(int i=0;i<num_point;i++)
       {
           fread(&item_data,sizeof(item_data),1,fl);
           CItemPoint* new_item=new CItemPoint(item_data);
           assert(bit_count(new_item->Type())==1);
           if(bit_count(new_item->Type())==1)
           {
               AddItemPoint(new_item);
           }
           if constexpr(BOT_DEBUG)
           {
               ReportPrint("Item: "+
                    get_string(new_item->Type())+'('+
                    std::to_string(new_item->Type())+
                    ")\nVector: "+ToString(new_item->Coord())+'\n');
           }
       }
       if constexpr(BOT_DEBUG)
       {
           ReportPrint("-----------Read items data end------------\n");
       }
   }
   else
   {
       ReportPrint("Default define items data \n");
       LinkItemPoints();
   }
   fclose(fl);

   int num_create_link=graph.Composer.all_edges();
   assert(nl==num_create_link);
   graph.InitDijkstra();
   return true;
}

///LinkItemPoints
// проверяем все Entity на предмет интересности
// и ассоциируем их с CWayPointами

void LinkItemPoints()
{
    for(const auto& wpt:glMapGraph.WayPoints())
    {
        wpt->m_items_flag=0;
        wpt->m_item_points.clear();
    }

    glMapGraph.m_item_points.clear();
    glMapGraph.m_items_flag=0;

    std::vector<std::pair<int,Vector>> items;
    AllItemsInMap(items);
    for(const auto& [flag,pos]:items)
    {
        AddItemPoint(new CItemPoint(pos,flag));
    }
}

void AddItemPoint(CItemPoint* item)
{
    const CWayPoint& near=glMapGraph.NearWP(item->Coord());
    if(Distance(near,item->Coord())>=200.0f||
       !Visible(item->Coord(),near,*glMe)) return;

    glMapGraph.m_item_points.push_back(item_ptr_t(item));
    glMapGraph.m_items_flag|=item->Type();
    near.m_item_points.push_back(glMapGraph.m_item_points.back());
    near.m_items_flag|=item->Type();
    item->m_owner=&near;
    assert(near.ItemPoint(item->Type())!=nullptr);
}


void RemoveItemPoint(item_ptr_t ipoint)
{
    const CWayPoint&near=ipoint->NodeOwner();

    auto iter=std::find(near.m_item_points.begin(),
                        near.m_item_points.end(),ipoint);

    // удаление из CWayPoint
    assert(iter!=near.m_item_points.end());
    *iter=near.m_item_points.back();
    near.m_item_points.pop_back();
    near.m_items_flag=0;
    for(const auto&rest_item:near.m_item_points)
    {
        near.m_items_flag|=rest_item->Type();
    }

    // удаление из CMapGraph
    iter=std::find(glMapGraph.m_item_points.begin(),
                   glMapGraph.m_item_points.end(),ipoint);

    assert(iter!=glMapGraph.m_item_points.end());
    *iter=glMapGraph.m_item_points.back();
    glMapGraph.m_item_points.pop_back();
    for(const auto&rest_item:glMapGraph.m_item_points)
    {
        glMapGraph.m_items_flag|=rest_item->Type();
    }
}

///TestGraph
// тестирование производительности glMapGraph.RestrictedFindPath
void TestGraph(const player_t& player,int cycles)
{
    const CWayPoint& near_wpt=glMapGraph.
                              NearSuitableWP(player.Center(),player);

    graph_path_t path;
    int num_pr_nodes=0;
    auto node_process=[&num_pr_nodes](const CWayPoint&wp)
    {
        num_pr_nodes++;
        return false;
    };
    for(int i=0;i<cycles;i++)
    {
        glMapGraph.RestrictedFindPath(near_wpt,path,node_process);
        path.clear();
    }
    ServerPrint("Number processed nodes: "+std::to_string(num_pr_nodes)+'\n');
}
