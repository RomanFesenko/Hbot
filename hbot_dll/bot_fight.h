#ifndef  _bot_fight
#define  _bot_fight

#include "bot_main.h"
#include "parse_config.h"

#include <deque>


class CEnemyData
{
   public:
   enum visible_t
   {
       id_vis_body=1<<0,
       id_vis_head=1<<1
   };
   struct palyer_data_t
   {
       Vector position;
       Vector view_angle;
   };
   private:
   const CBot& m_owner;//  бот которому принадлежат данные
   const player_t& m_enemy;
   int m_visible=0;
   palyer_data_t m_last_enemy_data;// где враг виден последний раз
   palyer_data_t m_last_owner_data;
   float m_last_time_vis=0; // когда враг виден последний раз

   public:
   CEnemyData(const CBot&owner,const player_t&enemy);

   void Run();
   const CBot&Owner()const{return m_owner;}
   void Update();
   const player_t&Enemy()const{return m_enemy;}
   int Visible()const{return m_visible;}
   bool IsVisible()const{return m_visible!=0;}
   const palyer_data_t& LastEnemyData()const{return m_last_enemy_data;}
   const palyer_data_t& LastOwnerData()const{return m_last_owner_data;}
   float LastVisTime()const{return m_last_time_vis;}
};

// true если 1й менее опасен чем второй
using enemy_compare_t=std::function<bool(const CEnemyData&,const CEnemyData&)>;
using enemy_data_ptr=std::unique_ptr<CEnemyData>;

bool DefaultEnemyCompare(const CEnemyData&,const CEnemyData&);

class CEnemyDataPool
{
   std::vector<enemy_data_ptr> m_data_pool;
   enemy_compare_t m_comparer=DefaultEnemyCompare;
   const CBot& m_owner; // кому принадлежит
   typename decltype(m_data_pool)::iterator m_current;
   bool is_new_current=false;
   public:
   using size_type=decltype(m_data_pool)::size_type;
   explicit CEnemyDataPool( const CBot&);
   CEnemyDataPool(const CEnemyDataPool&)=delete;
   CEnemyDataPool&operator=(const CEnemyDataPool&)=delete;
   void Run();

   const CBot&Owner()const{return m_owner;}

   const std::vector<enemy_data_ptr>& Enemies()const{return m_data_pool;}
   size_type NumVisible()const;
   const CEnemyData& Current()const;
   void SetEnemiesComparer(enemy_compare_t);
   void Update();

   bool IsEnemy(const player_t& enm)const;
   void Add(const player_t& enm);
   void Erase(const player_t& enm);
   void Clear(){m_data_pool.clear();}
   bool DispatchNewCurrent();
};

Vector PriorityViewPoint(const CEnemyDataPool&);


view_getter_t fn_shooting_viewing(const CEnemyDataPool&data);
act_getter_t fn_shooting_action(const CEnemyDataPool&data);

class CDamageRegister
{
    const CEnemyDataPool& m_enemies;
    float m_last_health;
    float m_last_armour;
    bool m_alarm;
    float m_time_damage;
    bool m_vis_enemy;
    public:
    // угол обзора бота
    Vector m_view_angle;
    // был ли при этом виден враг

    explicit CDamageRegister(const CEnemyDataPool&);

    CDamageRegister(const CDamageRegister&)=delete;
    CDamageRegister&operator=(const CDamageRegister&)=delete;
    void Run();
    void Update();

    // последнее повреждение было нанесено невидимым врагом
    // не более чем time секунд тому назад
    bool IsDamage(float time=100)const
    {return m_alarm&&(Time()<(m_time_damage+time));}
    void UndoAlarm(){m_alarm=false;}
    const Vector& ViewAngleDamage()const{return m_view_angle;}
};

CViewer::view_task_t LookBackViewing(const CDamageRegister&);

class CExtUseWall:public CFSMTask
{
    CUseHealthWall m_use_health;
    CUseArmourWall m_use_armour;
    CDamageRegister& m_damage_reg;
    CViewer m_viewer;
    const CItemPoint* m_wall;
    CFSMTask::fsm_state_t j_use_wall(CTask*task);
    CFSMTask::fsm_state_t j_viewer(CTask*task);

    virtual bool IsNativeComplete()const override;
    public:
    enum state_t
    {
        id_use_wall_st=0,
        id_view_st
    };
    const CItemPoint* Wall()const;
    CExtUseWall(CDamageRegister&);
    void SetWall(const CItemPoint*);
};

float ItemPriority(uint_t ,const CBot&);//0.0...1.0

class CVisitedItemsPoints
{
    float m_max_memoized_time;
    const int m_max_size=20;
    using record_t=std::pair<const CItemPoint*,float>;
    std::deque<record_t> m_visited_items;
    public:
    explicit CVisitedItemsPoints(float max_time=60.0f):
    m_max_memoized_time(max_time)
    {
        /*m_visited_items={{nullptr,1.0},
                         {nullptr,1.0},
                         {nullptr,1.0},
                         {nullptr,1.0}};
        m_visited_items.pop_back();
        m_visited_items.pop_back();
        m_visited_items.clear();*/
    }
    void RegVisit(const CItemPoint*);
    const std::deque<record_t>& Records()const
    {
        return m_visited_items;
    }
    std::optional<float> TimeLastVisit(const CItemPoint*)const;
};

std::tuple<uint_t,uint_t,uint_t> RangeNeededItems(const CBot&);

class CPriorityItemSearch
{
    std::vector<std::pair<const CItemPoint*,uint_t>> m_finded;
    const CVisitedItemsPoints& m_visited;
    bool find_most_wanted(const CWayPoint&);
    struct copy_wrapper
    {
        CPriorityItemSearch& m_impl;
        copy_wrapper(CPriorityItemSearch& impl):
        m_impl(impl){}
        bool operator()(const CWayPoint&wp)
        {
            return m_impl.find_most_wanted(wp);
        }
    };
    public:
    explicit CPriorityItemSearch(uint_t high,uint_t medial,uint_t low,
                        const CVisitedItemsPoints&);
    const CItemPoint* SuitableItem()const;
    copy_wrapper GetWrapper()
    {
        return copy_wrapper(*this);
    }
};

std::pair<CBaseRoute::route_param_t,const CItemPoint*> RouteToPriorityItems(const CBot&bot,
                                                          const CVisitedItemsPoints&);

class CDeathActivity
{
    action_t m_action=0;
    public:
    action_t operator()(const CTask&);
};

class CTotter:public CTask
{
    const CEnemyDataPool& m_enemies;
    virtual void NativeRun(CBot& bot)override;
    virtual move_t GetNativeMove()const override;
    virtual action_t GetNativeActivity()const override;
    public:
    struct param_t
    {
        float max_move_time=1.5f;
        float max_jump_time=4.0f;
    };
    private:
    param_t m_params;
    float m_next_change_move_time;
    mutable float m_next_jump_time;
    bool m_move_right;
    public:
    explicit CTotter(const CEnemyDataPool&);
    virtual void Update()override;
    void SetParams(const param_t&);
};

float DefBattleBalanse(const CBot&,const player_t&enemy);


class CMainTask;

class CRoute:public CTask
{
    CSearchInTransposed m_tr_search;
    const CVisitedItemsPoints &m_visited_points;
    const CWayPoint*m_last_node=nullptr;
    const CItemPoint* m_goal_item=nullptr;

    CBaseRoute m_base_route;
    CRouteViewer m_route_viewer;
    bool m_native_viewing=true;

    virtual void NativeRun(CBot&bot)override;
    virtual move_t GetNativeMove()const override;
    virtual action_t GetNativeActivity()const override;
    virtual Vector GetNativeView()const override;
    virtual void NativeClose()override;


    virtual bool IsNativeComplete()const override;

    public:
    explicit CRoute(CMainTask&);

    virtual void Update();
    virtual void SetView(view_getter_t)override;

    bool SetToPriority();
    bool UpdateCurrent(fn_b_wpref_t node_filter=nullptr,
                       edge_filter_t edge_filter=nullptr);
    void SetToVector(const Vector&);
    const CWayPoint& CurrentLastNode()const;
    const CItemPoint* CurrentGoal()const;
    const Vector&Destination()const;
};

class CPursuit:public CFSMTask
{
    CEnemyDataPool& m_enemies;
    CDamageRegister& m_damage_register;

    CRoute& m_route;
    CTotter m_totter;
    CViewer m_look_back;
    const float m_min_needed_dist=200.0f;
    const float m_max_for_update=300.f;
    const float m_max_needed_dist=400.0f;

    CFSMTask::fsm_state_t j_from_vis_enemy(CTask*);
    CFSMTask::fsm_state_t j_from_hide_enemy(CTask*);
    CFSMTask::fsm_state_t j_from_check_damage(CTask*);
    CFSMTask::fsm_state_t j_from_totter(CTask*);

    bool m_UpdateRoute();

    virtual void NativeRun(CBot&bot)override;
    virtual void NativeClose()override;
    virtual bool IsNativeComplete()const override;
    public:
    enum state_t
    {
       id_enemy_vis_st=0,
       id_enemy_hide_st,
       id_check_damage,
       id_totter_st
    };
    explicit CPursuit(CMainTask&);
};

class CFightRoute:public CFSMTask
{
    CEnemyDataPool& m_enemies;
    CDamageRegister& m_damage_register;

    CRoute& m_route;
    CTotter m_totter;
    CViewer m_look_back;

    CFSMTask::fsm_state_t j_from_route(CTask*);
    CFSMTask::fsm_state_t j_from_check_damage(CTask*);
    CFSMTask::fsm_state_t j_from_totter(CTask*);

    virtual void NativeRun(CBot&bot)override;
    virtual void NativeClose()override;
    public:
    enum state_t
    {
       id_route_st=0,
       id_check_damage,
       id_totter_st
    };
    explicit CFightRoute(CMainTask&);
};

class CMainRoute:public CFSMTask
{
    CEnemyDataPool& m_enemies;
    CDamageRegister& m_damage_register;

    CRoute& m_route;
    view_getter_t m_route_viewing=nullptr;
    CTotter m_totter;
    CViewer m_look_back;
    const float m_min_dist_for_totter=200.0f;

    CFSMTask::fsm_state_t j_from_route(CTask*);
    CFSMTask::fsm_state_t j_from_enemy_vis(CTask*);
    CFSMTask::fsm_state_t j_from_check_damage(CTask*);
    CFSMTask::fsm_state_t j_from_totter(CTask*);

    virtual void NativeRun(CBot&bot)override;
    public:
    enum state_t
    {
       id_route_st=0,
       id_enemy_vis_st,
       id_check_damage,
       id_totter_st
    };
    explicit CMainRoute(CMainTask&);
    void SetRouteViewing(view_getter_t);
};

class CBotsPool;
class CMainTask:public CTask
{
    CEnemyDataPool      m_enemies;
    CDamageRegister     m_damage_register;
    CVisitedItemsPoints m_visited_points;

    CRoute m_route;

    CTask       m_spawn_task;
    CMainRoute  m_calm_route;
    CFightRoute m_fight_route;
    CPursuit    m_pursuit;
    CExtUseWall m_use_wall;

    const CItemPoint* m_route_goal;
    std::unique_ptr<CFSMTask> m_main;

    CFSMTask::fsm_state_t j_from_spawn(CTask*);
    CFSMTask::fsm_state_t j_from_calm_route(CTask*);
    CFSMTask::fsm_state_t j_from_fight_route(CTask*);
    CFSMTask::fsm_state_t j_from_pursuit(CTask*);
    CFSMTask::fsm_state_t j_from_go_away(CTask*);
    CFSMTask::fsm_state_t j_from_use_wall(CTask*);

    void jump_action(CFSMTask::fsm_state_t,CFSMTask::fsm_state_t);

    virtual void NativeRun(CBot&bot)override;
    virtual bool IsNativeComplete()const override{return false;}

    virtual move_t GetNativeMove()const override;
    virtual action_t GetNativeActivity()const override;
    virtual Vector GetNativeView()const override;

    public:
    enum states_t
    {
        id_spawn_st=0,
        id_calm_route_st,
        id_fight_route_st,
        id_pursuit_st,
        id_go_away,
        id_use_wall_st
    };
    explicit CMainTask(CBot&bot);
    virtual void Update()override;
    friend class CRoute;
    friend class CPursuit;
    friend class CFightRoute;
    friend class CMainRoute;
    friend class CBotsPool;
};
void SelectWeapon(const CBot&);

class CBotsPool
{
    game_config_t m_game_config;
    bool m_config_parse=false;
    struct bot_data_t
    {
        std::unique_ptr<CBot> bot;
        CMainTask* main_task;
    };

    std::vector<bot_data_t> m_bots;
    static bool m_me_visible;


    bool m_Add(const std::string&name,float,float);

    public:
    const decltype(m_bots)::size_type m_max_bots=15;
    bool Add();
    void SetMeVisible(bool);
    void SetAccuracy(int);
    decltype(m_bots)::size_type Size()const
    {
        return m_bots.size();
    }
    void Clear();
    void DoThink();
};
inline CBotsPool glBots;
inline bool CBotsPool::m_me_visible=true;

#endif //_bot_fight
