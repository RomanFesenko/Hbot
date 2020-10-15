#ifndef  _bot_main_
#define  _bot_main_

#include <functional>
#include <optional>


#include "util.h"
#include "mapgraph.h"
#include "../../BaseLibraries/GEOMETRY/primitives_3D.h"


enum TaskType
{
    id_route_sch,
    id_over_obstacle_sch,
};

using move_t=std::optional<Vector>;
using action_t=unsigned short;


class Duration
{
    float m_duration;
    public:
    explicit Duration(float dur):m_duration(dur){}
    operator float()const{return m_duration;}
};

class CBot;
class executor_t
{
    CBot& m_bot;
    const float m_jump_tact=0.5;
    float m_next_jump=0.0f;
    Vector EvalViewAngle(const Vector&needed) const;
    float m_TurnVelocity(float )const;

    public:
    const float Velocity()const{return 220.0f;}
    executor_t(CBot& bot):m_bot(bot)
    {}
    void operator()(const std::optional<Vector>& _move,action_t act,Vector view);
};

const float Max_disperse_angle=4.5f;
const float Min_turn_vel=225.0f;
const float Max_turn_vel=275.0f;
const float Max_disperse_target=1.6f;


class CTask;
class CBot
{
    player_t m_player;
    executor_t m_executor;

    struct bot_chars_t
    {
        float shoot_disperse=2.25;
        float turn_velocity=250;
        float target_disperse=1.3;
        float aggression=0.0f;
    };
    bot_chars_t m_chars;
    std::unique_ptr<CTask> m_exe_task;

    const float m_view_cos=cos(85*Degree);
    public:
    CBot(const std::string& name,float,float);

    void Think();
    //обработка глобальных алармов и алармов специфических
    //для исполняемой темы
    Vector Center()const{return m_player.Center();}
    Vector Eyes()const{return m_player.Eyes();}
    Vector ViewAngle()const{return m_player.ViewAngle();}
    float Health()const{return m_player.Health();}
    float Armour()const{return m_player.Armour();}
    float IsDead()const{return m_player.IsDead();}
    uint_t ActiveItem()const{return m_player.ActiveItem();}
    const player_t& Player()const{return m_player;}
    void SelectItem(int item_flag){m_player.SelectItem(item_flag);}
    bool IsVisible(const Vector& vec)const;
    bool IsForwardVisible(const Vector& vec)const;
    Vector AnglesMoveTo(const Vector& dest)const
    {
        return VecToAngles(dest-Center());
    }

    Vector AnglesViewTo(const Vector& dest)const
    {
        return VecToAngles(dest-Eyes());
    }

    float NormalVelocity(int type_move=0)const
    {
        return (type_move==0)? 240.0:120;
    }
    const bot_chars_t& Chars()const{return m_chars;}
    void SetAccuracy(float);

    void SetExecuteTask(CTask* task);
    ~CBot();
    friend class executor_t;

    friend class CTask;
    // Тестовое
    //void RunRoute(const Vector& from,const Vector&to);

};

bool IsNear(const CBot& bot,const Vector& pos,
            float hor_dist=30.0,float vert_dist=60.0);


struct task_deleter
{
    void operator()(CTask*task);
};

using task_ptr_t=std::unique_ptr<CTask,task_deleter>;


using move_getter_t=std::function<move_t(const CTask&)>;
using view_getter_t=std::function<Vector(const CTask&)>;
using act_getter_t=std::function<action_t(const CTask&)>;
using end_getter_t=std::function<bool(const CTask&)>;

/*Иерархия CTask
CTask
    CMoveToPoint
    CMoveOnStairs
    COvercomeObstacle
    CTotter
    CFSMTask
        CTaskChain
            CUseButton
                CUseHealthWall
                CUseArmourWall
                CUseElevator
        CViewer
        CBaseRoute
        CRouteViewer
        CExtUseWall
        CPursuit
        CFightRoute
        CMainRoute
*/


class CTask
{
    static int m_total_num_tasks;
    public:
    // флаги указывающие на возможность переопределения
    // NativeMove,NativeActivity,NativeView
    enum maybe_redef_t
    {
        id_redef_move_t=1<<0,
        id_redef_act_t=1<<1,
        id_redef_view_t=1<<2,
        id_added_act_t=1<<3,
    };
    protected:
    bool m_is_run=false;
    float m_time_start;
    CBot* m_bot=nullptr;
    int m_redef_flags;
    // опциональные переопределения исходных реакций
    move_getter_t m_added_move=nullptr;
    act_getter_t m_added_action=nullptr;
    view_getter_t m_added_viewing=nullptr;
    end_getter_t m_added_terminator=nullptr;

    virtual void NativeRun(CBot& bot){}
    virtual void NativeClose(){}

    // двигаться ли, и если двигаться то куда
    virtual move_t GetNativeMove()const{return {};}

    // набор флагов из hl_iface.h: прыгать,стрелять,перезаряжать итд
    virtual action_t GetNativeActivity()const{return 0;}

    // желательный угол обзора
    virtual Vector GetNativeView()const
    {
        return m_bot->ViewAngle();
    }

    virtual bool IsNativeComplete()const
    {
        return true;
    }

    public:
    CTask();
    CBot& Bot();
    const CBot& Bot()const;
    void SetBot(CBot&);
    bool MayBeRedefNative(int flag)const;
    void ForbidRedef(int);
    virtual bool MayBeRedef(int)const;

    virtual void Update(){}

    void Run(CBot& bot);
    void Close();
    bool IsRun()const{return m_is_run;}
    virtual move_t GetMove()const;
    virtual action_t GetActivity()const;
    virtual Vector GetView()const;
    bool IsComplete()const;

    //переопределение GetNativeMove,GetNativeView,GetNativeActivity
    void SetMove(move_getter_t);
    void SetActivity(act_getter_t,maybe_redef_t redef=id_redef_act_t);
    virtual void SetView(view_getter_t);
    void SetTerminator(end_getter_t);

    float Duration()const
    {
        return Time()-m_time_start;
    }
    void SelectItem(int iden){m_bot->m_player.SelectItem(iden);}
    virtual ~CTask();
};
inline int CTask::m_total_num_tasks=0;

class CFSMTask:public CTask
{
    public:
    using fsm_state_t=std::pair<CTask*,int>;
    using jump_func_t=std::function<fsm_state_t(CTask*)>;

    bool m_CheckAccess(int)const;

    protected:
    std::vector<jump_func_t> m_jumps;

    //опциональная функция выполняемая при
    // переходе из одного состояния в другое
    std::function<void(fsm_state_t,fsm_state_t)> m_jump_action=nullptr;

    int m_current_state;
    decltype(m_jumps)::iterator m_current_jump;
    CTask* m_current=nullptr;

    virtual void NativeRun(CBot&bot)override;
    virtual void NativeClose()override;

    virtual move_t GetNativeMove()const override final;
    virtual action_t GetNativeActivity()const override final;
    virtual Vector GetNativeView()const override final;

    public:
    // текущее состояние и функция переходов
    using jump_t=std::pair<int,jump_func_t>;
    using jump_table_t=std::vector<jump_t>;
    CFSMTask(){};

    void SetJumpTable(CTask*begin_state,int begin_ind,
                        jump_table_t jump_table);

    void SetJumpAction(std::function<void(fsm_state_t,fsm_state_t)>);

    int CurrentState()const{return m_current_state;}
    const CTask& CurrentTask()const {return *m_current;}
    virtual void Update()override final;

    virtual move_t GetMove()const override final;
    virtual action_t GetActivity()const override final;
    virtual Vector GetView()const override final;
    virtual bool MayBeRedef(int)const override final;

    void SetForced(CTask*,int);
    virtual ~CFSMTask(){}
};

class CTaskChain:public CFSMTask
{
    std::vector<std::pair<CTask*,int>> m_tasks;
    decltype(m_tasks)::iterator m_current_iter;
    std::pair<CTask*,int> j_update_chain(CTask*);
    virtual bool IsNativeComplete()const override;
    public:
    CTaskChain(){};
    void SetChain(std::vector<std::pair<CTask*,int>> chain);
    virtual ~CTaskChain(){}
};


class CMoveChecker
{
    Vector m_goal;
    const CBot*m_bot=nullptr;
    float m_period_check;
    std::optional<float>  m_medial_vel={};
    float m_last_dist;
    float m_next_check;
    public:
    CMoveChecker(){};

    void Reset(const Vector& goal,
               float delta_check=1.5);

    std::optional<float> MedialVelocity()const{return m_medial_vel;}
    void Update();
    void Run(const CBot&);
};

class CViewer:public CFSMTask
{
    public:
    enum view_t{id_point,id_angle};
    struct view_task_t
    {
        Vector view_vector;
        float duration;
        view_t type;// точка или углы
    };
    private:
    virtual void NativeRun(CBot&bot)override;
    virtual bool IsNativeComplete()const override;
    std::vector<view_task_t> m_schedule;
    decltype(m_schedule)::iterator m_current_view;
    CTask m_current_task;
    void m_UpdateView();
    fsm_state_t jump_update(CTask*);
    public:
    CViewer();
    void SetView(const std::vector<view_task_t>&);

};

class CMoveToPoint:public CTask
{
    CMoveChecker m_move_checker;
    public:
    struct move_to_point_t
    {
        Vector point;
        action_t type=0;
        float hor_precision=30;
        float vert_precision=60;
    };
    private:
    move_to_point_t m_data;
    public:
    CMoveToPoint( );

    void ResetGoal(move_to_point_t to_point);
    const move_to_point_t& Data()const{return m_data;}
    std::optional<float> MedialVelocity()const;
    virtual void Update()override;

    private:
    virtual void NativeRun(CBot&)override;
    virtual move_t GetNativeMove()const  override;
    virtual action_t GetNativeActivity()const  override;
    virtual Vector GetNativeView()const override;
    virtual bool IsNativeComplete()const override;
};

class CMoveOnStairs:public CTask
{
    using iterator=CWayPoint::iterator;
    public:
    const float Angle=5*Degree;
    private:
    const CWayPoint*m_down_wp;
    const CWayPoint*m_up_wp;
    line_3D_t<Vector> m_line;
    Vector m_normal_angle;
    Vector m_destination;
    const float m_max_dist_to_edge=15.0f;
    bool m_is_up;
    public:
    CMoveOnStairs();

    void ResetGoal(const edge_path_t& stairs);

    private:
    virtual move_t GetNativeMove()const override;
    virtual Vector GetNativeView()const override;
    virtual bool IsNativeComplete()const override;
};


class COvercomeObstacle:public CTask
{
    public:
    struct obstacle_t
    {
        Vector from_vec;
        Vector to_vec;
        action_t type_move;
    };
    private:
    float m_max_deviation;
    const float m_angle_to_direct=30.0*Degree;
    obstacle_t m_data;
    Vector m_current_delta;

    mutable float m_next_jump_time;
    float m_next_update_time;

    public:
    COvercomeObstacle();
    const obstacle_t&Data()const{return m_data;}

    void ResetGoal(obstacle_t data,float max_deviation=90*Degree);

    virtual void Update()override;
    private:

    virtual void NativeRun(CBot&) override;
    virtual move_t GetNativeMove()const override;
    virtual action_t GetNativeActivity()const override;
    virtual Vector GetNativeView()const override;
    virtual bool IsNativeComplete()const override;
};

class CRouteTracker
{
    public:
    virtual void ToStartBeginning(const Vector&)=0;
    virtual void ToEndBeginning(const Vector&)=0;
    virtual void NewEdgeBeginning(const edge_path_t& )=0;
    virtual ~CRouteTracker(){}
};

class CRouteViewer;
class CBaseRoute:public CFSMTask
{
    fsm_state_t update_route_task();
    fsm_state_t jump_to_obstacle_task(int prev_task);

    fsm_state_t jump_from_start_node(CTask*);
    fsm_state_t jump_from_on_route(CTask*);
    fsm_state_t jump_from_in_obstacle(CTask*);
    fsm_state_t jump_from_end_point(CTask*);

    std::unique_ptr<CPathGenerator> m_path_gen;
    void m_JumpMessage(fsm_state_t,fsm_state_t);
    edge_path_t m_current_edge;
    Vector m_vec_from;
    Vector m_vec_to;

    CMoveToPoint m_base_move;
    COvercomeObstacle m_obstacle_move;
    CMoveOnStairs m_stairs_move;
    task_ptr_t m_elevator_move;

    int m_put_off_task=-1;
    CMoveToPoint::move_to_point_t m_put_off_data;

    Vector DefObstacleData(const Vector&from,
                              const Vector&to);

    CRouteTracker* m_tracker=nullptr;
    public:
    enum RouteState
    {
        id_to_start_node_st=0,
        id_on_route_st,
        id_in_obstacle_st,
        id_staircase_st,
        id_on_elevator_st,
        id_to_end_point_st
    };
    struct route_param_t
    {
        Vector from_vec;
        CPathGenerator*path_gen=nullptr;
        Vector to_vec;
    };
    CBaseRoute();

    void Set(const route_param_t&);
    void SetTracker(CRouteTracker*);
    const Vector&Destination()const;
    private:
    virtual void NativeRun(CBot&bot)override;
    virtual bool IsNativeComplete()const override;
    friend class CRouteViewer;
};


class CRouteViewer:public CFSMTask,
                   public CRouteTracker
{
    CViewer m_viewer;

    std::optional<fsm_state_t> m_IsMainLook();
    std::optional<fsm_state_t> m_IsLookBack();

    fsm_state_t jump_from_view_point(CTask*);
    fsm_state_t jump_from_look_back(CTask*);
    fsm_state_t jump_from_incomplete_view(CTask*);
    fsm_state_t jump_from_main_view(CTask*);

    edge_path_t m_current_edge;
    std::optional<Vector> m_main_view;
    Vector m_point;
    float m_next_look_back;
    enum update_t
    {
        id_no_update,
        id_new_point,
        id_new_edge
    };
    update_t m_update=id_no_update;
    update_t m_DispatchUpdate();

    public:
    virtual void ToStartBeginning(const Vector&)override;
    virtual void ToEndBeginning(const Vector&)override;
    virtual void NewEdgeBeginning(const edge_path_t& )override;
    enum state_t
    {
        id_view_point=0,
        id_look_back,
        id_incomplete_view,
        id_main_view
    };
    CRouteViewer();
    void Set( CBaseRoute&);
};

move_getter_t fn_move(const Angles& dir);
move_getter_t fn_move(const Point&vec);
move_getter_t fn_take_position(const Vector&vec,float hor_dis=30.0,float vert_dist=30.0);

view_getter_t fn_viewing(const Angles& dir);
view_getter_t fn_viewing(const Point&vec);
view_getter_t fn_viewing(CTask&);


end_getter_t fn_duration_terminator(float);
end_getter_t fn_position_terminator(const Vector&dest,
                                       float hor_prec=30,float vert_prec=60);

act_getter_t fn_action(action_t act);
act_getter_t fn_action(CTask& act_task);


end_getter_t fn_view_terminator(const Angles& dir,
                                float prec=3*Degree);

end_getter_t fn_view_terminator(const Point&vec,
                                float prec=3*Degree);

end_getter_t fn_act_item_terminator(int item);

class CUseButton : public CTaskChain
{
    CTask m_set_view;
    protected:
    const CItemPoint* m_item=nullptr;
    void SetItemPoint(const CItemPoint*);
    public:
    enum state_t
    {
        id_set_view_st=0,
        id_use_button_st
    };
    CUseButton();
    virtual void RunUse(CBot&bot){};
    virtual void UpdateUse(){};
    virtual bool IsCompleteUse()const=0;
    private:
    class use_button_impl:public CTask
    {
        CUseButton* m_owner;
        public:
        use_button_impl(CUseButton* owner):m_owner(owner){}
        virtual void Update()override;
        private:
        virtual void NativeRun(CBot&bot)override;
        virtual bool IsNativeComplete()const override;
    };
    use_button_impl m_use_button;
    public:
    const CItemPoint* ButtonPoint()const;
    virtual ~CUseButton(){}
};

class CUseHealthWall:public CUseButton
{
    float m_period_check=1.5f;
    std::optional<int>  m_delta_health={};
    int m_last_health;
    float m_next_check;
    public:
    CUseHealthWall(){}
    void SetHealthWall(const CItemPoint*);
    virtual void RunUse(CBot&bot) override;
    virtual void UpdateUse() override;
    virtual bool IsCompleteUse()const override;
};

class CUseArmourWall:public CUseButton
{
    float m_period_check=1.5f;
    std::optional<int>  m_delta_armour={};
    int m_last_armour;
    float m_next_check;
    public:
    CUseArmourWall(){}
    void SetArmourWall(const CItemPoint*);
    virtual void RunUse(CBot&bot) override;
    virtual void UpdateUse() override;
    virtual bool IsCompleteUse()const override;
};


class CUseElevator:public CUseButton
{
    edge_path_t m_elevator_edge;
    public:
    CUseElevator(const edge_path_t&);
    virtual bool IsCompleteUse()const override;
};


struct route_to_vector_param_t
{
    const CWayPoint*from_node;
    const CWayPoint*to_node;
    CBaseRoute::route_param_t route_param;
    operator CBaseRoute::route_param_t(){return route_param;}
};

route_to_vector_param_t
RouteToVector(const CBot&,const Vector&from_vec,const Vector&);

route_to_vector_param_t RandomRoute(const CBot&);

#endif //_bot_main_
