#include "bot_main.h"
#include "graph_editor.h"
#include "graph_editor.h"
#include <typeinfo>

const float Tact=0.1f;

bool IsNear(const CBot& bot,const Vector& pos,
            float hor_dist,float vert_dist)
{
    Vector delta=bot.Center()-pos;
    return distance_2D(delta)<hor_dist&&abs(delta[2])<vert_dist;
}

// executor_t :: m_TurnVelocity - turn speed
// depending on the difference between the current and desired viewing angle

float executor_t::m_TurnVelocity(float delta_ang)const
{
    if(delta_ang>0.0f)
    {
        return (delta_ang>60.0f)? 360.0f:m_bot.m_chars.turn_velocity;// 250-norms
    }
    else
    {
        return (delta_ang<-60.0f)? -360.0f:-m_bot.m_chars.turn_velocity;
    }
}

// executor_t :: EvalViewAngle -calc. adjustable viewing angle
// depending on the current and desired viewing angle

Vector executor_t::EvalViewAngle(const Vector&needed) const
{
    Vector current=m_bot.ViewAngle();
    Vector delta=needed-current;
    delta[0]=NormalizeAngle(delta[0]);
    delta[1]=NormalizeAngle(delta[1]);

    float delta_x=m_TurnVelocity(delta[0])*Tact;
    float delta_y=m_TurnVelocity(delta[1])*Tact;

    if(abs(delta_x)>abs(delta[0])) {delta_x=delta[0];}
    if(abs(delta_y)>abs(delta[1])) {delta_y=delta[1];}
    return Vector(current[0]+delta_x,current[1]+delta_y,0.0f);
}


void executor_t::operator()(const std::optional<Vector>& _move,action_t action,Vector view)
{
    float disp=m_bot.m_chars.shoot_disperse;
    Vector eval_set=EvalViewAngle(view);
    Vector disp_set=eval_set;

    if(action&IN_ATTACK)
        disp_set=eval_set+Vector(GetRandomFloat(-disp,disp),
                                 GetRandomFloat(-disp,disp),
                                 0);
    m_bot.m_player.SetViewAngle(disp_set);

    float vel; Vector move_dir;

    if(_move)
    {
        vel=Velocity();
        move_dir=*_move;
        action|=IN_FORWARD;
    }
    else
    {
        action&=(~IN_FORWARD);
        vel=0;
        move_dir=Vector(0,0,0);
    }
    if((action&IN_JUMP)!=0)
    {
        float current=Time();
        if(current<m_next_jump)
        {
            action&=(~IN_JUMP);
        }
        else
        {
            m_next_jump=current+m_jump_tact;
        }
    }
    m_bot.m_player.Move(move_dir,vel,action,Tact*1000);
    if(action&IN_ATTACK)
       m_bot.m_player.SetViewAngle(eval_set);
}

//CBot

CBot::CBot(const std::string& name,float accuracy,float aggres):
m_player(CreateFakePlayer(name)),
m_executor(*this)
{
    SetAccuracy(accuracy);
    m_chars.aggression=restrict_range(aggres,0.0f,1.0f)-0.5f;
}

void CBot::SetAccuracy(float accur)
{
    float norm_acc=restrict_range(accur,0.0f,1.0f);

    m_chars.shoot_disperse=Max_disperse_angle*(1.0-norm_acc);

    m_chars.turn_velocity=Min_turn_vel+
    (Max_turn_vel-Min_turn_vel)*norm_acc;

    m_chars.target_disperse=1.0+(Max_disperse_target-1.0)*(1.0-norm_acc);
}

bool CBot::IsForwardVisible(const Vector& vec)const
{
    return Visible(Eyes(),vec,m_player);
}

bool CBot::IsVisible(const Vector& vec)const
{
    Vector direct_view=AnglesToVec(ViewAngle());
    Vector delta=vec-Eyes();
    return IsForwardVisible(vec)&&
    dot_product_3D(direct_view,delta)>m_view_cos*distance_3D(delta);
}

void CBot::Think()
{
    if(m_exe_task==nullptr)
    {
        ServerPrint("No task for executing \n");
        return;
    }
    if(!m_exe_task->IsRun())
    {
        m_exe_task->Run(*this);
    }
    m_exe_task->Update();
    m_executor(m_exe_task->GetMove(),
               m_exe_task->GetActivity(),
               m_exe_task->GetView());
}

void CBot::SetExecuteTask(CTask* task)
{
    if(m_exe_task!=nullptr)
    {
        m_exe_task->Close();
    }
    m_exe_task.reset(task);
}

CBot::~CBot()
{
    if(m_exe_task!=nullptr)
    {
        m_exe_task->Close();
    }
}
///CTask

CTask::CTask()
{
    m_redef_flags=id_redef_move_t|
                   id_redef_act_t|
                   id_redef_view_t;
    m_total_num_tasks++;
}

void CTask::SetBot(CBot&bot)
{
    m_bot=&bot;
}

void CTask::Run(CBot&bot)
{
    if constexpr(BOT_DEBUG)
    {
        if(m_is_run)
        {
            ReportPrint("Repeat task running:"+std::string(typeid(*this).name())+'\n');
            assert(false);
        }
    }
    m_bot=&bot;
    m_time_start=Time();
    m_is_run=true;
    NativeRun(bot);
}

const CBot& CTask::Bot()const
{
    if constexpr(BOT_DEBUG)
    {
        if(m_bot==nullptr)
        {
            ReportPrint("Error bot init in task:"+std::string(typeid(*this).name())+'\n');
            assert(false);
        }
    }
    return *m_bot;
}

CBot& CTask::Bot()
{
    if constexpr(BOT_DEBUG)
    {
        if(m_bot==nullptr)
        {
            ReportPrint("Error bot init in task:"+std::string(typeid(*this).name())+'\n');
            assert(false);
        }
    }
    return *m_bot;
}

void CTask::Close()
{
    m_is_run=false;
    NativeClose();
}

move_t CTask::GetMove()const
{
    return (m_added_move==nullptr)?
            GetNativeMove():
            m_added_move(*this);
}

action_t CTask::GetActivity()const
{
    return (m_added_action==nullptr)?
            GetNativeActivity():
            m_added_action(*this);
}

Vector CTask::GetView()const
{
    return (m_added_viewing==nullptr)?
            GetNativeView():
            m_added_viewing(*this);
}

bool CTask::IsComplete()const
{
    return (m_added_terminator==nullptr)?
            IsNativeComplete():
            m_added_terminator(*this);
}

bool CTask::MayBeRedefNative(int flag)const
{
    return m_redef_flags&flag;
}

// CTask :: ForbidRedef - prohibition of overriding reaction

void CTask::ForbidRedef(int flag)
{
    m_redef_flags&=~flag;
}

bool CTask::MayBeRedef(int flag)const
{
    return MayBeRedefNative(flag);
}

void CTask::SetMove(move_getter_t move_getter)
{
    m_added_move=move_getter;
}


void CTask::SetActivity(act_getter_t act_getter,maybe_redef_t redef)
{
    if(redef==id_redef_act_t)
      {m_added_action=act_getter;}
    else if(redef==id_added_act_t)
    {
        assert(act_getter!=nullptr);
        m_added_action=[act_getter](const CTask&_this)
        {
             return act_getter(_this)|_this.GetNativeActivity();
        };
    }
    else{assert(false);}
}

void CTask::SetView(view_getter_t view_getter)
{
    m_added_viewing=view_getter;
}

void CTask::SetTerminator(end_getter_t end_getter)
{
    m_added_terminator=end_getter;
}

CTask::~CTask()
{
    m_total_num_tasks--;
    if(m_total_num_tasks==0)
    {
        ReportPrint("All tasks deleted\n");
    }
}

void task_deleter::operator()(CTask*task)
{
    assert(!task->IsRun());
    delete task;
}

bool CFSMTask::m_CheckAccess(int index)const
{
    bool check_range=(index>=0&&index<m_jumps.size());
    if(!check_range)
    {
        ErrorPrint("Inadmissible state index in: "+
                    std::string(typeid(*this).name())+'\n');
    }
    return check_range;
}

/// CFSMTask
// Hierarchical state machine whose states are CTask
// which can also be CFSMTask, of arbitrary nesting degree
// CFSMTask :: SetJumpTable - setting the jump table
// each state has its own transition function

void CFSMTask::SetJumpTable(CTask*begin_state,int begin_ind,
             std::vector<jump_t> jump_table)
{
    m_jumps.clear();
    int index=0;
    for(auto &[from_ind,jump_func]:jump_table)
    {
        assert(from_ind==index);
        index++;
        m_jumps.push_back(jump_func);
    }
    if constexpr(BOT_DEBUG)
    {
        if(!m_CheckAccess(begin_ind))
        {
            assert(false);
        }
    }
    m_current=begin_state;
    m_current_state=begin_ind;
    m_current_jump=m_jumps.begin()+begin_ind;
}

// CFSMTask :: Update () - implementation of transitions in the machine itself
// and in all nested

void CFSMTask::Update()
{
    m_current->Update();// first transitions in all nested

    auto jump_res=(*m_current_jump)(m_current);


    CTask*preview_task=m_current;
    int preview_iden=m_current_state;
    bool state_change=false;

    // the executable CTask changes
    if(jump_res.first!=nullptr)
    {
        state_change=true;
        m_current->Close();
        m_current=jump_res.first;
        m_current->Run(Bot());
    }
    // it is possible to change the state without
    // change the executable CTask
    if(m_current_state!=jump_res.second)
    {
        if constexpr(BOT_DEBUG)
        {
            if(!m_CheckAccess(jump_res.second))
            {
                assert(false);
            }
        }
        state_change=true;
        m_current_state=jump_res.second;
        m_current_jump=m_jumps.begin()+jump_res.second;
    }

    // m_jump_action - the action to be performed on transition
    // to another state or when the executable CTask changes,
    // for example sending messages
    if(state_change&&m_jump_action!=nullptr)
    {
        m_jump_action({preview_task,preview_iden},
                      {m_current,m_current_state});
    }
}

void CFSMTask::SetJumpAction(std::function<void(fsm_state_t,fsm_state_t)> j_act)
{
    m_jump_action=j_act;
}


// CFSMTask :: SetForced - force setting
// CTask executable and current state

void CFSMTask::SetForced(CTask*task,int iden)
{
    if(m_current!=nullptr)
    {
        m_current->Close();
    }
    m_current=task;
    if constexpr(BOT_DEBUG)
    {
        if(!m_CheckAccess(iden))
        {
            assert(false);
        }
    }
    m_current_state=iden;
    m_current_jump=m_jumps.begin()+iden;
    assert(m_current_jump!=m_jumps.end());
}

void CFSMTask::NativeRun(CBot&bot)
{
    assert(m_current!=nullptr);
    m_current->Run(bot);
}

void CFSMTask::NativeClose()
{
    m_current->Close();
}

move_t CFSMTask::GetNativeMove()const
{
    return m_current->GetMove();
}

action_t CFSMTask::GetNativeActivity()const
{
    return m_current->GetActivity();
}

Vector CFSMTask::GetNativeView()const
{
    return m_current->GetView();
}

// Each request GetMove, GetActivity, GetView
// linearly depends on the nesting depth CFSMTask

move_t CFSMTask::GetMove()const
{
    if(m_added_move!=nullptr&&
       m_current->MayBeRedef(CTask::id_redef_move_t))
    {
        return m_added_move(*this);
    }
    else
    {
        return m_current->GetMove();
    }
}

action_t CFSMTask::GetActivity()const
{
    if(m_added_action!=nullptr&&
       m_current->MayBeRedef(CTask::id_redef_act_t))
    {
        return m_added_action(*this);
    }
    else
    {
        return m_current->GetActivity();
    }
}

Vector CFSMTask::GetView()const
{
    if(m_added_viewing!=nullptr&&
       m_current->MayBeRedef(CTask::id_redef_view_t))
    {
        return m_added_viewing(*this);
    }
    else
    {
        return m_current->GetView();
    }
}

bool CFSMTask::MayBeRedef(int flag)const
{
    return MayBeRedefNative(flag)&&m_current->MayBeRedef(flag);
}



/// CTaskChain
// sequential execution of the CTask series

void CTaskChain::SetChain(std::vector<std::pair<CTask*,int>> chain)
{
    assert(!chain.empty());
    assert(m_tasks.empty());
    jump_table_t jump_table;

    auto jump_func=[this](CTask*task)
    {
        return j_update_chain(task);
    };

    for(auto [task,iden]:chain)
    {
        m_tasks.push_back(std::make_pair(task,iden));
        jump_table.push_back({iden,jump_func});
    }
    SetJumpTable(m_tasks.front().first,m_tasks.front().second,
                 jump_table);
    m_current_iter=m_tasks.begin();
}


bool CTaskChain::IsNativeComplete()const
{
    return m_current_iter==m_tasks.end()-1&&m_current->IsComplete();
}

CFSMTask::fsm_state_t CTaskChain::j_update_chain(CTask*task)
{
    if((!task->IsComplete())||m_current_iter==m_tasks.end()-1)
    {
          assert(m_current_iter->second==CurrentState());
          return {nullptr,m_current_iter->second};
    }
    else
    {
        ++m_current_iter;
        return {m_current_iter->first,m_current_iter->second};
    }
}

///CMoveChecker
// keeps track of the average movement speed for m_period_check
// to m_goal

void CMoveChecker::Update()
{
    float current=Time();

    if(current<m_next_check) return;

    m_next_check=current+m_period_check;
    float new_dist=distance_3D(m_goal-m_bot->Center());
    m_medial_vel=(m_last_dist-new_dist)/m_period_check;
    m_last_dist=new_dist;
}

void CMoveChecker::Reset(const Vector& goal,
                         float delta_check)
{
    m_goal=goal;
    m_period_check=delta_check;
}

void CMoveChecker::Run(const CBot&bot)
{
    m_bot=&bot;
    m_medial_vel={};
    m_last_dist=distance_3D(m_goal-m_bot->Center());
    m_next_check=Time()+m_period_check;
}

///CViewer
// sequential execution of a series of reviews
// where to look for how long

CViewer::CViewer()
{
    SetJumpTable(&m_current_task,0,{
                 {0,[this](CTask*task){return jump_update(task);}}
                 });
}

void CViewer::m_UpdateView()
{
    end_getter_t dur_term=fn_duration_terminator(m_current_view->duration);
    end_getter_t view_term;
    if(m_current_view->type==id_point)
    {
        m_current_task.SetView(fn_viewing(Point(m_current_view->view_vector)));
        view_term=fn_view_terminator(Point(m_current_view->view_vector));
    }
    else
    {
        m_current_task.SetView(fn_viewing(Angles(m_current_view->view_vector)));
        view_term=fn_view_terminator(Angles(m_current_view->view_vector));
    }
    m_current_task.SetTerminator
    ([dur_term,view_term](const CTask&task)
     {
         return dur_term(task)&&view_term(task);
     });
}

void CViewer::SetView(const std::vector<view_task_t>&viewers)
{
    assert(!viewers.empty());
    m_current_task.Close();
    m_schedule=viewers;
    m_current_view=m_schedule.begin();
    m_UpdateView();
}

CFSMTask::fsm_state_t CViewer::jump_update(CTask*task)
{
    if(!task->IsComplete()||
       m_current_view==m_schedule.end()-1) {return {nullptr,0};}

    ++m_current_view;
    m_UpdateView();
    return {&m_current_task,0};
}

void CViewer::NativeRun(CBot&bot)
{
    m_current_task.Run(bot);
    assert(!m_schedule.empty());
    m_current_view=m_schedule.begin();
    m_UpdateView();
}

bool CViewer::IsNativeComplete()const
{
    return  (m_current_view==m_schedule.end()-1)&&
             m_current_task.IsComplete();
}

/// CMoveToPoint
// move to a fixed point with tracking
// movement speed

CMoveToPoint::CMoveToPoint( )
{
    ForbidRedef(id_redef_move_t);// you cannot override the movement method.
}

void CMoveToPoint::ResetGoal(move_to_point_t to_point)
{
    m_data=to_point;
    m_move_checker.Reset(to_point.point,1.5);
}

std::optional<float> CMoveToPoint::MedialVelocity()const
{
    return m_move_checker.MedialVelocity();
}

void CMoveToPoint::Update()
{
    m_move_checker.Update();
}

void CMoveToPoint::NativeRun(CBot&bot)
{
    m_move_checker.Run(bot);
}

move_t CMoveToPoint::GetNativeMove()const
{
    return VecToAngles(m_data.point-Bot().Center());
}

action_t CMoveToPoint::GetNativeActivity()const
{
    return m_data.type;
}

Vector CMoveToPoint::GetNativeView()const
{
    return VecToAngles(m_data.point-Bot().Center());
}

bool CMoveToPoint::IsNativeComplete()const
{
    return IsNear(Bot(),m_data.point,m_data.hor_precision,
                                     m_data.vert_precision);
}

/// CMoveOnStairs
// move along a steep staircase, the algorithm is based
// keeping a small distance between
// bot and a line connecting the beginning. and finally. points

CMoveOnStairs::CMoveOnStairs():
m_line(Vector(0,0,0),Vector(0,0,1))
{
    ForbidRedef(id_redef_move_t);
}

void CMoveOnStairs::ResetGoal(const edge_path_t& stairs)
{
    m_down_wp=stairs.from_node;
    m_up_wp=stairs.to_node;

    m_line.set(m_down_wp->Coord(),
               m_up_wp->Coord());
    m_is_up=m_down_wp->Coord()[2]<m_up_wp->Coord()[2];
    if(!m_is_up)
    {
        std::swap(m_down_wp,m_up_wp);
    }
    assert(m_down_wp->size_adjacents()==2);

    iterator iter_down;
    for(iter_down.set_base(*m_down_wp);!iter_down.end();iter_down.next())
    {
        if(&iter_down.to_node()!=m_up_wp)
        {
            Vector move_dir=turn_in_plane(Vector(0,0,1),
                                         m_down_wp->Coord()-iter_down.to_node().Coord(),
                                         Angle);
            if(!m_is_up) move_dir=-move_dir;
            m_normal_angle=VecToAngles(move_dir);
        }
    }
    m_destination=(m_is_up)?m_up_wp->Coord():m_down_wp->Coord();
}


move_t CMoveOnStairs::GetNativeMove()const
{
    Vector project_pt;
    Vector dist_to_line=distance_3D(m_line,Bot().Center(),project_pt);
    if(dist_to_line[0]<m_max_dist_to_edge)
    {
        return m_normal_angle;
    }

    float base_nz=(m_is_up)? 1.0f:-1.0f;
    return VecToAngles(turn_in_plane(Vector(0,0,base_nz),
                                     project_pt-Bot().Center(),
                                     Angle));

}

Vector CMoveOnStairs::GetNativeView()const
{
    return m_normal_angle;
}

bool CMoveOnStairs::IsNativeComplete()const
{
    return distance_3D(Bot().Center()-m_destination)<20.0f;
}

/// COvercomeObstacle
// overcoming an obstacle on the way to m_data.to_vec,
// the algorithm is based on adding a small
// perpendicular to the current
// vector m_data.to_vec-bot.Center ()

COvercomeObstacle::COvercomeObstacle()
{
     ForbidRedef(id_redef_move_t);
}

void COvercomeObstacle::NativeRun(CBot&bot)
{
    Update();
    m_next_jump_time=GetRandomFloat(0.0f,1.0f)+Time();
}

void COvercomeObstacle::Update()
{
    m_current_delta=turn_around_axis(m_data.from_vec-m_data.to_vec,
                                     glOrtZ,
                                     GetRandomFloat(-m_max_deviation,
                                                   m_max_deviation));
    m_next_update_time=Time()+GetRandomFloat(2.5f,5.0f);
}

void COvercomeObstacle::ResetGoal(obstacle_t data,float dev)
{
    m_data=data;
    m_max_deviation=dev;
}

move_t COvercomeObstacle::GetNativeMove()const
{
    Vector direct=Bot().Center()-m_data.to_vec;

    Vector cr_pr=cross_product_3D(m_current_delta,direct);

    float ang_turn=(cr_pr[2]>0.0f)?m_angle_to_direct:-m_angle_to_direct;
    return VecToAngles(turn_around_axis(-direct,
                                        glOrtZ,
                                        ang_turn));
}

action_t COvercomeObstacle::GetNativeActivity()const
{
    float current=Time();
    if(m_next_jump_time>current)
    {
        m_next_jump_time=current+GetRandomFloat(1.5f,3.0f);
        return m_data.type_move|IN_JUMP;
    }
    else return m_data.type_move;
}

Vector COvercomeObstacle::GetNativeView()const
{
    return VecToAngles(m_data.to_vec-Bot().Center());
}

bool COvercomeObstacle::IsNativeComplete()const
{
    return IsNear(Bot(),m_data.to_vec);
}

/// CBaseRoute
// base CTask performing movement from m_vec_from to m_vec_to
// along a set of given CMapGraph edges

CBaseRoute::CBaseRoute()
{
    SetJumpTable(&m_base_move,id_to_start_node_st,
            {{
                // move from the starting point to the first edge
                id_to_start_node_st,
                [this](CTask*task)
                 {return jump_from_start_node(task);}
            },
            {

                // move along the edge of the graph
                id_on_route_st,
                [this](CTask*task)
                {return jump_from_on_route(task);}
            },
            {
                // what to do if you get stuck somewhere
                id_in_obstacle_st,
                [this](CTask*task)
                {return jump_from_in_obstacle(task);}
            },
            {
                // move up a steep staircase
                id_staircase_st,
                [this](CTask*task)
                {return jump_from_on_route(task);}
            },
            {
                // actions for using the lift
                id_on_elevator_st,
                [this](CTask*task)
                {return jump_from_on_route(task);}
            },
            {
                // move to the end point
                id_to_end_point_st,
                [this](CTask*task)
                {return jump_from_end_point(task);}
            }});
    SetJumpAction([this](fsm_state_t from,fsm_state_t to)
                  {m_JumpMessage(from,to);});
    ForbidRedef(id_redef_move_t);
}

// CBaseRoute :: m_JumpMessage - sending a message to an abstract handler
// m_tracker when moving from one edge of the path to another

void CBaseRoute::m_JumpMessage(fsm_state_t from_st,fsm_state_t to_st)
{
    if(!m_tracker) return;
    if(to_st.second==id_on_route_st||
       to_st.second==id_staircase_st||
       to_st.second==id_on_elevator_st)
    {
        m_tracker->NewEdgeBeginning(m_current_edge);
    }
    else if(to_st.second==id_to_end_point_st)
    {
        m_tracker->ToEndBeginning(m_vec_to);
    }
    else{}
}

const Vector&CBaseRoute::Destination()const
{
    return m_vec_to;
}

void CBaseRoute::Set(const route_param_t& route_)
{
    m_vec_from=route_.from_vec;
    m_vec_to=route_.to_vec;
    assert(route_.path_gen!=nullptr);
    m_path_gen.reset(route_.path_gen);
    m_path_gen->begin();
    m_base_move.ResetGoal({m_vec_from,0,30.0f,60.0f});
    SetForced(&m_base_move,id_to_start_node_st);
    m_put_off_task=-1;
}

void CBaseRoute::SetTracker(CRouteTracker* tracker)
{
    m_tracker=tracker;
}

Vector CBaseRoute::DefObstacleData(const Vector&from,
                                  const Vector&to)
{
    if(distance_3D(from-to)<=100.0f) return to;
    return from+100.0f*normalize_3D(to-from);
}

std::pair<CTask*,int> CBaseRoute::update_route_task()
{

    if(!m_path_gen->empty())//following the edge of the path
    {
        m_current_edge=m_path_gen->get();
        m_path_gen->next();
        int type_move=m_current_edge.edge->type_moving();
        if(type_move&IN_STAIRS)
        {
            m_stairs_move.ResetGoal(m_current_edge);
            return {&m_stairs_move,id_staircase_st};
        }
        else if(type_move&IN_ELEVATOR)
        {
            m_elevator_move.reset(new CUseElevator(m_current_edge));
            return {m_elevator_move.get(),id_on_elevator_st};
        }
        else
        {
            m_base_move.ResetGoal({
                                m_current_edge.to_node->Coord(),
                                static_cast<action_t>(type_move),
                                20.0f,
                                20.0f});
            return {&m_base_move,id_on_route_st};
        }
    }
    else
    {
        m_base_move.ResetGoal({
                              m_vec_to,
                               0,
                               30.0f,
                               60.0f});

        return {&m_base_move,id_to_end_point_st};
    }
}

std::pair<CTask*,int> CBaseRoute::jump_to_obstacle_task(int prev_task)
{
    assert(m_put_off_task==-1);
    m_put_off_task=prev_task;
    m_put_off_data=m_base_move.Data();

    m_obstacle_move.
    ResetGoal({Bot().Center(),
               DefObstacleData(Bot().Center(),m_base_move.Data().point),
               m_base_move.Data().type});

    return {&m_obstacle_move,id_in_obstacle_st};
}


std::pair<CTask*,int> CBaseRoute::jump_from_start_node(CTask*task)
{
    if(m_base_move.IsComplete())
    {
        if(!m_path_gen->empty())
        {
             m_current_edge=m_path_gen->get();
             m_path_gen->next();
             m_base_move.ResetGoal({
                         m_current_edge.to_node->Coord(),
                         m_current_edge.edge->us_type_moving(),
                         30.0f,
                         30.0f});

            return {&m_base_move,id_on_route_st};
        }
        else
        {
            m_base_move.ResetGoal({
                         m_vec_to,
                         0,
                         30.0f,
                         60.0f});
            return {&m_base_move,id_to_end_point_st};
        }
    }

    auto med_vel=m_base_move.MedialVelocity();
    if(med_vel&&*med_vel<100.0f)
    {
        return jump_to_obstacle_task(id_to_start_node_st);
    }
    else
    {
        return {nullptr,id_to_start_node_st};
    }
}

std::pair<CTask*,int> CBaseRoute::jump_from_on_route(CTask*task)
{
    if(CurrentTask().IsComplete())
    {
        return update_route_task();
    }
    if(CurrentState()==id_staircase_st)
    {
        return {nullptr,id_staircase_st};
    }
    if(CurrentState()==id_on_elevator_st)
    {
        return {nullptr,id_on_elevator_st};
    }

    auto med_vel=m_base_move.MedialVelocity();
    if(med_vel&&*med_vel<100.0f)
        {return jump_to_obstacle_task(id_on_route_st);}

    else
    {
        return {nullptr,id_on_route_st};
    }
}


std::pair<CTask*,int> CBaseRoute::jump_from_in_obstacle(CTask*task)
{
    assert(m_put_off_task!=-1);

    if(m_obstacle_move.IsComplete())
    {
        int continue_task=m_put_off_task;
        m_put_off_task=-1;

        m_base_move.ResetGoal(m_put_off_data);

        return {&m_base_move,continue_task};
    }
    else
    {
        return {nullptr,id_in_obstacle_st};
    }
}

std::pair<CTask*,int> CBaseRoute::jump_from_end_point(CTask*task)
{
    auto med_vel=m_base_move.MedialVelocity();
    if(med_vel&&*med_vel<100.0f)
    {
        return jump_to_obstacle_task(id_to_end_point_st);
    }
    else
    {
        return {nullptr,id_to_end_point_st};
    }
}

void CBaseRoute::NativeRun(CBot&bot)
{
    CFSMTask::NativeRun(bot);
    if(m_tracker)
    {
        m_tracker->ToStartBeginning(m_vec_from);
    }
}

bool CBaseRoute::IsNativeComplete()const
{
    return CurrentState()==id_to_end_point_st&&
           m_base_move.IsComplete();
}

/// CRouteViewer
// way of viewing the terrain when executing CBaseRoute

CRouteViewer::CRouteViewer()
{
    SetJumpTable(&m_viewer,id_view_point,
                {{
                    // looking at the end point of the edge
                    // along which the bot is moving now
                    id_view_point,
                    [this](CTask*task)
                    {return jump_from_view_point(task);}
                },
                {
                    //look back
                    id_look_back,
                    [this](CTask*task)
                    {return jump_from_look_back(task);}
                },
                {
                    // completing the already inactive look
                    id_incomplete_view,
                    [this](CTask*task)
                    {return jump_from_incomplete_view(task);}
                },
                {
                    // look mapgraph.cpp
                    id_main_view,
                    [this](CTask*task)
                    {return jump_from_main_view(task);}
                }});
}

void CRouteViewer::ToStartBeginning(const Vector&beg_vec)
{
    m_point=beg_vec;
    m_update=id_new_point;
    m_current_edge.from_node=nullptr;
    m_current_edge.to_node=nullptr;
    m_main_view={};
}

void CRouteViewer::ToEndBeginning(const Vector&_vec)
{
    m_point=_vec;
    m_update=id_new_point;
    m_current_edge.from_node=nullptr;
    m_current_edge.to_node=nullptr;
    m_main_view={};
}

void CRouteViewer::NewEdgeBeginning(const edge_path_t&edge )
{
    m_current_edge=edge;
    m_update=id_new_edge;
    m_main_view=glMapGraph.VisibleData().GetViewing(m_current_edge);
}

CRouteViewer::update_t CRouteViewer::m_DispatchUpdate()
{
    update_t for_ret=m_update;
    m_update=id_no_update;
    return for_ret;
}

std::optional<CFSMTask::fsm_state_t> CRouteViewer::m_IsMainLook()
{
    if(m_main_view)
    {
        float dist=distance_3D(
                   Bot().Center()-m_current_edge.to_node->Coord());
        if(dist<=100.0f)
        {
            m_viewer.SetView({{*m_main_view,0.0f,CViewer::id_angle}});
            return {{&m_viewer,id_main_view}};
        }
    }
    return {};
}

std::optional<CFSMTask::fsm_state_t> CRouteViewer::m_IsLookBack()
{
    float current=Time();
    if(current>m_next_look_back&&m_current_edge.to_node!=nullptr)
    {
        m_next_look_back=current+GetRandomFloat(3.0f,6.0f);
        Vector look_back=VecToAngles(
                         m_current_edge.from_node->Coord()-
                         m_current_edge.to_node->Coord());

        m_viewer.SetView({{look_back,0.0f,CViewer::id_angle}});
        return {{&m_viewer,id_look_back}};
    }
    return {};
}

CFSMTask::fsm_state_t CRouteViewer::jump_from_view_point(CTask*task)
{
    update_t update=m_DispatchUpdate();
    if(update==id_new_point)
    {
        m_viewer.SetView({{m_point,0.0f,CViewer::id_point}});
        return {&m_viewer,id_view_point};
    }
    else if(update==id_new_edge)
    {
        m_viewer.SetView({{m_current_edge.to_node->Coord(),0.0f,CViewer::id_point}});
        return {&m_viewer,id_view_point};
    }
    else {}

    if(auto is_update=m_IsMainLook();is_update) {return *is_update;}

    if(auto is_update=m_IsLookBack();is_update) {return *is_update;}

    return {nullptr,id_view_point};
}

CFSMTask::fsm_state_t CRouteViewer::jump_from_look_back(CTask*task)
{
    update_t update=m_DispatchUpdate();
    if(update==id_new_point)
    {
        return {nullptr ,id_incomplete_view};
    }
    else if(update==id_new_edge)
    {
        return {nullptr,id_incomplete_view};
    }
    else {}

    if(auto is_update=m_IsMainLook();is_update) {return *is_update;}

    if(m_viewer.IsComplete())
    {
        assert(m_current_edge.to_node!=nullptr);
        m_viewer.SetView({{m_current_edge.to_node->Coord(),0.0f,CViewer::id_point}});
        return {&m_viewer,id_view_point};
    }
    return {nullptr, id_look_back};
}

CFSMTask::fsm_state_t CRouteViewer::jump_from_incomplete_view(CTask*task)
{
    update_t update=m_DispatchUpdate();
    if(update==id_new_point)
    {
        m_viewer.SetView({{m_point,0.0f,CViewer::id_point}});
        return {&m_viewer,id_view_point};
    }
    else if(update==id_new_edge)
    {
        m_viewer.SetView({{m_current_edge.to_node->Coord(),0.0f,CViewer::id_point}});
        return {&m_viewer,id_view_point};
    }
    else {}

    if(auto is_update=m_IsMainLook();is_update) {return *is_update;}

    if(m_viewer.IsComplete())
    {
        if(m_current_edge.to_node!=nullptr)
        {
            m_viewer.SetView({{m_current_edge.to_node->Coord(),0.0f,CViewer::id_point}});
            return {&m_viewer,id_view_point};
        }
        else
        {
            m_viewer.SetView({{m_point,0.0f,CViewer::id_point}});
            return {&m_viewer,id_view_point};
        }
    }
    return {nullptr,id_incomplete_view};
}

CFSMTask::fsm_state_t CRouteViewer::jump_from_main_view(CTask*task)
{
    update_t update=m_DispatchUpdate();
    if(update==id_new_point)
    {
        return {nullptr ,id_incomplete_view};
    }
    else if(update==id_new_edge)
    {
        return {nullptr ,id_incomplete_view};
    }
    else {}

    return {nullptr,id_main_view};
}

void CRouteViewer::Set( CBaseRoute& base_route)
{
    m_update=id_no_update;
    m_next_look_back=0.0f;
    m_current_edge.from_node=nullptr;
    m_current_edge.to_node=nullptr;
    m_main_view={};
    Close();
    assert(base_route.IsRun());
    int state=base_route.CurrentState();
    if(state==CBaseRoute::id_to_end_point_st)
    {
        m_point=base_route.m_vec_to;
        m_viewer.SetView({{m_point,0.0f,CViewer::id_point}});
        SetForced(&m_viewer,id_view_point);
    }
    else if(state==CBaseRoute::id_to_start_node_st)
    {
        m_point=base_route.m_vec_from;
        m_viewer.SetView({{m_point,0.0f,CViewer::id_point}});
        SetForced(&m_viewer,id_view_point);
    }
    else
    {
        m_current_edge=base_route.m_current_edge;
        m_main_view=glMapGraph.VisibleData().GetViewing(m_current_edge);
        m_viewer.SetView({{m_current_edge.to_node->Coord(),0.0f,CViewer::id_point}});
        SetForced(&m_viewer,id_view_point);
    }
    Run(base_route.Bot());
}

/// Getters

// move in a fixed direction and to a fixed point
move_getter_t fn_move(const Angles& _dir)
{
    return [dir=_dir.get()](const CTask&tsk){return dir;};
}

move_getter_t fn_move(const Point&_vec)
{
    return [vec=_vec.get()](const CTask&task)
    {return VecToAngles(vec-task.Bot().Center());};
}

// hold the given maximum distance to vec
move_getter_t fn_take_position(const Vector&vec,float hor_dis,float vert_dist)
{
    return [vec,hor_dis,vert_dist](const CTask&task)->move_t
    {
        if(IsNear(task.Bot(),vec,hor_dis,vert_dist))
        {
            return {};
        }
        else
        {
            return VecToAngles(vec-task.Bot().Center());
        }
    };
}

act_getter_t fn_action(action_t act)
{
    return [act](const CTask&tsk){return act;};
}


// looking in a fixed direction and at a fixed point
view_getter_t fn_viewing(const Angles& _dir)
{
    return [dir=_dir.get()](const CTask&task){return dir;};
}

view_getter_t fn_viewing(const Point&_vec)
{
    return [vec=_vec.get()](const CTask&task)
    {return VecToAngles(vec-task.Bot().Center());};
}

// browse function can be borrowed from another task
view_getter_t fn_viewing(CTask&view_task)
{
    return [&view_task](const CTask&exe_task)
    {
        return view_task.GetView();
    };
}

// the end of the task after a fixed time
end_getter_t fn_duration_terminator(float dur)
{
    return [dur](const CTask&task){return task.Duration()>dur;};
}


// end the task upon reaching the given point
end_getter_t fn_position_terminator(const Vector&dest,
                                    float hor_prec,float vert_prec)
{
    return [dest,hor_prec,vert_prec](const CTask&task)
    {return IsNear(task.Bot(),dest,hor_prec,vert_prec);};
}


// end the task when the given viewing angle is reached
// or direction of view
end_getter_t fn_view_terminator(const Angles& dir,
                                float prec)
{
    float min_cos=cos(prec);
    Vector dir_vector=AnglesToVec(dir.get());
    return [dir_vector,min_cos](const CTask&task)
    {
        return
        cos_angle_3D(dir_vector,AnglesToVec(task.Bot().ViewAngle()))>min_cos;
    };
}

end_getter_t fn_view_terminator(const Point&_vec,float prec)
{
    float min_cos=cos(prec);
    return [vec=_vec.get(),min_cos](const CTask&task)
    {
        Vector dir_vector=normalize_3D(AnglesToVec(vec-task.Bot().Eyes()));
        return
        cos_angle_3D(dir_vector,AnglesToVec(task.Bot().ViewAngle()))>min_cos;
    };
}

end_getter_t fn_act_item_terminator(int item)
{
    return [item](const CTask&task)
    {
        return task.Bot().ActiveItem()==item;
    };
}

act_getter_t fn_action(CTask& act_task)
{
    return [&act_task](const CTask&exe_task)
    {
        return act_task.GetActivity();
    };
}


/// CUseButton
// Basic function of using buttons

CUseButton::CUseButton():
m_use_button(this)
{
    SetChain({{&m_set_view,id_set_view_st},// direct bot eyes to the button
             { &m_use_button,id_use_button_st}});// IN_USE button

    // neither view nor movement can be overrided
    ForbidRedef(id_redef_view_t|
                id_redef_move_t);

    m_use_button.ForbidRedef(id_redef_act_t);
    m_use_button.SetActivity(fn_action(IN_USE));
}

const CItemPoint* CUseButton::ButtonPoint()const
{
    return m_item;
}

void CUseButton::SetItemPoint(const CItemPoint* item)
{
    m_item=item;
    assert(m_item!=nullptr);

    m_set_view.SetView(fn_viewing(Angles(m_item->ViewAngle())));
    m_set_view.SetTerminator(fn_view_terminator(Angles(m_item->ViewAngle())));
    m_set_view.SetMove(fn_take_position(m_item->Coord(),10.0f,10.0f));

    m_use_button.SetView(fn_viewing(Angles(m_item->ViewAngle())));
    m_use_button.SetMove(fn_take_position(m_item->Coord(),10.0f,10.0f));
}

void CUseButton::use_button_impl::Update()
{
    m_owner->UpdateUse();
}

void CUseButton::use_button_impl::NativeRun(CBot&bot)
{
    m_owner->RunUse(bot);
}

// termination condition is defined by derived classes
bool CUseButton::use_button_impl::IsNativeComplete()const
{
    return m_owner->IsCompleteUse();
}

///CUseHealthWall
void CUseHealthWall::SetHealthWall(const CItemPoint*wall)
{
    assert(wall->Type()==id_health_getter);
    SetItemPoint(wall);
}

void CUseHealthWall::UpdateUse()
{
    float current=Time();
    if(current<m_next_check) return;
    m_delta_health=Bot().Health()-m_last_health;
    m_next_check=current+m_period_check;
    m_last_health=Bot().Health();
}

void CUseHealthWall::RunUse(CBot&bot)
{
    m_last_health=bot.Health();
    m_next_check=Time()+m_period_check;
    m_delta_health={};
}

bool CUseHealthWall::IsCompleteUse()const
{
    if(Bot().Health()>99.0f||CurrentTask().Duration()>10.0f)
    {
        //ReportPrint
        //("Use HW complete 1:"+std::to_string(Bot().Health())+'\n');
        return true;
    }
    if(!m_delta_health) return false;

    return *m_delta_health==0;
}

///CUseArmourWall

void CUseArmourWall::SetArmourWall(const CItemPoint*wall)
{
    assert(wall->Type()==id_armour_getter);
    SetItemPoint(wall);
}

void CUseArmourWall::UpdateUse()
{
    float current=Time();
    if(current<m_next_check) return;
    m_delta_armour=Bot().Armour()-m_last_armour;
    m_next_check=current+m_period_check;
    m_last_armour=Bot().Armour();
}

void CUseArmourWall::RunUse(CBot&bot)
{
    m_last_armour=bot.Armour();
    m_next_check=Time()+m_period_check;
    m_delta_armour={};
}

bool CUseArmourWall::IsCompleteUse()const
{
    if(Bot().Armour()>99.0f||CurrentTask().Duration()>10.0f)
    {
        return true;
    }
    if(!m_delta_armour) return false;

    return *m_delta_armour==0;
}

///CUseElevator
// use the lift

CUseElevator::CUseElevator(const edge_path_t&elev):
m_elevator_edge(elev)
{
    SetItemPoint(elev.from_node->ItemPoint(id_elevator_button));
}

bool CUseElevator::IsCompleteUse()const
{
    return
    abs(Bot().Center()[2]-m_elevator_edge.to_node->Coord()[2])<100.0f;
}


// RouteToVector- route from_vec to to_vec
route_to_vector_param_t RouteToVector(const CBot&bot,const Vector&from_vec,const Vector&to_vec)
{
    const CWayPoint& from_node=glMapGraph.NearSuitableWP(from_vec,
                                                        bot.Player());

    const CWayPoint& to_node=glMapGraph.NearSuitableWP(to_vec,
                                                      bot.Player());

    CFixedPath* path_gen(new CFixedPath);
    bool res=glMapGraph.FindPath(from_node,to_node,path_gen->Data());

    assert(res);

    return route_to_vector_param_t{
        &from_node,
        &to_node,
        CBaseRoute::route_param_t(from_node.Coord(),path_gen,to_vec)
        };
}


// RouteToVector- route to a random node of the graph from the current location
route_to_vector_param_t RandomRoute(const CBot&bot)
{
    const CWayPoint& from_node=glMapGraph.NearSuitableWP(bot.Center(),
                                                        bot.Player());
    const CWayPoint& to_node=*glMapGraph.WayPoints()[GetRandomInt(0,glMapGraph.WayPoints().size()-1)];

    CFixedPath* path_gen(new CFixedPath);
    bool res=glMapGraph.FindPath(from_node,to_node,path_gen->Data());

    assert(res);

    return route_to_vector_param_t{
        &from_node,
        &to_node,
        CBaseRoute::route_param_t(from_node.Coord(),path_gen,to_node.Coord())
        };
}
