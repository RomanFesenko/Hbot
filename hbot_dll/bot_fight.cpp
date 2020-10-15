#include "bot_fight.h"
#include "items_data.h"

extern player_t* glMe;

// ItemPriority -определение ценности айтема по шкале 0...1

float ItemPriority(uint_t item,const CBot& bot)
{
    assert(bit_count(item)==1);
    uint_t weapon_ammo=0;
    if(item&Weapon)
    {
        if((item&AssaultWeapon)&&
           ((bot.Player().AllItems()&AssaultWeapon)==0))
        {
            return 0.8;
        }
        const item_properties_t& it_prop=ItemProperties(item);
        weapon_ammo=it_prop.ammo;
        if(!bot.Player().HasItem(item))
        {
            return it_prop.default_priority;
        }
        else
        {
            float decrease_coeff=
            static_cast<float>(it_prop.max_total-bot.Player().Ammo(weapon_ammo))/
            it_prop.max_total;
            if(decrease_coeff<=0.0) return 0.0;

            return it_prop.default_priority*decrease_coeff;
        }
    }
    else if(item&Ammunition)
    {
        const item_properties_t& it_prop=ItemProperties(item);
        uint_t ass_weap=0;
        if(item!=id_ammo_gaussclip)
        {
            ass_weap=ItemProperties(item).ammo;
            if(!bot.Player().HasItem(ass_weap))
                ass_weap=0;
        }
        else
        {
            if(bot.Player().HasItem(id_weapon_egon))
            {
                ass_weap=id_weapon_egon;
            }
            else if(bot.Player().HasItem(id_weapon_gauss))
            {
                ass_weap=id_weapon_gauss;
            }
            else{}
        }
        if(ass_weap)
        {
            float decrease_coeff=
            static_cast<float>(it_prop.max_total-bot.Player().Ammo(item))/
            it_prop.max_total;
            if(decrease_coeff<=0.0) return 0.0;
            return decrease_coeff*ItemProperties(ass_weap).default_priority;
        }
        else
        {
            return 0.2f;
        }
    }
    else if(item==id_health_getter)
    {
        return (100.0f-bot.Health())/70.0f;
    }
    else if(item==id_armour_getter)
    {
        return (100.0f-bot.Armour())/150.0f;
    }
    else
    {
        return 0.0f;
    }
}

///CEnemyData -
// сохранение и обновление информации о противнике
// Позволяет избавиться от многократных вызовов выч. затратной
// функции определения видимости

CEnemyData::CEnemyData(const CBot&owner,const player_t&enemy):
m_owner(owner),m_enemy(enemy)
{

}

void CEnemyData::Run()
{
    m_visible=0;
    m_last_time_vis=0.0f;
}

void CEnemyData::Update()
{
     int new_visible=0;
     if(Enemy().IsAlive())
     {
         if(m_owner.IsVisible(m_enemy.Center()))
         new_visible|=id_vis_body;

         if(m_owner.IsVisible(m_enemy.Eyes()))
         new_visible|=id_vis_head;
     }
     m_visible=new_visible;
     if(new_visible!=0)
     {
         m_last_time_vis=Time();
         m_last_enemy_data={Enemy().Center(),Enemy().ViewAngle()};
         m_last_owner_data={Owner().Center(),Owner().ViewAngle()};
     }
}

//DefaultEnemyCompare - функция сравнения противников по степени
// их опасности

bool DefaultEnemyCompare(const CEnemyData&fst,const CEnemyData&snd)
{
    assert(&fst.Owner()==&snd.Owner());
    if(fst.IsVisible()!=snd.IsVisible())
    {
        return snd.IsVisible();
    }
    return distance_3D(fst.Owner().Center()-fst.Enemy().Center())>
           distance_3D(snd.Owner().Center()-snd.Enemy().Center());
}


///CEnemyDataPool
// сохранение и обновление информации о противниках
// с определением наиболее опасного

CEnemyDataPool::CEnemyDataPool( const CBot&bot):
m_owner(bot)
{
    m_data_pool.reserve(glBots.m_max_bots);
}

void CEnemyDataPool::Run()
{
    for(auto &en_data:Enemies())
    {
        en_data->Run();
    }
}

void CEnemyDataPool::Update()
{
    for(auto& data:m_data_pool)
    {
        data->Update();
    }
    auto last_current=m_current;
    m_current=std::max_element(m_data_pool.begin(),
                               m_data_pool.end(),
    [this](const enemy_data_ptr&fst,const enemy_data_ptr&snd)
    {return m_comparer(*fst,*snd);});
    is_new_current=(last_current!=m_current)&&(NumVisible()>0);
}

void CEnemyDataPool::SetEnemiesComparer(enemy_compare_t comparer)
{
    m_comparer=comparer;
}

const CEnemyData& CEnemyDataPool::Current()const
{
    assert(!m_data_pool.empty());
    assert(m_current<m_data_pool.end());
    return **m_current;
}

CEnemyDataPool::size_type CEnemyDataPool::NumVisible()const

{
    return std::count_if(
           m_data_pool.begin(),
           m_data_pool.end(),
           [](const enemy_data_ptr& data){return data->IsVisible();});
}

bool CEnemyDataPool::IsEnemy(const player_t& enm)const
{
    return std::find_if(
           m_data_pool.begin(),
           m_data_pool.end(),
           [&enm](const enemy_data_ptr& data)
           {return &data->Enemy()==&enm;})!=m_data_pool.end();
}

void CEnemyDataPool::Add(const player_t& enm)
{
    assert(!IsEnemy(enm));

    m_data_pool.push_back(enemy_data_ptr(new CEnemyData(m_owner,enm)));
    Update();
}

void CEnemyDataPool::Erase(const player_t& enm)
{
    auto iter=std::find_if(
              m_data_pool.begin(),
              m_data_pool.end(),
              [&enm](enemy_data_ptr& data)
              {return &data->Enemy()==&enm;});

    if(iter==m_data_pool.end()) return;
    m_data_pool.erase(iter);
    m_current=m_data_pool.end();
    Update();
}

// DispatchNewCurrent - возвращает true если
// изменился наиболее опасный противник

bool CEnemyDataPool::DispatchNewCurrent()
{
    bool for_ret=is_new_current;
    is_new_current=false;
    return for_ret;
}

//PriorityViewPoint-куда нужно смотреть/стрелять

Vector PriorityViewPoint(const CEnemyDataPool&data)
{
    const CEnemyData& current=data.Current();
    const CBot& bot=data.Owner();
    int vis=current.Visible();
    Vector view_point;
    if(!vis)
    {
        view_point=current.LastEnemyData().position;
    }
    else if(vis&CEnemyData::id_vis_body)
    {
        view_point=current.Enemy().Center();
    }
    else if(vis&CEnemyData::id_vis_head)
    {
        view_point=current.Enemy().Eyes();
    }
    else {assert(false);}
    // от ракеты рпг можно увернуться поэтому целимся в ноги
    if(bot.ActiveItem()==id_weapon_rpg)
    {
        view_point-=Vector(0,0,35.0f);
    }
    return view_point;
}

// LookBackViewing - возвращает куда и как смотреть если
// нанесено повреждение а врага не видно

CViewer::view_task_t LookBackViewing(const CDamageRegister& dreg)
{
    Vector dir=-AnglesToVec(dreg.ViewAngleDamage());
    return {VecToAngles(dir),1.0f,CViewer::id_angle};
}

view_getter_t fn_shooting_viewing(const CEnemyDataPool&data)
{
    return [&data](const CTask&task)
    {
        return VecToAngles(PriorityViewPoint(data)-task.Bot().Eyes());
    };
}

// fn_shooting_action -нужно ли стрелять/перезаряжать
// в зависимости от состояния оружия и точности наведения

act_getter_t fn_shooting_action(const CEnemyDataPool&data)
{
    return [&data](const CTask&task)->action_t
    {
        const CBot&bot=task.Bot();
        uint_t cur_item=bot.Player().ActiveItem();
        if(!(cur_item&Weapon)) return 0;
        if(cur_item==id_weapon_gauss)
        {
            if(bot.Player().Ammo(id_ammo_gaussclip)<=1)
            {
                SelectWeapon(bot);
                return 0;
            }
        }
        else if(cur_item==id_weapon_egon)
        {
            if(bot.Player().Ammo(id_ammo_gaussclip)==0)
            {
                SelectWeapon(bot);
                return 0;
            }
        }
        else if(bot.Player().InClip()==0)
        {
            uint_t ammo_item=ItemProperties(cur_item).ammo;
            assert(ammo_item&Ammunition);
            if((cur_item&Rechargeable)&&
               bot.Player().Ammo(ammo_item)>0)
            {
                return IN_RELOAD;
            }
            else
            {
                SelectWeapon(bot);
                return 0;
            }
        }
        else{}
        const CEnemyData& current=data.Current();
        float dist_to_enemy=distance_3D(current.LastEnemyData().position-
                                        bot.Center());
        if(!current.Visible()) return 0;
        const item_properties_t&item_prop=ItemProperties(cur_item);
        float current_dot=dot_product_3D(
                      normalize_3D(PriorityViewPoint(data)-bot.Eyes()),
                      AnglesToVec(bot.Player().ViewAngle()));


        float sin_disp=sqrt(1-current_dot*current_dot);
        float max_sin_disp=task.Bot().Chars().target_disperse*item_prop.target_rad/
        sqrt(item_prop.target_rad*item_prop.target_rad+dist_to_enemy*dist_to_enemy);

        return (sin_disp<max_sin_disp&&current_dot>0.0)? IN_ATTACK:0;
    };
}

///CDamageRegister
// Регистрирует повреждение, исполняемая тема может отреагировать

CDamageRegister::CDamageRegister(const CEnemyDataPool& data):
m_enemies(data)
{

}

void CDamageRegister::Run()
{
    m_last_health=m_enemies.Owner().Health();
    m_last_armour=m_enemies.Owner().Armour();
    m_alarm=false;
}

void CDamageRegister::Update()
{
    if(m_enemies.Owner().Health()<m_last_health||
       m_enemies.Owner().Armour()<m_last_armour)
    {
        m_time_damage=Time();
        m_view_angle=m_enemies.Owner().ViewAngle();
        m_alarm=true;
    }
    m_last_health=m_enemies.Owner().Health();
    m_last_armour=m_enemies.Owner().Armour();
}

///CExtUseWall
// Расширение CUseWall с реакцией на нанесенное повреждение

CExtUseWall::CExtUseWall(CDamageRegister&d_reg):
m_damage_reg(d_reg)
{
    SetJumpTable(nullptr,id_use_wall_st,
        {{
            id_use_wall_st,
            [this](CTask*task){return j_use_wall(task);}
        },
        {
            id_view_st,
            [this](CTask*task){return j_viewer(task);}
        }});
    m_viewer.ForbidRedef(CTask::id_redef_view_t|
                         CTask::id_redef_move_t);
}

void CExtUseWall::SetWall(const CItemPoint*wall)
{
    if(wall->Type()==id_health_getter)
    {
        m_use_health.SetHealthWall(wall);
        SetForced(&m_use_health,id_use_wall_st);
    }
    else if(wall->Type()==id_armour_getter)
    {
        m_use_armour.SetArmourWall(wall);
        SetForced(&m_use_armour,id_use_wall_st);
    }
    else
    {
        assert(false);
    }
    m_wall=wall;
    Vector dir_to_wall=AnglesToVec(m_wall->ViewAngle());
    Vector ang_1=VecToAngles(turn_around_axis(dir_to_wall,glOrtZ,120*Degree));
    Vector ang_2=VecToAngles(turn_around_axis(dir_to_wall,glOrtZ,-120*Degree));

    m_viewer.SetView({
    {ang_1,0.0,CViewer::id_angle},
    {ang_2,1.0,CViewer::id_angle}});
}

bool CExtUseWall::IsNativeComplete()const
{
    return CurrentState()==id_use_wall_st&&
           CurrentTask().IsComplete();
}

CFSMTask::fsm_state_t CExtUseWall::j_use_wall(CTask*task)
{
    if(m_damage_reg.IsDamage())
    {
        return {&m_viewer,id_view_st};
    }
    return {nullptr,id_use_wall_st};
}

//CExtUseWall::j_viewer - оглянуться по сторонам, если был
// нанесен ущерб

CFSMTask::fsm_state_t CExtUseWall::j_viewer(CTask*task)
{
    if(task->IsComplete())
    {
        m_damage_reg.UndoAlarm();
        return (m_wall->Type()==id_health_getter)?
                CFSMTask::fsm_state_t{&m_use_health,id_use_wall_st}:
                CFSMTask::fsm_state_t{&m_use_armour,id_use_wall_st};
    }
    return {nullptr,id_view_st};
}

const CItemPoint* CExtUseWall::Wall()const
{
    return m_wall;
}

///CVisitedItemsPoints
// регистрирует недавно посещенные  CItemPoint
// чтобы бот не возвращался в пустое место

void CVisitedItemsPoints::RegVisit(const CItemPoint*point)
{
    assert(point!=nullptr);
    float current=Time();
    if(!m_visited_items.empty())
    {
        assert(current>=m_visited_items.front().second);
    }
    m_visited_items.push_front({point,current});
    if(m_visited_items.size()>m_max_size)
    {
        m_visited_items.pop_back();
    }
    while(!m_visited_items.empty())
    {
        if(m_visited_items.back().second+m_max_memoized_time<current)
        {
            m_visited_items.pop_back();
        }
        else
        {
            return;
        }
    }
}

std::optional<float> CVisitedItemsPoints::TimeLastVisit(const CItemPoint* point)const
{
    for(const auto&record:m_visited_items)
    {
        if(record.first==point) return record.second;
    }
    return {};
}

///RangeNeededItems
//array[0]- наиболее ценные айтемы,array[2] - наименее

std::tuple<uint_t,uint_t,uint_t> RangeNeededItems(const CBot&bot)
{
    uint_t all_needed=      Weapon|
                        Ammunition|
                  id_health_getter|
                  id_armour_getter;

    uint_t high_needed{0},medial_needed{0},low_needeed{0};
    auto set_item=[&high_needed,&medial_needed,&low_needeed]
    (uint_t item,float prior)
    {
        if(prior>=0.67)
        {
            high_needed|=item;
        }
        else if(prior>=0.34)
        {
            medial_needed|=item;
        }
        else if(prior>=0.01)
        {
            low_needeed|=item;
        }
        else{}
    };

    generate_flags gen(all_needed);
    for(gen.begin();!gen.end();gen.next())
    {
        uint_t current=gen;
        set_item(current,ItemPriority(current,bot));
    }
    return {high_needed,medial_needed,low_needeed};
}

///CPriorityItemSearch
// Визитор в алг.Дейкстры, останавливающий алгоритм
// если найден айтем с высшим приоритетом,
// и регистрируюший пару ближайших нодов
// с низшим приоритетом

CPriorityItemSearch::CPriorityItemSearch(uint_t high,uint_t medial,uint_t low,
                                         const CVisitedItemsPoints& visited):
m_visited(visited)
{
    if(high)
    {
        m_finded.push_back({nullptr,high});
    }
    if(medial)
    {
        m_finded.push_back({nullptr,medial});
    }
    if(low)
    {
        m_finded.push_back({nullptr,low});
    }
    assert(!m_finded.empty());
}


bool CPriorityItemSearch::find_most_wanted(const CWayPoint& wpoint)
{
    for(auto&[cur_item,flag]:m_finded)
    {
        if(cur_item!=nullptr) break;
        if(wpoint.ItemsFlag()&flag)
        {
            const CItemPoint* item=wpoint.ItemPoint(flag);
            assert(item!=nullptr);
            auto rec_visit=m_visited.TimeLastVisit(item);
            if((!rec_visit)||Time()>*rec_visit+
               ItemProperties(item->Type()).storage_restore_time)
            {
                cur_item=item;
                break;
            }
        }
    }
    return m_finded.front().first!=nullptr;
}

const CItemPoint* CPriorityItemSearch::SuitableItem()const
{
    auto iter=find_if(m_finded.begin(),
                      m_finded.end(),
                      [](auto pair){return pair.first!=nullptr;});

    return (iter!=m_finded.end())? iter->first:nullptr;
}

//RouteToPriorityItems - возврашает параметры пути
// и CItemPoint к ближайшему ценному айтему

std::pair<CBaseRoute::route_param_t,const CItemPoint*>
RouteToPriorityItems(const CBot&bot,
                     const CVisitedItemsPoints&visited)
{
    const CWayPoint&near=glMapGraph.NearSuitableWP(bot.Center(),
                                                  bot.Player());
    auto [high_,medial_,low_]=RangeNeededItems(bot);
    if((high_|medial_|low_)==0)
    {
        if(BOT_DEBUG)
        {
            ReportPrint("Never wanted \n");
        }
        return {CBaseRoute::route_param_t(),nullptr};
    }
    CPriorityItemSearch finder(high_,medial_,low_,visited);
    CFixedPath* path_gen=new CFixedPath;
    bool find_high_prior=glMapGraph.
                         RestrictedFindPath(near,path_gen->Data(),
                                            finder.GetWrapper());

    const CItemPoint* finded_item=finder.SuitableItem();
    if(finded_item==nullptr)
    {
        delete path_gen;
        if constexpr(BOT_DEBUG)
        {
            ReportPrint("Never find \n");
        }
        return {CBaseRoute::route_param_t(),nullptr};
    }
    if(!find_high_prior)
    {
        glMapGraph.GetPath(near,finded_item->NodeOwner(),path_gen->Data());
    }

    //ReportPrint("ROUTE GOAL: "+get_string(finded_item->Type())+'\n');

    return {CBaseRoute::route_param_t(near.Coord(),
                                      path_gen,
                                      finded_item->Coord()),
            finded_item};
}

//CDeathActivity - кликать лкм для респауна

action_t CDeathActivity::operator()(const CTask&task)
{
    action_t action=m_action;
    m_action=(m_action==0)? IN_ATTACK:0;
    return action;
}

///------------------------------------------------
///               Fighting
///------------------------------------------------

///CTotter
// шатание из стороны в сторону не давая прицелиться

CTotter::CTotter(const CEnemyDataPool&data):
m_enemies(data)
{
    SetView(fn_shooting_viewing(data));
    SetActivity(fn_shooting_action(data),CTask::id_added_act_t);
}

void CTotter::NativeRun(CBot& bot)
{
    float current=Time();
    m_move_right=GetRandomInt(0,1);
    m_next_change_move_time=current+GetRandomFloat(0.1,m_params.max_move_time);
    m_next_jump_time=current+GetRandomFloat(1,m_params.max_jump_time);
}

move_t CTotter::GetNativeMove()const
{
    Vector move_dir=cross_product_3D(
                    PriorityViewPoint(m_enemies)-Bot().Center(),glOrtZ);
    if(!m_move_right) move_dir*=(-1.0f);
    if(distance_3D(move_dir)<1.0f) return {};
    return VecToAngles(move_dir);
}

void CTotter::Update()
{
    float current=Time();
    if(current>m_next_change_move_time)
    {
        m_move_right=!m_move_right;
        m_next_change_move_time=current+GetRandomFloat(0.1,m_params.max_move_time);
    }
}

action_t CTotter::GetNativeActivity()const
{
    float current=Time();
    if(current>m_next_jump_time)
    {
        m_next_jump_time=current+GetRandomFloat(1,m_params.max_jump_time);
        return IN_JUMP;
    }
    return 0;
}

void CTotter::SetParams(const param_t&param)
{
    m_params=param;
}


/// CRoute
// Расширение CBaseRoute включающее
// наиболее информативное оглядывание по сторонам
// и предоставляющее простой интерфейс для CMainTask

CRoute::CRoute(CMainTask&main_):
m_visited_points(main_.m_visited_points)
{
    m_base_route.SetTracker(&m_route_viewer);
    m_base_route.SetView(fn_viewing(m_route_viewer));
}

void CRoute::Update()
{
    m_base_route.Update();
    m_route_viewer.Update();
}

void CRoute::NativeRun(CBot&bot)
{
    m_base_route.Run(bot);
    if(m_native_viewing)
    {
        m_route_viewer.Set(m_base_route);
    }
}

void CRoute::NativeClose()
{
    if(m_native_viewing)
    {
        m_route_viewer.Close();
    }
    m_base_route.Close();
}

move_t CRoute::GetNativeMove()const
{
    return m_base_route.GetMove();
}

action_t CRoute::GetNativeActivity()const
{
    return m_base_route.GetActivity();
}

Vector CRoute::GetNativeView()const
{
    return m_base_route.GetView();
}

bool CRoute::IsNativeComplete()const
{
    return m_base_route.IsComplete();
}

const CWayPoint& CRoute::CurrentLastNode()const
{
    assert(m_last_node!=nullptr);
    return *m_last_node;
}

///CRoute::SetToPriority
// установить маршрут к самому нужному айтему
// или случайный маршрут если ничего не надо или ничего нет

bool CRoute::SetToPriority()
{
    auto route_param=RouteToPriorityItems(Bot(),m_visited_points);
    m_goal_item=route_param.second;
    if(route_param.second!=nullptr)
    {
        m_last_node=&route_param.second->NodeOwner();
        m_base_route.Set(route_param.first);
        /* ReportPrint("Create route: \nItem: "+
                        get_string(m_goal_item->Type())+
                        "\nVector: "+ToString(m_goal_item->Coord())+'\n');*/
    }
    else
    {
        auto random_route_param=RandomRoute(Bot());
        m_last_node=random_route_param.to_node;
        m_base_route.Set(random_route_param);
        if constexpr(BOT_DEBUG)
        {
            ReportPrint("Set random route\n" );
        }
    }
    return  route_param.second!=nullptr;
}

/*CRoute::UpdateCurrent - если текущий нод назаначения
 не изменился,но бот сбился с кратчайшего пути -
 выполняется поиск всех кратчайших путей в целевой нод:
 поиск всех кратчайших путей ИЗ целевого нода в обращенном графе
 находит все кратчайшие пути В целевой нод в исходном графе
 и устанавливаетя новый кратч. путь
*/
bool CRoute::UpdateCurrent(fn_b_wpref_t node_filter,
                           edge_filter_t edge_filter)
{
    using path_generator_t= CSearchInTransposed::path_generator_t;
    assert(m_last_node!=nullptr);
    Vector vec_dest=m_base_route.Destination();
    if(m_tr_search.Destination()!=m_last_node)
    {
        m_tr_search.RestrictedFindPathes(*m_last_node,
                                         node_filter,
                                         edge_filter);
    }
    const CWayPoint&near=glMapGraph.NearSuitableWP(Bot().Center(),
                                                  Bot().Player());

    path_generator_t*path_gen=new path_generator_t(m_tr_search);
    path_gen->UpdateDestination();
    bool result=path_gen->SetBeginning(near);
    if(result)
    {
        m_base_route.Set({near.Coord(),path_gen,vec_dest});
    }
    return result;
}

// CRoute::SetToVector - путь в конкретную точку

void CRoute::SetToVector(const Vector&vec_dest)
{
    auto param=RouteToVector(Bot(),Bot().Center(),vec_dest);
    m_last_node=param.to_node;
    m_base_route.Set(param);
    m_goal_item=nullptr;
}

const CItemPoint* CRoute::CurrentGoal()const
{
    return m_goal_item;
}

void CRoute::SetView(view_getter_t view_getter)
{
    if(view_getter==nullptr)
    {
        if(m_native_viewing) return;
        m_native_viewing=true;
        m_base_route.SetView(fn_viewing(m_route_viewer));
        if(m_base_route.IsRun())
        {
            m_route_viewer.Set(m_base_route);
        }
    }
    else
    {
        if(m_native_viewing)
        {
            m_route_viewer.Close();
            m_native_viewing=false;
        }
        m_base_route.SetView(view_getter);
    }
}

const Vector&CRoute::Destination()const
{
    return m_base_route.Destination();
}

///_______________________________________________
///                       SUBTASKS                |
///_______________________________________________|

///CPursuit
// Преследование противника

CPursuit::CPursuit(CMainTask&main_task):
m_enemies(main_task.m_enemies),
m_damage_register(main_task.m_damage_register),
m_route(main_task.m_route),
m_totter(main_task.m_enemies)
{
    SetJumpTable(&m_route,id_enemy_vis_st,
                {{
                    id_enemy_vis_st,// противник виден
                    [this](CTask*task){return j_from_vis_enemy(task);}
                },
                {
                    id_enemy_hide_st, // противник скрылся
                    [this](CTask*task){return j_from_hide_enemy(task);}
                },
                {
                    id_check_damage, // нанесен ущерб, но враг не виден
                    [this](CTask*task){return j_from_check_damage(task);}
                },
                {
                    id_totter_st, // предельное сближение с видимым прот.
                    [this](CTask*task){return j_from_totter(task);}
                }});
}

void CPursuit::NativeRun(CBot&bot)
{
    m_route.SetView(fn_shooting_viewing(m_enemies));
    m_route.SetActivity(fn_shooting_action(m_enemies),CTask::id_added_act_t);
    m_route.SetToVector(m_enemies.Current().LastEnemyData().position);
    CFSMTask::NativeRun(bot);
}

void CPursuit::NativeClose()
{
    m_route.SetView(nullptr);
    m_route.SetActivity(nullptr);
    CFSMTask::NativeClose();
}

bool CPursuit::IsNativeComplete()const
{
    return m_enemies.Current().Enemy().IsDead()||
           (CurrentState()==id_enemy_hide_st&&m_route.IsComplete());
}

// CPursuit::m_UpdateRoute- обновить марщрут преследования
// если расстояние между текущим положением врага
// и целевой точкой больше m_max_for_update

bool CPursuit::m_UpdateRoute()
{
    bool need_update_route=false;
    float delta= distance_3D(
                 m_route.Destination()-
                 m_enemies.Current().LastEnemyData().position);
    if(delta>m_max_for_update)
    {
        m_route.SetToVector(m_enemies.Current().LastEnemyData().position);
        need_update_route=true;
    }
    return need_update_route;
}

//CPursuit::j_from_vis_enemy - если противник виден
// атакуем до тех пор пока расстояние не сократится до  m_min_needed_dist

CFSMTask::fsm_state_t CPursuit::j_from_vis_enemy(CTask*task)
{
    bool is_update=m_UpdateRoute();
    if(m_enemies.NumVisible()==0)
    {
        m_damage_register.UndoAlarm();
        return (is_update)?
               CFSMTask::fsm_state_t{&m_route,id_enemy_hide_st}:
               CFSMTask::fsm_state_t{nullptr,id_enemy_hide_st};
    }
    float delta=distance_3D(
                Bot().Center()-
                m_enemies.Current().Enemy().Center());
    if(delta<m_min_needed_dist)
    {
        return {&m_totter,id_totter_st};
    }

    return (is_update)?
            CFSMTask::fsm_state_t{&m_route,id_enemy_vis_st}:
            CFSMTask::fsm_state_t{nullptr,id_enemy_vis_st};
}

//CPursuit::j_from_hide_enemy - если противник скрылся
// бежим к точке где он был виден последний раз
// также нужно реагировать на нанесенный ущерб

CFSMTask::fsm_state_t CPursuit::j_from_hide_enemy(CTask*task)
{
    if(m_enemies.NumVisible()!=0)
    {
        bool is_update=m_UpdateRoute();
        return (is_update)?
                CFSMTask::fsm_state_t{&m_route,id_enemy_vis_st}:
                CFSMTask::fsm_state_t{nullptr, id_enemy_vis_st};
    }
    if(m_damage_register.IsDamage())
    {
        m_look_back.SetView({LookBackViewing(m_damage_register)});
        m_look_back.Run(Bot());
        m_route.SetView(fn_viewing(m_look_back));
        return {nullptr,id_check_damage};
    }
    return {nullptr,id_enemy_hide_st};
}

//CPursuit::j_from_check_damage - оглянуться если был ущерб
// продолжая преследование

CFSMTask::fsm_state_t CPursuit::j_from_check_damage(CTask*task)
{
    if(m_enemies.NumVisible()!=0)
    {
        bool is_update=m_UpdateRoute();
        m_look_back.Close();
        m_route.SetView(fn_shooting_viewing(m_enemies));
        m_damage_register.UndoAlarm();
        return (is_update)?
                CFSMTask::fsm_state_t{&m_route,id_enemy_vis_st}:
                CFSMTask::fsm_state_t{nullptr, id_enemy_vis_st};
    }
    if(m_look_back.IsComplete())
    {
        m_damage_register.UndoAlarm();
        m_look_back.Close();
        m_route.SetView(fn_shooting_viewing(m_enemies));
        return {nullptr,id_enemy_hide_st};
    }
    return {nullptr,id_check_damage};
}

//CPursuit::j_from_totter

CFSMTask::fsm_state_t CPursuit::j_from_totter(CTask*task)
{
    if(m_enemies.NumVisible()==0)
    {
        m_damage_register.UndoAlarm();
        m_UpdateRoute();
        return {&m_route,id_enemy_hide_st};
    }
    float delta= distance_3D(
                 Bot().Center()-
                 m_enemies.Current().Enemy().Center());
    if(delta>m_max_needed_dist)
    {
        m_UpdateRoute();
        return {&m_route,id_enemy_vis_st};
    }
    return {nullptr,id_totter_st};
}

///CFightRoute
// Расширение CRoute к цели включающий
// перестрелки с видимым противником и реакцию на ущерб

CFightRoute::CFightRoute(CMainTask& main_task):
m_enemies(main_task.m_enemies),
m_damage_register(main_task.m_damage_register),
m_route(main_task.m_route),
m_totter(main_task.m_enemies)
{
    SetJumpTable(&m_totter,id_totter_st,
                {{
                    id_route_st,
                    [this](CTask*task){return j_from_route(task);}
                },
                {
                    id_check_damage,
                    [this](CTask*task){return j_from_check_damage(task);}
                },
                {
                    id_totter_st,
                    [this](CTask*task){return j_from_totter(task);}
                }});
}

void CFightRoute::NativeRun(CBot&bot)
{
    m_route.SetView(fn_shooting_viewing(m_enemies));
    m_route.SetActivity(fn_shooting_action(m_enemies),CTask::id_added_act_t);
    CFSMTask::NativeRun(bot);
}

void CFightRoute::NativeClose()
{
    m_route.SetView(nullptr);
    m_route.SetActivity(nullptr);
    CFSMTask::NativeClose();
}

CFSMTask::fsm_state_t CFightRoute::j_from_route(CTask*task)
{
    if(m_enemies.NumVisible()!=0)
    {
        return {&m_totter,id_totter_st};
    }
    if(m_damage_register.IsDamage())
    {
        m_look_back.SetView({LookBackViewing(m_damage_register)});
        m_look_back.Run(Bot());
        m_route.SetView(fn_viewing(m_look_back));
        return {nullptr,id_check_damage};
    }
    return {nullptr,id_route_st};
}

CFSMTask::fsm_state_t CFightRoute::j_from_totter(CTask*task)
{
    if(m_enemies.NumVisible()==0)
    {
        m_damage_register.UndoAlarm();
        m_route.UpdateCurrent();
        return {&m_route,id_route_st};
    }
    return {nullptr,id_totter_st};
}

CFSMTask::fsm_state_t CFightRoute::j_from_check_damage(CTask*task)
{
    if(m_enemies.NumVisible()!=0)
    {
        m_look_back.Close();
        return {&m_totter,id_totter_st};
    }
    if(m_look_back.IsComplete())
    {
        m_look_back.Close();
        m_route.SetView(fn_shooting_viewing(m_enemies));
        return {nullptr,id_route_st};
    }
    return {nullptr,id_check_damage};
}

///CMainRoute

CMainRoute::CMainRoute(CMainTask& main_task):
m_enemies(main_task.m_enemies),
m_damage_register(main_task.m_damage_register),
m_route(main_task.m_route),
m_totter(main_task.m_enemies)
{
    SetJumpTable(&m_route,id_route_st,
                {{
                    id_route_st,
                    [this](CTask*task){return j_from_route(task);}
                },
                {
                    id_enemy_vis_st,
                    [this](CTask*task){return j_from_enemy_vis(task);}
                },
                {
                    id_check_damage,
                    [this](CTask*task){return j_from_check_damage(task);}
                },
                {
                    id_totter_st,
                    [this](CTask*task){return j_from_totter(task);}
                }});
}

void CMainRoute::NativeRun(CBot&bot)
{
    m_route.SetView(m_route_viewing);
    CFSMTask::NativeRun(bot);
}

void CMainRoute::SetRouteViewing(view_getter_t view_get)
{
    m_route_viewing=view_get;
    if(m_route.IsRun())
    {
        m_route.SetView(m_route_viewing);
    }
}

CFSMTask::fsm_state_t CMainRoute::j_from_route(CTask*task)
{
    if(m_enemies.NumVisible()!=0)
    {
        m_route.SetView(fn_shooting_viewing(m_enemies));
        m_route.SetActivity(fn_shooting_action(m_enemies),CTask::id_added_act_t);
        return {nullptr,id_enemy_vis_st};
    }
    if(m_damage_register.IsDamage())
    {
        m_look_back.SetView({LookBackViewing(m_damage_register)});
        m_look_back.Run(Bot());
        m_route.SetView(fn_viewing(m_look_back));
        return {nullptr,id_check_damage};
    }
    return {nullptr,id_route_st};
}

CFSMTask::fsm_state_t CMainRoute::j_from_enemy_vis(CTask*task)
{
    if(m_enemies.NumVisible()==0)
    {
        m_damage_register.UndoAlarm();
        m_route.SetView(m_route_viewing);
        m_route.SetActivity(nullptr);
        return {nullptr,id_route_st};
    }
    float delta=distance_3D(
                Bot().Center()-m_enemies.Current().Enemy().Center());

    if(delta<m_min_dist_for_totter-50.0f)
    {
        return {&m_totter,id_totter_st};
    }
    return {nullptr,id_enemy_vis_st};
}

CFSMTask::fsm_state_t CMainRoute::j_from_check_damage(CTask*task)
{
    if(m_enemies.NumVisible()!=0)
    {
        m_look_back.Close();
        m_route.SetView(fn_shooting_viewing(m_enemies));
        m_route.SetActivity(fn_shooting_action(m_enemies),CTask::id_added_act_t);
        return {nullptr,id_enemy_vis_st};
    }
    if(m_look_back.IsComplete())
    {
        m_look_back.Close();
        m_damage_register.UndoAlarm();
        m_route.SetView(m_route_viewing);
        return {nullptr,id_route_st};
    }
    return {nullptr,id_check_damage};
}

CFSMTask::fsm_state_t CMainRoute::j_from_totter(CTask*task)
{
    if(m_enemies.NumVisible()==0)
    {
        m_route.UpdateCurrent();
        m_damage_register.UndoAlarm();
        m_route.SetView(m_route_viewing);
        m_route.SetActivity(nullptr);
        return {&m_route,id_route_st};
    }
    float delta=distance_3D(
                Bot().Center()-m_enemies.Current().Enemy().Center());
    if(delta>m_min_dist_for_totter+80.0f)
    {
        m_route.UpdateCurrent();
        return {&m_route,id_enemy_vis_st};
    }
    return {nullptr,id_totter_st};
}

///_______________________________________________
///                       CMainTask               |
///_______________________________________________|

void CMainTask::NativeRun(CBot&bot)
{
    m_enemies.Run();
    m_damage_register.Run();
    m_route.SetBot(bot);
    m_main->Run(bot);
}

void CMainTask::Update()
{
    m_enemies.Update();
    m_damage_register.Update();
    m_main->Update();
}

move_t CMainTask::GetNativeMove()const
{
    return m_main->GetMove();
}

action_t CMainTask::GetNativeActivity()const
{
    return m_main->GetActivity();
}

Vector CMainTask::GetNativeView()const
{
    return m_main->GetView();
}

CMainTask::CMainTask(CBot&bot):
m_enemies(bot),
m_damage_register(m_enemies),
m_route(*this),
m_calm_route(*this),
m_fight_route(*this),
m_pursuit(*this),
m_use_wall(m_damage_register)
{
    m_spawn_task.SetActivity(CDeathActivity());

    m_main.reset(new CFSMTask);
    m_main->SetJumpTable(
            &m_spawn_task,id_spawn_st,
            {{
                id_spawn_st,
                [this](CTask*task){return j_from_spawn(task);}
            },
            {
                id_calm_route_st,
                [this](CTask*task){return j_from_calm_route(task);}
            },
            {
                id_fight_route_st,
                [this](CTask*task){return j_from_fight_route(task);}
            },
            {
                id_pursuit_st,
                [this](CTask*task){return j_from_pursuit(task);}
            },
            {
                id_go_away,
                [this](CTask*task){return j_from_go_away(task);}
            },
            {
                id_use_wall_st,
                [this](CTask*task){return j_from_use_wall(task);}
            }});

    auto j_act_=[this](CFSMTask::fsm_state_t from,CFSMTask::fsm_state_t to_)
    {
        jump_action(from,to_);
    };
    m_main->SetJumpAction(j_act_);
}

//CMainTask::j_from_spawn - респаун

CFSMTask::fsm_state_t CMainTask::j_from_spawn(CTask* current_task)
{
    if(Bot().IsDead()) return {nullptr,id_spawn_st};

    m_damage_register.UndoAlarm();
    m_damage_register.Run();
    m_route.SetToPriority();
    return {&m_calm_route,id_calm_route_st};
}

//CMainTask::j_from_calm_route

CFSMTask::fsm_state_t CMainTask::j_from_calm_route(CTask*task)
{
    if(Bot().IsDead()){
        return {&m_spawn_task,id_spawn_st};
    }

    // обнаружен враг: определяем соотношение сил
    // и выбираем тактику боя
    if(m_enemies.NumVisible())
    {
        float balance=DefBattleBalanse(
                      Bot(),m_enemies.Current().Enemy());
        // вступаем в перестрелки
        // если противник виден
        if((balance>-0.5)&&(balance<0.5))
        {
            return {&m_fight_route,id_fight_route_st};
        }
        else if(balance>=0.5)// догнать и уничтожить
        {
            return {&m_pursuit,id_pursuit_st};
        }
        // возможно скорее покидаем поле боя
        else
        {
            m_calm_route.SetRouteViewing(fn_shooting_viewing(m_enemies));
            return {nullptr,id_go_away};
        }
    }
    if(!m_route.IsComplete())
    {
        return {nullptr,id_calm_route_st};
    }
    else
    {
        const CItemPoint* goal=m_route.CurrentGoal();
        if(goal==nullptr)
        {
            m_route.SetToPriority();
            return {&m_calm_route,id_calm_route_st};
        }
        if(goal->Type()==id_health_getter||
           goal->Type()==id_armour_getter)
        {
            m_use_wall.SetWall(goal);
            return {&m_use_wall,id_use_wall_st};
        }
        else
        {
            m_visited_points.RegVisit(goal);
            m_route.SetToPriority();
            return {&m_calm_route,id_calm_route_st};
        }
    }
}

// CMainTask::j_from_use_wall - использование "банкоматов"

CFSMTask::fsm_state_t CMainTask::j_from_use_wall(CTask*task)
{
    if(Bot().IsDead()){
        return {&m_spawn_task,id_spawn_st};
    }
    if(m_enemies.NumVisible())
    {
        return {&m_pursuit,id_pursuit_st};
    }
    if(task->IsComplete())
    {
        m_visited_points.RegVisit(m_use_wall.Wall());
        m_route.SetToPriority();
        return {&m_calm_route,id_calm_route_st};
    }
    return {nullptr,id_use_wall_st};
}

///CMainTask::j_from_fight_route

CFSMTask::fsm_state_t CMainTask::j_from_fight_route(CTask*task)
{
    if(Bot().IsDead()){
        return {&m_spawn_task,id_spawn_st};
    }
    if(m_route.IsComplete())
    {
        m_calm_route.SetRouteViewing(nullptr);
        const CItemPoint* goal=m_route.CurrentGoal();
        if(goal==nullptr)
        {
            m_route.SetToPriority();
            return {&m_calm_route,id_calm_route_st};
        }
        if(goal->Type()!=id_health_getter&&goal->Type()!=id_armour_getter)
            {m_visited_points.RegVisit(m_route.CurrentGoal());}

        m_route.SetToPriority();
        return {&m_calm_route,id_calm_route_st};
    }
    if(m_enemies.DispatchNewCurrent())
    {
        float balance=DefBattleBalanse(
                      Bot(),m_enemies.Current().Enemy());
        if(balance>=0.5)
        {
            return {&m_pursuit,id_pursuit_st};
        }
        else if(balance<=-0.5)
        {
            m_calm_route.SetRouteViewing(fn_shooting_viewing(m_enemies));
            return {nullptr,id_go_away};
        }
        else{}
    }
    if(m_enemies.Current().Enemy().IsDead()||
       m_enemies.Current().LastVisTime()+8.0f<Time())
    {
        m_calm_route.SetRouteViewing(nullptr);
        return {nullptr,id_calm_route_st};
    }
    return {nullptr,id_fight_route_st};
}

CFSMTask::fsm_state_t CMainTask::j_from_pursuit(CTask*task)
{
    if(Bot().IsDead()){

        return {&m_spawn_task,id_spawn_st};
    }
    if(m_enemies.DispatchNewCurrent())
    {
        float balance=DefBattleBalanse(
                      Bot(),m_enemies.Current().Enemy());
        if((balance>-0.5)&&(balance<0.5))
        {
            m_route.SetToPriority();
            return {&m_calm_route,id_fight_route_st};
        }
        else if(balance<=-0.5)
        {
            m_route.SetToPriority();
            m_calm_route.SetRouteViewing(fn_shooting_viewing(m_enemies));
            return {&m_calm_route,id_go_away};
        }
        else{}
    }
    if(!task->IsComplete()) return {nullptr,id_pursuit_st};
    m_route.SetToPriority();
    m_calm_route.SetRouteViewing(nullptr);
    return {&m_calm_route,id_calm_route_st};
}


void CMainTask::jump_action(CFSMTask::fsm_state_t from_st,CFSMTask::fsm_state_t to_st)
{
    if(to_st.second!=id_spawn_st||
       to_st.second!=id_use_wall_st)
    {
        SelectWeapon(Bot());
    }

    static const std::map<int,const std::string>  iden_states=
    {
      {id_spawn_st,"spawn"},
      {id_calm_route_st,"route"},
      {id_fight_route_st,"fight"},
      {id_pursuit_st,"pursuit"},
      {id_go_away,"go_away"},
      {id_use_wall_st,"use_wall"},
    };
    if constexpr(BOT_DEBUG)
    {
         /*if(to_st.second==id_spawn_st||
            to_st.second==id_calm_route_st) return;

         ReportPrint("_______Switch from: "+get_property(iden_states,from_st.second)+
                    ", to: "+get_property(iden_states,to_st.second)+",time:"+
                    std::to_string(Time())+'\n');*/
    }
}

CFSMTask::fsm_state_t CMainTask::j_from_go_away(CTask*task)
{
    if(Bot().IsDead()){
        return {&m_spawn_task,id_spawn_st};
    }
    if(m_route.IsComplete())
    {
        m_calm_route.SetRouteViewing(nullptr);
        const CItemPoint* goal=m_route.CurrentGoal();
        if(goal==nullptr)
        {
            m_route.SetToPriority();
            return {&m_calm_route,id_calm_route_st};
        }
        if(goal->Type()!=id_health_getter&&goal->Type()!=id_armour_getter)
            {m_visited_points.RegVisit(m_route.CurrentGoal());}

        m_route.SetToPriority();
        return {&m_calm_route,id_calm_route_st};
    }
    if(m_enemies.DispatchNewCurrent())
    {
        float balance=DefBattleBalanse(
                      Bot(),m_enemies.Current().Enemy());
        if((balance>-0.5)&&(balance<0.5))
        {
            m_calm_route.SetRouteViewing(nullptr);
            return {nullptr,id_fight_route_st};
        }
        else if(balance>0.5)
        {
            m_calm_route.SetRouteViewing(nullptr);
            return {&m_pursuit,id_pursuit_st};
        }
        else{}
    }
    if(m_enemies.Current().Enemy().IsDead()||
       m_enemies.Current().LastVisTime()+8.0f<Time())
    {
        m_calm_route.SetRouteViewing(nullptr);
        return {nullptr,id_calm_route_st};
    }
    return {nullptr,id_go_away};
}

///DefBattleBalanse
// Определение баланса сил при вступлении в бой

float DefBattleBalanse(const CBot&bot,const player_t&enemy)
{
    float balance=0.0f;
    balance+=(bot.Health()-50.0f)/100.0f+bot.Armour()/100.0f;
    uint_t all_items=bot.Player().AllItems();
    if(all_items&Power_weapon)
    {
        balance+=1.0f;
    }
    else if(all_items&Normal_weapon)
    {
        balance+=0.5f;
    }
    else{}
    uint_t enemy_item=enemy.ActiveItem();
    if(enemy_item&Power_weapon)
    {
        balance-=1.0f;
    }
    else if(enemy_item&Normal_weapon)
    {
        balance-=0.5f;
    }
    else{}
    balance+=bot.Chars().aggression;
    return restrict_range(balance,-1.0f,1.0f);
}

///SelectWeapon
// Выбрать наиболее эффективное оружие

void SelectWeapon(const CBot& bot)
{
    uint_t current=bot.ActiveItem();
    if(current==0) return;
    uint_t for_choise=current;

    float max_priority=ItemProperties(current).default_priority;
    generate_flags weap(bot.Player().AllItems()&Weapon);
    for(weap.begin();!weap.end();weap.next())
    {
        const auto& cur_prop=ItemProperties(weap);
        assert(cur_prop.ammo&Ammunition);
        if(bot.Player().Ammo(cur_prop.ammo)>0&&
           cur_prop.default_priority>max_priority)
        {
            for_choise=weap;
            max_priority=cur_prop.default_priority;
        }
    }
    assert(bit_count(for_choise)==1);
    if(for_choise!=current)
        bot.Player().SelectItem(for_choise);
}

///CBotsPool

bool CBotsPool::m_Add(const std::string&name,float accur,float aggres)
{
    if constexpr(BOT_DEBUG)
    {
        ReportPrint("Creating bot:\nName:"+name+'\n'+
                  "Accuracy: "  +std::to_string(accur) +'\n'+
                  "Aggressive: "+std::to_string(aggres)+'\n');

    }

    CBot*new_bot=new CBot(name,accur,aggres);
    CMainTask*new_main=new CMainTask(*new_bot);
    for(auto&other:m_bots)
    {
        other.main_task->m_enemies.Add(new_bot->Player());
        new_main->m_enemies.Add(other.bot->Player());
    }
    if(m_me_visible)
    {
        new_main->m_enemies.Add(*glMe);
    }
    new_bot->SetExecuteTask(new_main);
    m_bots.push_back(bot_data_t());
    m_bots.back().bot.reset(new_bot);
    m_bots.back().main_task=new_main;
    return true;
}

bool CBotsPool::Add()
{
    if(!m_config_parse)
    {
        ParseConfig(m_game_config);
        m_config_parse=true;
    }
    if(!glCallbacks.IsCallback(CCallbacks::id_bot_run_cb))
    {
        glCallbacks.AddCallback(CCallbacks::id_bot_run_cb,
        {
            0.05,
            [this](){DoThink();}
        });
    }

    if(Size()>=m_max_bots) return false;

    if(Size()<m_game_config.size())
    {
        auto& [name,accur,aggress]=m_game_config[Size()];
        m_Add(name,accur,aggress);
    }
    else
    {
        m_Add("Bot_"+std::to_string(Size()+1),0.5f,0.5f);
    }

    return true;
}

void CBotsPool::SetMeVisible(bool new_vis)
{
    if(m_me_visible==new_vis) return;
    m_me_visible=new_vis;
    if(m_me_visible)
    {
        for(auto&other:m_bots)
        {
            other.main_task->m_enemies.Add(*glMe);
        }
    }
    else
    {
        for(auto&other:m_bots)
        {
            other.main_task->m_enemies.Erase(*glMe);
        }
    }
}


void CBotsPool::SetAccuracy(int accur)
{
    float fl_accur=static_cast<float>(restrict_range(accur,0,5));
    fl_accur/=5.0f;
    for(auto&other:m_bots)
    {
        other.bot->SetAccuracy(fl_accur);
    }
}

void CBotsPool::Clear()
{
    m_bots.clear();
    m_config_parse=false;
    glCallbacks.RemoveCallback(CCallbacks::id_bot_run_cb);
}

void CBotsPool::DoThink()
{
    for(auto&b_data:m_bots)
    {
        b_data.bot->Think();
    }
}
