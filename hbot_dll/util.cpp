#include "util.h"
#include <random>


void AllItemsInMap(std::vector<std::pair<int,Vector>>& res)
{
   res.clear();
   const char* Items[MaxItems];
   float coords[MaxItems*3];
   int num_items=glHLiface.pfnAllItemsInMap(Items,coords);
   for(int i=0;i<num_items;++i)
   {
       if(uint_t id_=get_identifier(Items[i]);id_!=id_error)
       {
           res.push_back({id_,Vector(coords+3*i)});
       }
   }
}

void PlayersInGame(std::vector<player_t>& players)
{
    players.clear();
    void* ptrs[MaxPlayers];
    int num=glHLiface.pfnPlayers(ptrs);
    for(int i=0;i<num;++i)
    {
        players.push_back(player_t(ptrs[i]));
    }
}

void FakePlayersInGame(std::vector<player_t>& players)
{
    players.clear();
    void* ptrs[MaxPlayers];
    int num=glHLiface.pfnFakePlayers(ptrs);
    for(int i=0;i<num;++i)
    {
        players.push_back(player_t(ptrs[i]));
    }
}

Vector AnglesToVec(const Vector& _angles)
{
    Vector angles=to_radian(_angles);
    float cos_teta=cos(angles[0]);
    return
    Vector{cos_teta*cos(angles[1]),cos_teta*sin(angles[1]),-sin(angles[0])};
}

Vector VecToAngles(const Vector& forward)
{
    if(abs(forward[0])<0.1f&&abs(forward[1])<0.1f&&abs(forward[2])<0.1f)
    {
       // assert(false);
        return Vector(0.0f,0.0f,0.0f);
    }
    Vector ort=normalize_3D(forward);

    if(abs(ort[2])>0.999f) return Vector(-asin(ort[2]),0.0f,0.0f);

    float fi_ang=acos(ort[0]/sqrt(1-ort[2]*ort[2]));

    return  to_degree(Vector(-asin(ort[2]),(ort[1]>0)? fi_ang:-fi_ang, 0));
}

std::string ToString(const Vector& coords)
{
    return "x:"+std::to_string(coords[0])+','+
    "y:"+std::to_string(coords[1])+','+
    "z:"+std::to_string(coords[2]);
}


float GetRandomFloat(float min_val,float max_val)
{
    static std::default_random_engine st_dre;
    static std::uniform_real_distribution<float> st_urd;
    std::uniform_real_distribution<float>::param_type pt(min_val,max_val);

    return st_urd(st_dre,pt);
}

int GetRandomInt(int min_val,int max_val)
{
    static std::default_random_engine st_dre;
    static std::uniform_int_distribution<int> st_urd;
    std::uniform_int_distribution<int>::param_type pt(min_val,max_val);

    return st_urd(st_dre,pt);
}



float NormalizeAngle(float angle)
{
    while(angle>180.0f){angle-=360.0f;}
    while(angle<-180.0f){angle+=360.0f;}
    return angle;
}

bool CTimer::IsAlarm()const
{
    return Time()>m_time_alarm;
}

void CTimer::SetAlarm(float time)
{
    m_time_alarm=time;
}

void CallbacksRun(){glCallbacks.Run();}

CCallbacks::CCallbacks(){}

void CCallbacks::Run()
{
    float current=Time();
    for(auto& cb_func:m_callbacks)
    {
        if(current>cb_func.m_last_call_time+cb_func.m_period)
        {
            cb_func.m_functor();
            cb_func.m_last_call_time=current;
        }
    }
}

bool CCallbacks::AddCallback(int iden,std::pair<float ,vv_func_t> functor)
{
    auto iter=std::find_if(m_callbacks.begin(),
                           m_callbacks.end(),
                           finder(iden));

    if(iter==m_callbacks.end())
    {

       functor.second();

       m_callbacks.push_back({iden,
                              functor.first,
                              functor.second,
                              Time()
                              });
        return true;
    }

    return false;
}

bool CCallbacks::RemoveCallback(int iden)
{
    auto iter=std::find_if(m_callbacks.begin(),
                         m_callbacks.end(),
                         finder(iden));

    if(iter!=m_callbacks.end())
    {
        *iter=m_callbacks.back();
        m_callbacks.pop_back();
        return true;
    }
    return false;
}


bool CCallbacks::IsCallback(int iden)const
{
    return std::find_if(m_callbacks.begin(),
                         m_callbacks.end(),
                         finder(iden))!=m_callbacks.end();
}

CCallbacks::size_type CCallbacks::size()const
{
    return m_callbacks.size();
}

void CCallbacks::Clear()
{
    m_callbacks.clear();
}

void ReportPrint(const std::string&report)
{
    FILE*temp=fopen((BotDirectory()+"bot_report.txt").c_str(),"a");
    assert(temp!=nullptr);
    fwrite(report.c_str(),sizeof(char),report.size(),temp);
    fclose(temp);
}

void WarningPrint(const std::string&report)
{
    ReportPrint("------>WARNING:");
    ReportPrint(report);
}

void ErrorPrint(const std::string&report)
{
    ReportPrint("------>ERROR:");
    ReportPrint(report);
}

///player_t

void player_t::Move(const Vector& angles,float speed,unsigned short buttons,unsigned char msec)
{
    glHLiface.pfnPlayerMove(m_handle,angles.data(),speed,buttons,msec);
}

void player_t::SetViewAngle( Vector& view)
{
    glHLiface.pfnSetViewAngle(m_handle,view.data());
}

Vector player_t::Center()const
{
    Vector res;
    glHLiface.pfnCoordCenter(m_handle,res.data());
    return res;
}

Vector player_t::Eyes()const
{
    Vector res;
    glHLiface.pfnCoordEyes(m_handle,res.data());
    return res;
}

Vector player_t::ViewAngle()const
{
    Vector res;
    glHLiface.pfnViewAngle(m_handle,res.data());
    return res;
}

float player_t::Health()const
{
    return glHLiface.pfnHealth(m_handle);
}

float player_t::Armour()const
{
    return glHLiface.pfnArmour(m_handle);
}

bool player_t::IsDead()const
{
    return glHLiface.pfnIsDead(m_handle);
}

bool player_t::IsAlive()const
{
    return !glHLiface.pfnIsDead(m_handle);
}

bool player_t::HasItem(uint_t item_flag)const
{
    const std::string& str_item=get_string(item_flag);
    if(str_item==glVoidString) return false;
    return glHLiface.pfnHasItem(m_handle,str_item.c_str())!=0;
}

uint_t player_t::ActiveItem()const
{
    const char* c_item=glHLiface.pfnActiveItem(m_handle);
    if(c_item==0) return 0;
    std::string str_item(c_item);
    return get_identifier(str_item);
}

void player_t::SelectItem(uint_t item_flag)const
{
    const std::string& str_item=get_string(item_flag);
    if(str_item==glVoidString) return;
    glHLiface.pfnSelectItem(m_handle,str_item.c_str());
}

uint_t player_t::AllItems()const
{
    const char* Items[MaxItems];
    int num_res=glHLiface.pfnAllItems(m_handle,Items);
    uint_t res=0;
    for(int i=0;i<num_res;++i)
    {
        if(uint_t fl=get_identifier(Items[i]);fl!=id_error)
        {
            res|=fl;
        }
    }
    return res;
}

int player_t::Ammo(uint_t item_flag)const
{
    return glHLiface.pfnAmmo(m_handle,item_flag);
}

int player_t::InClip()const
{
    return glHLiface.pfnInClip(m_handle);
}
