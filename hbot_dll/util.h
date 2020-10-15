#ifndef _util_
#define _util_

#include<memory>
#include <vector>
#include<utility>

#include <functional>


#include "hl_ifase.h"
#include "items.h"
#include "../../BaseLibraries/GEOMETRY/primitives.h"


using Vector=Point3D<float>;

const Vector glOrtX=Vector(1.0f,0.0f,0.0f);
const Vector glOrtY=Vector(0.0f,1.0f,0.0f);
const Vector glOrtZ=Vector(0.0f,0.0f,1.0f);
const float CosDegree[10]={
                cos(0),
                cos(Degree),
                cos(2*Degree),
                cos(3*Degree),
                cos(4*Degree),
                cos(5*Degree),
                cos(6*Degree),
                cos(7*Degree),
                cos(8*Degree),
                cos(9*Degree)};



const bool BOT_DEBUG=true;


const int MaxPlayers=32;

class player_t
{
    void* m_handle;
    public:
    explicit player_t(void* _ptr):m_handle(_ptr){}
    void Move(const Vector& angles,float speed,unsigned short buttons,unsigned char msec);
    void SetViewAngle( Vector& view);

    Vector Center()const;
    Vector Eyes()const;
    Vector ViewAngle()const;
    float Health()const;
    float Armour()const;
    bool IsDead()const;
    bool IsAlive()const;
    bool HasItem(uint_t item_flag)const;
    uint_t ActiveItem()const;
    void SelectItem(uint_t item_flag)const;
    uint_t AllItems()const;
    int Ammo(uint_t item_flag)const;
    int InClip()const;

    friend bool Visible(const Vector& from, const Vector&to,const player_t&);
};


inline bool Visible(const Vector& from, const Vector&to,const player_t& ignore)
{
    return glHLiface.pfnTraceLine(from.data(),to.data(),ignore.m_handle);
}

inline void ServerPrint(const std::string& str)
{
    glHLiface.pfnServerPrint(str.c_str());
}

inline player_t CreateFakePlayer(const std::string& str)
{
    return player_t(glHLiface.pfnCreateFakePlayer(str.c_str()));
}

inline void SetCallBack(float tact,vv_pfn_t pfn)
{
    glHLiface.pfnSetCallback(tact,pfn);
}

void AllItemsInMap(std::vector<std::pair<int,Vector>>& res);

inline float Time()
{
    return glHLiface.pfnTime();
}

inline std::string MapName()
{
    return std::string(glHLiface.pfnMapName());
}

inline std::string BotDirectory()
{
    return "valve/BotFiles/";
}

class beam_t
{
    BeamData m_data;
    std::shared_ptr<void> m_handle;
    struct beam_deleter
    {
        void operator()(void* ptr)
        {
            if(ptr!=nullptr)
            glHLiface.pfnDeleteBeam(ptr);
        }
    };
    public:
    beam_t(const Vector& from,const Vector& to,
           const Vector& color=Vector(255,255,255),float wid=50)
    //:m_handle(nullptr,beam_deleter())
    {
        from.to_array(m_data.begin);
        to.to_array(m_data.end);
        color.to_array(m_data.color);
        m_data.width=wid;
        m_handle.reset(glHLiface.pfnShowBeam(0,&m_data),beam_deleter());
    }
    void SetColor(const Vector& col)
    {
        col.to_array(m_data.color);
        glHLiface.pfnShowBeam(m_handle.get(),&m_data);
    }
    void SetWidth(int _w)
    {
        m_data.width=_w;
        glHLiface.pfnShowBeam(m_handle.get(),&m_data);
    }
    void SetEnds(const Vector& from,const Vector& to)
    {
        from.to_array(m_data.begin);
        to.to_array(m_data.end);
        glHLiface.pfnShowBeam(m_handle.get(),&m_data);
    }
    ~beam_t(){}
};

void PlayersInGame(std::vector<player_t>& players);

void FakePlayersInGame(std::vector<player_t>& players);

Vector AnglesToVec(const Vector& _angles);

Vector VecToAngles(const Vector& forward);
float NormalizeAngle(float);

std::string ToString(const Vector& coords);

float GetRandomFloat(float ,float);
int GetRandomInt(int ,int);


class CTimer
{
    float m_time_alarm;
    public:
    CTimer(){}
    bool IsAlarm()const;
    void SetAlarm(float);
};

class Point
{
    const Vector& m_vec;
    public:
    explicit Point(const Vector& vec):m_vec(vec){}
    const Vector&get()const{return m_vec;}
};

class Angles
{
    const Vector& m_vec;
    public:
    explicit Angles(const Vector& vec):m_vec(vec){}
    const Vector&get()const{return m_vec;}
};

template<class _Ret_t,class _Mem_t>
class updater
{
    using default_func_t=std::function<_Ret_t()>;
    using update_func_t=std::function<std::pair<_Ret_t,_Mem_t>(_Mem_t,float)>;
    default_func_t m_default_func;
    update_func_t m_update_func;
    float m_tact;
    float m_last_update_time;
    _Mem_t m_last_update_value;
    public:
    updater(float tact,_Mem_t start_value,
    default_func_t deff,update_func_t update):
    m_default_func(deff),
    m_update_func(update),
    m_tact(tact),m_last_update_value(start_value){}

    _Ret_t operator()()
    {
        float current=Time();
        if(current<m_last_update_time+m_tact)
        {
            return m_default_func();
        }
        else
        {
            m_last_update_time=current;
            auto up_pr= m_update_func(m_last_update_value,m_last_update_time);
            m_last_update_value=up_pr.second;
            return up_pr.first;
        }
    }
};

void CallbacksRun();

class CCallbacks
{
    using vv_func_t=std::function<void()>;

    struct callback_t{
        int m_identifier;
        float m_period;
        vv_func_t m_functor;
        float m_last_call_time;
    };

    auto finder(int iden)const
    {
        return [iden](const callback_t& callback)
        {
            return callback.m_identifier==iden;
        };
    }

    std::vector<callback_t> m_callbacks;
    using size_type=std::vector<callback_t>::size_type;

    void Run();
    public:
    enum{
        id_bot_run_cb=0,
        id_graph_view_cb,
        id_path_tracker_cb,
        id_graph_test
    };
    CCallbacks();
    bool AddCallback(int iden,std::pair<float ,vv_func_t>);
    bool RemoveCallback(int iden);
    bool IsCallback(int iden)const;
    size_type size()const;
    void Clear();
    friend void CallbacksRun();
};

inline CCallbacks glCallbacks;

void ReportPrint(const std::string&report);
void WarningPrint(const std::string&report);
void ErrorPrint(const std::string&report);

template<class _Val>
_Val restrict_range(_Val val,_Val min_val,_Val max_val)
{
    if(val<min_val) return min_val;
    if(max_val<val) return max_val;
    return val;
}

template<class _Val,class _Comparer>
_Val restrict_range(_Val val,_Val min_val,_Val max_val,_Comparer comp)
{
    if(comp(val,min_val)) return min_val;
    if(comp(max_val,val)) return max_val;
    return val;
}


#endif






