#ifndef _hl_iface_
#define _hl_iface_

enum ItemIdentifiers
{
    id_weapon_crowbar=    (1 << 0),
    id_weapon_9mmhandgun= (1 << 1),
    id_weapon_shotgun=    (1 << 2),
    id_weapon_9mmAR=      (1 << 3),
    id_weapon_crossbow=   (1 << 4),
    id_weapon_hornetgun=  (1 << 5),
    id_weapon_357=        (1 << 6),
    id_weapon_gauss=      (1 << 7),
    id_weapon_rpg=        (1 << 8),
    id_weapon_egon=       (1 << 9),
    id_ammo_buckshot=     (1 << 10),
    id_ammo_9mmAR=        (1 << 11),
    id_ammo_357=          (1 << 12),
    id_ammo_gaussclip=    (1 << 13),
    id_ammo_rpgclip=      (1 << 14),
    id_ammo_crossbow=     (1 << 15),
    id_ammo_hornets=      (1 << 16),

    id_weapon_handgrenade=(1 << 17),
    id_weapon_satchel=    (1 << 18),
    id_healthkit=         (1 << 19),

    id_health_getter=     (1 << 20),
    id_armour_getter=     (1 << 21),
    id_elevator_button=   (1 << 22),
    id_observation_point= (1 << 23),

    id_error=             (1 << 31)
};

struct BeamData
{
    float begin[3];
    float end[3];
    int color[3];
    int width;
};

struct hl_to_bot_funcs
{
    void (*pfnServerPrint)( const char * );

    void*(*pfnShowBeam)( void*, BeamData*);
    void (*pfnDeleteBeam)(void*);

    int  (*pfnPlayers)(void**); // human players
    int  (*pfnFakePlayers)(void**); // bots
    void*(*pfnCreateFakePlayer)(const char*);
    void (*pfnDeleteFakePlayer)(void*);

    void (*pfnSetCallback)(float,void(*)(void));

    void (*pfnPlayerMove)(void*,const float*,float,unsigned short buttons,byte msec);
    void (*pfnSetViewAngle)(void*,float*);
    void (*pfnCoordCenter)(const void*,float*);
    void (*pfnCoordEyes)(const void*,float*);
    void (*pfnViewAngle)(const void*,float*);
    void (*pfnViewVecs)(const void*,float*,float*,float*);
    float (*pfnHealth)(const void*);
    float (*pfnArmour)(const void*);
    bool (*pfnIsDead)(const void*);

    bool (*pfnTraceLine)(const float *v1, const float *v2,const void *);

    // presence / absence / installation of items
    int (*pfnHasItem)(const void*,const char*);
    const char* (*pfnActiveItem)(const void*);
    void (*pfnSelectItem)(void*,const char*);
    int (*pfnAllItems)(const void*,const char**);
    // ammo quantity
    int (*pfnAmmo)(const void*,int);
    int (*pfnInClip)(const void*);
    // all usable map objects
    int (*pfnAllItemsInMap)(const char**,float*);
    float (*pfnTime)();
    const char* (*pfnMapName)();
};

typedef void(*vv_func_t)(void);

class Clock;
extern Clock* glClock;

class Clock:public CBaseEntity
{
    vv_func_t m_callback;
    float m_tact;
    const float min_tact;
    public:
    static bool is_init;
    Clock():m_callback(0),m_tact(0.2),min_tact(0.05){}
    void SetCallback(float _t, vv_func_t _cb)
    {
        if(_t<min_tact) _t=min_tact;
        m_tact=_t;
        m_callback=_cb;
    }
    void Spawn()
    {
        is_init=true;
        glClock=this;

        g_engfuncs.pfnServerPrint("Clock run... \n");
        SetThink(ClockTick);
        ClockTick();
    }
    void ClockTick()
    {
        if(m_callback!=0)
        {
            m_callback();
        }
        pev->nextthink=gpGlobals->time+m_tact;
    }
};

typedef bool(*server_command_t)(const char*);

typedef void(*dll_export_t)(const hl_to_bot_funcs*,server_command_t&,
                            vv_func_t&);
#endif
