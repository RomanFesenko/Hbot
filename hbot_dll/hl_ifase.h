#ifndef _hl_ifase_
#define _hl_ifase_
#include <string>

using cus_t=const unsigned short;

cus_t IN_ATTACK	=(1 << 0);
cus_t IN_JUMP	=	(1 << 1);
cus_t IN_DUCK	=	(1 << 2);
cus_t IN_FORWARD =  (1 << 3);
cus_t IN_BACK	=	(1 << 4);
cus_t IN_USE	=	(1 << 5);
cus_t IN_CANCEL	=(1 << 6);
cus_t IN_LEFT	=	(1 << 7);
cus_t IN_RIGHT=	(1 << 8);
cus_t IN_MOVELEFT	=(1 << 9);
cus_t IN_MOVERIGHT= (1 << 10);
cus_t IN_ATTACK2	=(1 << 11);
cus_t IN_RUN    =  (1 << 12);
cus_t IN_RELOAD	=(1 << 13);
cus_t IN_ALT1	=	(1 << 14);
cus_t IN_SCORE	=(1 << 15);

const int IN_STAIRS=(1<<16);
const int IN_ELEVATOR=(1<<17);

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

    int  (*pfnPlayers)(void**); // humans
    int  (*pfnFakePlayers)(void**); // bots
    void*(*pfnCreateFakePlayer)(const char*);
    void (*pfnDeleteFakePlayer)(void*);

    void (*pfnSetCallback)(float,void(*)(void));

    void (*pfnPlayerMove)(void*,const float*,float,unsigned short buttons,unsigned char msec);
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

bool ServerCommand(const char*from_derver);
void ServerDeactivate();
typedef bool(*server_command_t)(const char*);
typedef void(*vv_pfn_t)(void);

inline hl_to_bot_funcs glHLiface;

inline void SetHLiface(const hl_to_bot_funcs* from_hl_dll,server_command_t& to_hl_dll,
                       vv_pfn_t&server_deact_cd)
{
    memcpy(&glHLiface,from_hl_dll,sizeof(hl_to_bot_funcs));
    to_hl_dll=ServerCommand;
    server_deact_cd=ServerDeactivate;
}
#endif
