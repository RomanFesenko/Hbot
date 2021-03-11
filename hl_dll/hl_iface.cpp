#include "extdll.h"
#include "util.h"
#include "cbase.h"
#include "player.h"
#include "weapons.h"
#include "client.h"

#include "hl_iface.h"

vv_func_t glServerDeactivateCommand;
server_command_t glServerCommandProcessor;
extern enginefuncs_t g_engfuncs;


LINK_ENTITY_TO_CLASS( clock, Clock );
bool Clock::is_init=false;

Clock* glClock;

namespace bot{

void ServerPrint( const char * str)
{
    g_engfuncs.pfnServerPrint(str);
}

void* ShowBeam( void* ptrBeam, BeamData* bd)
{
    CBeam *bm;
    if(ptrBeam==0)
    {
        bm=CBeam::BeamCreate( g_pModelNameLaser, 10 );
        bm->SetScrollRate( 255 );
        bm->SetBrightness( 120 );
    }
    else
    {
        bm=static_cast<CBeam*>(ptrBeam);
    }
    bm->PointsInit( Vector(bd->begin),Vector(bd->end) );
    bm->SetColor((bd->color)[0],(bd->color)[1],(bd->color)[2]);
    bm->SetWidth( bd->width );
    return bm;
}

void DeleteBeam(void*ptrBeam )
{
    REMOVE_ENTITY(ENT( (static_cast<CBeam *>(ptrBeam))->pev));
}

int  Players(void** result)
{
    edict_t		*pEdict = g_engfuncs.pfnPEntityOfEntIndex( 1 );
	CBaseEntity *pEntity;
	const char * str;
	int num_players=0;
    for ( int i = 1; i < gpGlobals->maxEntities; i++, pEdict++ )
	{
		pEntity = CBaseEntity::Instance(pEdict);
		if ( !pEntity ) continue;
        str=STRING((pEdict->v.classname));
        if(FStrEq("player",str ))
        {
            if(!(pEdict->v.flags&FL_FAKECLIENT))
            {
                result[num_players++]=(void*)pEntity;
            }
        }
	}
	return num_players;
}

int  FakePlayers(void** result)
{
    edict_t		*pEdict = g_engfuncs.pfnPEntityOfEntIndex( 1 );
	CBaseEntity *pEntity;
	const char * str;
	int num_players=0;
    for ( int i = 1; i < gpGlobals->maxEntities; i++, pEdict++ )
	{
		pEntity = CBaseEntity::Instance(pEdict);
		if ( !pEntity ) continue;
        str=STRING((pEdict->v.classname));
        if(FStrEq("player",str ))
        {
            if(pEdict->v.flags&FL_FAKECLIENT)
            {
                result[num_players++]=(void*)pEntity;
            }
        }
	}
	return num_players;
}

void* CreateFakePlayer(const char* bot_name)
{
    edict_t *botedit=g_engfuncs.pfnCreateFakeClient(bot_name);
    entvars_t *pev = &(botedit->v);
    botedit->v.flags |= FL_FAKECLIENT;

    ClientPutInServer(botedit);
    botedit->v.framerate=1;
    botedit->v.gravity=1;
    botedit->v.friction=1;
    botedit->v.flags |= FL_FAKECLIENT;
    g_engfuncs.pfnRunPlayerMove( botedit,Vector(0.0,0.0,0.0) , 50.0,
                                   0, 0, IN_FORWARD, 0, 100);
    return GetClassPtr((CBasePlayer *)pev);
}


void DeleteFakePlayer(void* _player)
{
    CBasePlayer* ptr=( CBasePlayer*)_player;
    ClientDisconnect(ptr->edict());
    ptr->UpdateClientData();
    ptr->SUB_Remove();
}

void SetCallback(float tact,vv_func_t call_back)
{
    glClock->SetCallback(tact,call_back);
}

void PlayerMove(void* _player,const float* dir,float vel,unsigned short buttons,byte msec)
{
    CBasePlayer* ptr=( CBasePlayer*)_player;

    g_engfuncs.pfnRunPlayerMove
    (ptr->edict(),dir,vel,0, 0,buttons,0,msec);
}

void SetViewAngle(void* _player,float* dir)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    ptr->pev->v_angle.x=dir[0];
    ptr->pev->v_angle.y=dir[1];
    ptr->pev->v_angle.z=dir[2];

    ptr->pev->angles.x=-dir[0]/3;
    ptr->pev->angles.y=dir[1];
}

void CoordCenter(const void* _player,float* dest)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    entvars_t& pl_ev=*(ptr->pev);
    dest[0]=pl_ev.origin.x;
    dest[1]=pl_ev.origin.y;
    dest[2]=pl_ev.origin.z;
}

void CoordEyes(const void* _player,float*dest)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    entvars_t& pl_ev=*(ptr->pev);
    dest[0]=pl_ev.origin.x+pl_ev.view_ofs.x;
    dest[1]=pl_ev.origin.y+pl_ev.view_ofs.y;
    dest[2]=pl_ev.origin.z+pl_ev.view_ofs.z;
}

void ViewAngle(const void* _player,float*dest)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    entvars_t& pl_ev=*(ptr->pev);
    dest[0]=pl_ev.v_angle.x;
    dest[1]=pl_ev.v_angle.y;
    dest[2]=pl_ev.v_angle.z;
}

void ViewVecs(const void* _player,float* forward,float*left,float*up)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    entvars_t& pl_ev=*(ptr->pev);
    UTIL_MakeVectors(pl_ev.v_angle);
    forward[0]=gpGlobals->v_forward.x;
    forward[1]=gpGlobals->v_forward.y;
    forward[2]=gpGlobals->v_forward.z;
}

float Health(const void* _player)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    entvars_t* pl_ev=ptr->pev;
    return pl_ev->health;
}

float Armour(const void* _player)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    return ptr->pev->armorvalue;
}

bool IsDead(const void* _player)
{
    CBasePlayer* ptr=( CBasePlayer*)_player;
    return !ptr->IsAlive();
    //return (ptr->pev->deadflag==DEAD_DEAD)||(ptr->pev->deadflag==DEAD_RESPAWNABLE);
}

bool TraceLine(const float *v1, const float *v2, const void * _ignore_player)
{
    CBasePlayer* ptr=(CBasePlayer*)_ignore_player;
    TraceResult tr;
    g_engfuncs.pfnTraceLine(v1,v2,TRUE|0x100, ptr->edict(), &tr);
    return tr.flFraction == 1.0;
}


int HasItem(const void* _player,const char* item)
{
    CBasePlayer* ptr=(CBasePlayer*)_player;
    return ptr->HasNamedPlayerItem( item );
}

const char* ActiveItem(const void* _player)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    if(ptr->m_pActiveItem==0) return 0;
    return STRING(ptr->m_pActiveItem->pev->classname);
}

void SelectItem(void* _player,const char* item)
{
    CBasePlayer* ptr=(CBasePlayer*)_player;
    ptr->SelectItem(item);
}

int AllItems(const void*_player ,const char** items)
{
    int num_items=0;
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    for(int i = 0 ; i < MAX_ITEM_TYPES ; i++)
    {
        CBasePlayerItem *pPlayerItem =ptr->m_rgpPlayerItems[ i ];
        while(pPlayerItem)
        {
            items[num_items++]=STRING(pPlayerItem->pev->classname);
            pPlayerItem=pPlayerItem->m_pNext;
        }
    }
    return num_items;
}

int Ammo(const void* _player,int type)
{
    int res=0;
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    switch(type)
    {
        case id_ammo_9mmAR:res=ptr->ammo_9mm;break;
        case id_ammo_357:res=ptr->ammo_357;break;
        case id_ammo_crossbow:res=ptr->ammo_bolts;break;
        case id_ammo_buckshot:res=ptr->ammo_buckshot;break;
        case id_ammo_rpgclip:res=ptr->ammo_rockets;break;
        case id_ammo_gaussclip:res=ptr->ammo_uranium;break;
        case id_ammo_hornets:res=ptr->ammo_hornets;break;
        default:res=-10;
    }
    return res;
}

int InClip(const void*_player)
{
    const CBasePlayer* ptr=(const CBasePlayer*)_player;
    CBasePlayerWeapon* bw=(CBasePlayerWeapon*)(ptr->m_pActiveItem);
    if(bw==0) return 0;
    return bw->m_iClip;
}

int AllItemsInMap(const char** items,float* coords)
{
    int numitems=0;
    int coord_index=0;
    edict_t		*pEdict = g_engfuncs.pfnPEntityOfEntIndex( 1 );
	CBaseEntity *pEntity;
    for ( int i = 1; i < gpGlobals->maxEntities; i++, pEdict++ )
	{
		pEntity = CBaseEntity::Instance(pEdict);
		if ( !pEntity )
			continue;

        items[numitems++]=STRING(pEntity->pev->classname);
        coords[coord_index++]=pEntity->pev->origin.x;
        coords[coord_index++]=pEntity->pev->origin.y;
        coords[coord_index++]=pEntity->pev->origin.z;
	}
	return numitems;
}

float Time(){return gpGlobals->time;}

const char* MapName()
{
    return g_engfuncs.pfnSzFromIndex(gpGlobals->mapname);
}


}//bot

hl_to_bot_funcs hl_to_bot_funcs_impl=
{
    bot::ServerPrint,
    bot::ShowBeam,
    bot::DeleteBeam,
    bot::Players,
    bot::FakePlayers,
    bot::CreateFakePlayer,
    bot::DeleteFakePlayer,
    bot::SetCallback,
    bot::PlayerMove,
    bot::SetViewAngle,

    bot::CoordCenter,
    bot::CoordEyes,
    bot::ViewAngle,
    bot::ViewVecs,
    bot::Health,
    bot::Armour,
    bot::IsDead,
    bot::TraceLine,
    bot::HasItem,
    bot::ActiveItem,
    bot::SelectItem,
    bot::AllItems,
    bot::Ammo,
    bot::InClip,
    bot::AllItemsInMap,
    bot::Time,
    bot::MapName
};








