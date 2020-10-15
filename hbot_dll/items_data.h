#ifndef _items_data_
#define _items_data_

#include "items.h"

template<class T>
const T& get_property(const std::map<int,T>&prop_map,
               uint_t ident)
{
    auto iter=prop_map.find(ident);
    assert(iter!=prop_map.end());
    return iter->second;
}

struct item_properties_t
{
    float storage_restore_time;
    int ammo;
    int max_in_clip;
    int max_total;
    float recharge_time;
    float shoot_frequency;//в секунду
    float shoot_damage;
    float target_rad;
    float default_priority;
};

const std::map<int,item_properties_t> glItemPropertiesMap=
{
    {
        id_weapon_crowbar,
        {0,0,0,0,0,0,0,0,0}
    },
    {
        id_weapon_9mmhandgun,
        {30.0f,id_ammo_9mmAR,17,250,2.0f,2.0f,10.0f,25.0,0}
    },
    {
        id_weapon_shotgun,
        {30.0f,id_ammo_buckshot,8,100,5.0f,1.0f,45.0f,35.0,0.48}
    },
    {
        id_weapon_9mmAR,
        {30.0f,id_ammo_9mmAR,30,250,2.0f,5.0f,8.0f,30.0,0.52}
    },
    {
        id_weapon_crossbow,
        {30.0f,id_ammo_crossbow,5,30,3.0f,1.0f,100.0f,20.0,0.46}
    },
    {
        id_weapon_hornetgun,
        {30.0f,id_ammo_hornets,8,8,8.0f,2.0f,10.0f,40.0,0.5}
    },
    {
        id_weapon_357,
        {30.0f,id_ammo_357,6,30,3.0f,0.8f,60.0f,20.0,0.4}
    },
    {
        id_weapon_gauss,
        {30.0f,id_ammo_gaussclip,-1,100,0.0f,2.0f,20.0f,25.0,0.7}
    },
    {
        id_weapon_rpg,
        {30.0f,id_ammo_rpgclip,1,5,3.0f,4.0f,100.0f,30.0,0.78}
    },
    {
        id_weapon_egon,
        {30.0f,id_ammo_gaussclip,-1,100,0.0f,1.0f,30.0f,30.0,0.8}
    },
    {
        id_ammo_buckshot,
        {30.0f,id_weapon_shotgun,0,0,0,0,0,0,0}
    },
    {
        id_ammo_9mmAR,
        {30.0f,id_weapon_9mmAR,0,0,0,0,0,0,0}
    },
    {
        id_ammo_357,
        {30.0f,id_weapon_357,0,0,0,0,0,0,0}
    },
    {
        id_ammo_gaussclip,
        {30.0f,id_weapon_gauss|id_weapon_egon,0,0,0,0,0,0,0}
    },
    {
        id_ammo_rpgclip,
        {30.0f,id_weapon_rpg,0,0,0,0,0,0,0}
    },
    {
        id_ammo_crossbow,
        {30.0f,id_weapon_crossbow,0,0,0,0,0,0,0}
    },
    {
        id_ammo_hornets,
        {30.0f,id_weapon_hornetgun,0,0,0,0,0,0,0}
    },
    {
        id_weapon_handgrenade,
        {30.0f,0,1,10,1,1,100,0,0.3}
    },
    {
        id_health_getter,
        {30.0f,0,0,0,0,0,0,0,1.0}
    },
    {
        id_armour_getter,
        {30.0f,0,0,0,0,0,0,0,1.0}
    },
    {
        id_elevator_button,
        {2.0f,0,0,0,0,0,0,0,0}
    }
};

inline const item_properties_t& ItemProperties(uint_t item)
{
    return get_property(glItemPropertiesMap,item);
}

const uint_t Normal_weapon=
id_weapon_shotgun|id_weapon_9mmAR|id_weapon_hornetgun;

const uint_t Power_weapon=
id_weapon_gauss|id_weapon_egon|id_weapon_rpg;

const uint_t AssaultWeapon=Normal_weapon|Power_weapon;

const uint_t Sniper_weapon=
id_weapon_crossbow|id_weapon_gauss|id_weapon_357;

const uint_t Ammunition=
id_ammo_buckshot|
id_ammo_9mmAR|
id_ammo_357|
id_ammo_gaussclip|
id_ammo_rpgclip|
id_ammo_crossbow|
id_ammo_hornets;

const uint_t Weapon=
id_weapon_9mmhandgun|
id_weapon_shotgun|
id_weapon_9mmAR|
id_weapon_crossbow|
id_weapon_hornetgun|
id_weapon_357|
id_weapon_gauss|
id_weapon_rpg|
id_weapon_egon;


const uint_t Rechargeable=id_weapon_shotgun|
                            id_weapon_9mmAR|
                         id_weapon_crossbow|
                              id_weapon_357|
                       id_weapon_9mmhandgun;


#endif //
