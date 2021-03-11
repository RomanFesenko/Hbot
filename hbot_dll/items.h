#ifndef _items_
#define _items_

#include<map>
#include<string>

using uint_t=/*unsigned*/ int;
enum ItemIdentifiers:uint_t
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

const int MaxItemTypes=31;
const int MaxItems=300;
const int MaxItems_3=900;


const std::string glVoidString="Error identifier";

using ui_s_t=std::pair<uint_t,const std::string>;
#define IDEN_TO_STRING(itemstring) ui_s_t(id_ ## itemstring,#itemstring)
const std::map<uint_t,const std::string> glStringByIdentifiers=
{
    IDEN_TO_STRING(weapon_crowbar),
    IDEN_TO_STRING(weapon_9mmhandgun),
    IDEN_TO_STRING(weapon_shotgun),
    IDEN_TO_STRING(weapon_9mmAR),
    IDEN_TO_STRING(weapon_crossbow),
    IDEN_TO_STRING(weapon_hornetgun),
    IDEN_TO_STRING(weapon_357),
    IDEN_TO_STRING(weapon_gauss),
    IDEN_TO_STRING(weapon_rpg),
    IDEN_TO_STRING(weapon_egon),
    IDEN_TO_STRING(ammo_buckshot),
    IDEN_TO_STRING(ammo_9mmAR),
    IDEN_TO_STRING(ammo_357),
    IDEN_TO_STRING(ammo_gaussclip),
    IDEN_TO_STRING(ammo_rpgclip),
    IDEN_TO_STRING(ammo_crossbow),
    IDEN_TO_STRING(weapon_shotgun),

    IDEN_TO_STRING(weapon_handgrenade),
    IDEN_TO_STRING(weapon_satchel),
    IDEN_TO_STRING(healthkit),

    IDEN_TO_STRING(health_getter),
    IDEN_TO_STRING(armour_getter),
    IDEN_TO_STRING(elevator_button),
    IDEN_TO_STRING(observation_point)
};

using s_ui_t=std::pair<const std::string,uint_t>;
#define STRING_TO_IDEN(itemstring) s_ui_t(#itemstring,id_ ## itemstring)

const std::map<const std::string,uint_t> glIdentifiersByString=
{
    STRING_TO_IDEN(weapon_crowbar),
    STRING_TO_IDEN(weapon_9mmhandgun),
    STRING_TO_IDEN(weapon_shotgun),
    STRING_TO_IDEN(weapon_9mmAR),
    STRING_TO_IDEN(weapon_crossbow),
    STRING_TO_IDEN(weapon_hornetgun),
    STRING_TO_IDEN(weapon_357),
    STRING_TO_IDEN(weapon_gauss),
    STRING_TO_IDEN(weapon_rpg),
    STRING_TO_IDEN(weapon_egon),
    STRING_TO_IDEN(ammo_buckshot),
    STRING_TO_IDEN(ammo_9mmAR),
    STRING_TO_IDEN(ammo_357),
    STRING_TO_IDEN(ammo_gaussclip),
    STRING_TO_IDEN(ammo_rpgclip),
    STRING_TO_IDEN(ammo_crossbow),
    STRING_TO_IDEN(weapon_shotgun),

    STRING_TO_IDEN(weapon_handgrenade),
    STRING_TO_IDEN(weapon_satchel),
    STRING_TO_IDEN(healthkit),

    STRING_TO_IDEN(health_getter),
    STRING_TO_IDEN(armour_getter),
    STRING_TO_IDEN(elevator_button),
    STRING_TO_IDEN(observation_point)
};

inline const std::string& get_string(uint_t id)
{
    auto res=glStringByIdentifiers.find(id);
    return (res!=glStringByIdentifiers.end())? res->second:glVoidString;
}

inline uint_t get_identifier(const std::string& str)
{
    auto res=glIdentifiersByString.find(str);
    return (res!=glIdentifiersByString.end())? res->second: id_error;
}

class generate_flags
{
    uint_t m_bitset;
    uint_t m_for_return=1;
    void advance()
    {
        while(true)
        {
            if(m_for_return==id_error) return;
            if(m_for_return&m_bitset)return;
            m_for_return<<=1;
        }
    }
    public:
    generate_flags()
    {
        m_bitset=0;
        begin();
    }
    explicit generate_flags(uint_t _flags):m_bitset(_flags)
    {
        begin();
    }
    void set(uint_t fl)
    {
        m_bitset=fl;
        begin();
    }
    void begin()
    {
        m_for_return=1;
        advance();
    }
    void next()
    {
        m_for_return<<=1;
        advance();
    }
    bool end()const
    {
        return m_for_return==id_error;
    }
    operator uint_t()const
    {
        return m_for_return;
    }
};

constexpr int bit_count(uint_t num)
{
    int count=0;
    for(;num!=0;++count)
    {
        num&=num-1;
    }
    return count;
}

#endif //
