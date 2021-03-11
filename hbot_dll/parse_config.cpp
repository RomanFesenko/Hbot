#include "parse_config.h"
#include "util.h"
#include <regex>


using std::string;
using std::regex;
using std::regex_search;
using std::regex_match;
using std::smatch;

bool FileToString(const string&file_name,string&dest)
{
    FILE* in_f=fopen(file_name.c_str(),"r");
    dest.clear();
    if(in_f==nullptr){return false;}
    char temp_line[3];
    while(fgets(temp_line,2,in_f)!=0)
    {
        dest+=temp_line;
    }
    fclose(in_f);
    return true;
}


std::string identifier_regex()
{
    return "[_[:alpha:]][_[:alnum:]]*";
}

std::string int_number_regex()
{
    return "[[:digit:]]+";
}

inline std::string float_number_regex(char point)
{
    return std::string("[[:digit:]]+([")+point+
           std::string("][[:digit:]]+)?");
}

/*
 bot{
   name=some_name;
   accuracy=0.34;
   aggressive=0.84;
}
bot{...........
*/

void ParseConfig(game_config_t&bot_data)
{
    if constexpr(BOT_DEBUG)
    {
        ReportPrint("Parsing bot_data.txt...\n");
    }
    bot_data.clear();
    string  data;
    if(!FileToString(BotDirectory()+"bot_data.txt",data))
    {
        return;
    }
    auto erased_delim=[](char char_)
    {
        return (char_==' ') ||
               (char_=='\n')||
               (char_=='\t');
    };
    std::erase_if(data,erased_delim); //C++20


    regex bot_data_regex("bot[{]name=("+
                        identifier_regex()+");"+
                        "accuracy=("+
                        float_number_regex('.')+");"+
                        "aggressive=("+
                        float_number_regex('.')+");[}]");

    auto pos=data.cbegin();
    auto end=data.cend();
    smatch matcher;
    for(;regex_search(pos,end,matcher,bot_data_regex);
        pos=matcher.suffix().first)
    {
        assert(matcher.size()==6);

        bot_data.push_back({matcher[1].str(),
                            std::stof(matcher[2].str()),
                            std::stof(matcher[4].str())});
    }
}

