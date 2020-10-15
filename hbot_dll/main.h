#ifndef __MAIN_H__
#define __MAIN_H__

#include <windows.h>
#include "hl_ifase.h"

/*  To use this exported function of dll, include this header
 *  in your project.
 */

#ifdef BUILD_DLL
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __declspec(dllimport)
#endif


#ifdef __cplusplus
extern "C"
{
#endif

void  DLL_EXPORT LinkData(const hl_to_bot_funcs* ,server_command_t&,
                          vv_pfn_t& );

#ifdef __cplusplus
}
#endif

#endif // __MAIN_H__
