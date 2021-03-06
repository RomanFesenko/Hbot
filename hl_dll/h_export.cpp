/***
*
*	Copyright (c) 1996-2002, Valve LLC. All rights reserved.
*
*	This product contains software technology licensed from Id
*	Software, Inc. ("Id Technology").  Id Technology (c) 1996 Id Software, Inc.
*	All Rights Reserved.
*
*   Use, distribution, and modification of this source code and/or resulting
*   object code is restricted to non-commercial enhancements to products from
*   Valve LLC.  All other use, distribution, or modification is prohibited
*   without written permission from Valve LLC.
*
****/
/*

===== h_export.cpp ========================================================

  Entity classes exported by Halflife.

*/

#include "extdll.h"
#include "util.h"

#include "cbase.h"

#include "hl_iface.h"
extern server_command_t glServerCommandProcessor;
extern vv_func_t glServerDeactivateCommand;
extern hl_to_bot_funcs hl_to_bot_funcs_impl;
#include <assert.h>

// Holds engine functionality callbacks
enginefuncs_t g_engfuncs;
globalvars_t  *gpGlobals;


#ifdef _WIN32

// Required DLL entry point
BOOL WINAPI DllMain(
   HINSTANCE hinstDLL,
   DWORD fdwReason,
   LPVOID lpvReserved)
{
	if      (fdwReason == DLL_PROCESS_ATTACH)
    {
    }
	else if (fdwReason == DLL_PROCESS_DETACH)
    {
    }
	return TRUE;
}

void DLLEXPORT GiveFnptrsToDll(	enginefuncs_t* pengfuncsFromEngine, globalvars_t *pGlobals )
{
	memcpy(&g_engfuncs, pengfuncsFromEngine, sizeof(enginefuncs_t));
	gpGlobals = pGlobals;
	HINSTANCE hst=LoadLibrary("valve/dlls/hbot.dll");
	assert(hst!=NULL);
	FARPROC proc=GetProcAddress(hst,"LinkData");
	assert(proc!=NULL);
    dll_export_t exe_f=(dll_export_t)proc;
    exe_f(&hl_to_bot_funcs_impl,glServerCommandProcessor,
          glServerDeactivateCommand);
}


#else

extern "C" {

void GiveFnptrsToDll(	enginefuncs_t* pengfuncsFromEngine, globalvars_t *pGlobals )
{
	memcpy(&g_engfuncs, pengfuncsFromEngine, sizeof(enginefuncs_t));
	gpGlobals = pGlobals;

	HINSTANCE hst=LoadLibrary("valve/dlls/hbot.dll");
	assert(hst!=NULL);
	FARPROC proc=GetProcAddress(hst,"LinkData");
	assert(proc!=NULL);
    dll_export_t exe_f=(dll_export_t)proc;
    exe_f(&hl_to_bot_funcs_impl,glServerCommandProcessor,
          glServerDeactivateCommand);
}

}

#endif
