#pragma once

#include <Ryan_Debug/plugin.h>
#include "info.h"

#define D_rtmath_validator() extern "C" void FORCE_DLEXPORT dlVerRTMATH(rtmath::versioning::versionInfo& vf, void** rd) \
				{ \
		rtmath::versioning::genVersionInfo(vf); \
		*rd = (void*) &(Ryan_Debug_registry_register_dll); }
//#define gcc_init(x) void __attribute__((constructor)) plugin_gcc_init() { x(); }
//#define msvc_init(x) BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved) \
//{ if (dwReason == DLL_PROCESS_ATTACH) x(); return true; }

#ifndef _MSC_FULL_VER
#define rtmath_plugin_init(x) d_dllVer(x); //gcc_init();
#else
#define rtmath_plugin_init(x) d_dllVer(x); //msvc_init();
#endif

