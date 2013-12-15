#pragma once

#include "registry.h"


#define gcc_init(x) void __attribute__((constructor)) plugin_gcc_init() { x(); }
#define msvc_init(x) BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved) \
{ if (dwReason == DLL_PROCESS_ATTACH) x(); return true; }

#ifndef _MSC_FULL_VER
#define rtmath_plugin_init(x) gcc_init(x)
#else
#define rtmath_plugin_init(x) msvc_init(x)
#endif

#ifdef _WIN32
#include "windows.h"
typedef HINSTANCE dlHandleType;
#endif
#ifdef __unix__
#include "dlfcn.h"
typedef void* dlHandleType;
#endif

