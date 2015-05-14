#pragma once



#ifdef EXPORTING_Ryan_Debug

#ifdef _MSC_FULL_VER
#define RYAN_DEBUG_DLEXPORT __declspec(dllexport)
#else
#define RYAN_DEBUG_DLEXPORT
#endif

#else

#ifdef _MSC_FULL_VER
#define RYAN_DEBUG_DLEXPORT __declspec(dllimport)
#else
#define RYAN_DEBUG_DLEXPORT
#endif

#ifndef RYAN_DEBUG_NO_LINK
#ifdef _MSC_FULL_VER
#ifdef _M_X64
#pragma comment(linker, "/include:Ryan_Debug_dummy")
#else
#pragma comment(linker, "/include:_Ryan_Debug_dummy")
#endif
#endif
#endif

#endif


#ifdef _MSC_FULL_VER
#define FORCE_DLEXPORT __declspec(dllexport)
#else
#define FORCE_DLEXPORT
#endif


#ifdef _MSC_FULL_VER
#include "Ryan.Ryan_Debug.manifest.h"
#endif
