#pragma once

#ifndef RYAN_DEBUG_NO_LINK
#ifdef _MSC_FULL_VER
#ifdef RYAN_DEBUG_LINK_STATIC
#pragma comment(lib, "ryan-debug_static.lib")
#else
#pragma comment(lib, "ryan-debug.lib")
#pragma comment(linker, "/include:_ryan_debug_dummy")
#endif
#endif
#endif

#ifdef RYAN_DEBUG_EXPORTING
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
#endif



namespace ryan_debug
{
	void RYAN_DEBUG_DLEXPORT appEntry();
	void RYAN_DEBUG_DLEXPORT appExit();
	bool RYAN_DEBUG_DLEXPORT pidExists(int pid);
	bool RYAN_DEBUG_DLEXPORT waitOnExit();
	int RYAN_DEBUG_DLEXPORT getPID();
	int RYAN_DEBUG_DLEXPORT getPPID(int pid);
}

/*
extern "C"
{
	void RYAN_DEBUG_DLEXPORT ryan_debug_dummy();
}
*/
