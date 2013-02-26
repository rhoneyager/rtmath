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
	// appEntry and appExit are setup on dll load. No need to call them 
	// using msvc or gcc. Others may require an explicit call.
	void RYAN_DEBUG_DLEXPORT appEntry();
	void RYAN_DEBUG_DLEXPORT appExit();

	// Does the app pause on termination? Auto-detected.
	bool RYAN_DEBUG_DLEXPORT waitOnExit();
	void RYAN_DEBUG_DLEXPORT waitOnExit(bool);
	bool RYAN_DEBUG_DLEXPORT waitOnExitForce();

	// Process detection functions
	bool RYAN_DEBUG_DLEXPORT pidExists(int pid);
	int RYAN_DEBUG_DLEXPORT getPID();
	int RYAN_DEBUG_DLEXPORT getPPID(int pid);

	// Print the compiler information for the debug library to std::cerr.
	void RYAN_DEBUG_DLEXPORT printDebugInfo();
}

