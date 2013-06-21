#pragma once

#ifndef RYAN_DEBUG_NO_LINK
#ifdef _MSC_FULL_VER
#ifdef _M_X64
#pragma comment(linker, "/include:ryan_debug_dummy")
#else
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

#include <string>
//#include <vector>

namespace ryan_debug
{
	struct processInfo;

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
	processInfo getInfo(int pid);

	// Print the compiler information for the debug library to std::cerr.
	void RYAN_DEBUG_DLEXPORT printDebugInfo();

	// Contains information about a process
	struct processInfo
	{
		std::string name;
		std::string path;
		std::string cwd;
		std::string environ;
		std::string cmdline;
		std::string startTime;
		int pid;
		int ppid;
	};
}

std::ostream & operator<<(std::ostream&, const ryan_debug::processInfo&);

