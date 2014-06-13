#pragma once
#include "defs.h"

#include <string>
#include <iostream>
#include <ostream>
// Intended as a single dll, so using std::vector is complicated.
//#include <vector>

namespace boost
{
	namespace program_options {
		class options_description;
		class variables_map;
	}
	namespace filesystem {
		class path;
	}
}


namespace Ryan_Debug
{
	struct processInfo;
	typedef const processInfo* hProcessInfo;

	// appEntry and appExit are setup on dll load. No need to call them 
	// using msvc or gcc. Others may require an explicit call.
	void RYAN_DEBUG_DLEXPORT appEntry();
	void RYAN_DEBUG_DLEXPORT appExit();

	// Does the app pause on termination? Auto-detected.
	bool RYAN_DEBUG_DLEXPORT waitOnExit();
	void RYAN_DEBUG_DLEXPORT waitOnExit(bool);
	bool RYAN_DEBUG_DLEXPORT waitOnExitForce();

#ifdef _WIN32
	/// Is the current program running in admin mode (WIN32 only)
	bool RYAN_DEBUG_DLEXPORT IsAppRunningAsAdminMode();
#endif

	// Process detection functions
	bool RYAN_DEBUG_DLEXPORT pidExists(int pid);
	int RYAN_DEBUG_DLEXPORT getPID();
	int RYAN_DEBUG_DLEXPORT getPID(const hProcessInfo);
	int RYAN_DEBUG_DLEXPORT getPPID(int pid);
	int RYAN_DEBUG_DLEXPORT getPPID(const hProcessInfo);

	/// Get a handle to a structure representing process information
	hProcessInfo RYAN_DEBUG_DLEXPORT getInfo(int pid);

	RYAN_DEBUG_DLEXPORT const char* getName(const hProcessInfo);
	RYAN_DEBUG_DLEXPORT const char* getPath(const hProcessInfo);
	RYAN_DEBUG_DLEXPORT const char* getCwd(const hProcessInfo);
	RYAN_DEBUG_DLEXPORT const char* getEnviron(const hProcessInfo, size_t &sz);
	RYAN_DEBUG_DLEXPORT const char* getCmdline(const hProcessInfo, size_t &sz);
	RYAN_DEBUG_DLEXPORT const char* getStartTime(const hProcessInfo);
	void RYAN_DEBUG_DLEXPORT freeProcessInfo(hProcessInfo);

	/// Get current username
	RYAN_DEBUG_DLEXPORT const char* getUsername();

	/// Get system hostname
	RYAN_DEBUG_DLEXPORT const char* getHostname();

	/// Print the compiler information for the debug library.
	void RYAN_DEBUG_DLEXPORT printDebugInfo(std::ostream &out = std::cerr);

	/// \brief Get number of threads available in the system
	/// \todo Finish implementation using Windows and Linux system calls.
	size_t RYAN_DEBUG_DLEXPORT getConcurrentThreadsSupported();

	/**
	* \brief Adds options to a program
	*
	* \item cmdline provides options only allowed on the command line
	* \item config provides options available on the command line and in a config file
	* \item hidden provides options allowed anywhere, but are not displayed to the user
	**/
	void RYAN_DEBUG_DLEXPORT add_options(
		boost::program_options::options_description &cmdline,
		boost::program_options::options_description &config,
		boost::program_options::options_description &hidden);
	/// Processes static options defined in add_options
	void RYAN_DEBUG_DLEXPORT process_static_options(
		boost::program_options::variables_map &vm);
}

/// Allows writing of a ryan_debug::processInfo structure to a stream
RYAN_DEBUG_DLEXPORT std::ostream &  operator<<(std::ostream&, const Ryan_Debug::processInfo&);

