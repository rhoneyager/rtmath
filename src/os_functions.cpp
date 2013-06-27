// OS-dependent functions that provide tons of useful information

// Tells if, on windows, the parent process is cmd or explorer
// Will be used in apps on exit to determine if the program should wait 
// before terminating or not
#define RYAN_DEBUG_EXPORTING
#define RYAN_DEBUG_NO_LINK

// debug_subversion.h is auto-generated
#include <debug_subversion.h>

#include <boost/filesystem.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <cstdlib>
#include <cstdio>
#include <time.h>
#ifdef _WIN32
//#include "../../rtmath/rtmath/ROOTlink.h"
#include <Windows.h>
#include <TlHelp32.h>
#include <Psapi.h>

#pragma comment(lib, "Psapi")

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

#undef environ // Conflicts with my structure
#endif
#ifdef __unix__
#include <unistd.h>
#include <sys/types.h>
#include <dlfcn.h>
#endif

//#include <TError.h> // ROOT info message suppression
// ImageMagick init functions
#ifdef _WIN32
#pragma warning( disable : 4251 )
#ifndef NO_IMAGEMAGICK
//#include <Magick++.h>
#endif
#endif


#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/info.h"

// DLL binding and unbinding code
#ifndef _MSC_FULL_VER //__GNUC__, __llvm__, __clang__, __INTEL_COMPILER, ...

/// GCC - code that is executed on DLL import
void __attribute__ ((constructor)) debug_gcc_init()
{
	Ryan_Debug::appEntry();
}

/// GCC - code that is executed on DLL unload
void __attribute__ ((destructor)) debug_gcc_fini()
{
}
#endif
#ifdef _MSC_FULL_VER
/// MSVC - code that is executed on DLL load and unload
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	if (dwReason == DLL_PROCESS_ATTACH)
	{
		Ryan_Debug::appEntry();
		// DisableThreadLibraryCalls(hInstance);
	}
	return true;
}
#endif

extern "C" {
	/// Useless function that is required for MSVC autolink (using the header debug.h)
	void RYAN_DEBUG_DLEXPORT Ryan_Debug_dummy()
	{
		// Dummy function that is used in auto-link
		int i=0;
		i++;
	}
}

namespace Ryan_Debug {

	// Don't export this symbol (not in header)
#ifdef _WIN32
	BOOL WINAPI _CloseHandlerRoutine( DWORD dwCtrlType ); // Helps gracefully close console
	bool _consoleTerminated = false;
#endif

	/// Private flag that determines if the app waits for the user to press 'Enter' to terminate it at the end of execution.
	bool doWaitOnExit = false;

	// Regular defs start here

	/** 
	 * \brief Entry function that gets called when a debugged application first loads
	 * 
	 * This function gets called at the beginning of an application's execution
	 * (generally). It:
	 * - determines if the app should wait on exit (to keep the console open)
	 * - resets the console title in case any other library overrides it.
	 *   A good example of this is the CERN ROOT image lobraries.
	 * - Overrides the console control key handlers on Windows. This lets a user 
	 *   exit with CTRL-C without the debug code causing the app to crash.
	 */
	void appEntry()
	{
		doWaitOnExit = waitOnExit();

		// Set appexit
		atexit(appExit);

		// Do ImageMagick intialization
#ifdef _WIN32
#ifndef NO_IMAGEMAGICK
		//Magick::InitializeMagick(*argv);
#endif
#endif

		// ROOT info message suppression
//			gErrorIgnoreLevel = 2000;

		// Prevent ROOT from renaming the console title on Windows
		// Do this by setting the window name to its file name and path

		// Note: nowadays, could use GetConsoleOriginalTitle?
#ifdef _WIN32
		// Get PID
		DWORD pid = 0;
		HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (NULL == h) return;
		PROCESSENTRY32 pe = { 0 };
		pe.dwSize = sizeof(PROCESSENTRY32);
		pid = GetCurrentProcessId();
		CloseHandle(h);

		// Get parent process name
		h = OpenProcess( PROCESS_QUERY_LIMITED_INFORMATION
								//| PROCESS_VM_READ
								,FALSE, pid );
		if (NULL == h) return;
		TCHAR szModName[600];
		DWORD success = 0;
		DWORD sz = sizeof(szModName) / sizeof(TCHAR);
		success = QueryFullProcessImageName(h,0,szModName,&sz);

		// Set Console Title
		SetConsoleTitle(szModName);


		// Also, set the window closing routine
		// This allows for the user to click the X (or press ctrl-c)
		// without causing a fault.
		// The fault is because the window closes before the atexit 
		// functions can write output.
		SetConsoleCtrlHandler(_CloseHandlerRoutine, true);
		CloseHandle(h);
#endif
	}

	/**
	 * \brief Function called on application exit to hold the console window open
	 *
	 * This function is the completion of the appEntry() code.
	 * If the window is already closed (such as by the user clicking the X or 
	 * pressing CTRL-C), then it silently falls through.
	 * Otherwise, it examines the doWaitOnExit flag.
	 * If the application is spawned in its own window (parent is not cmd.exe), 
	 * then it prompts the user to press the return key.
	 */
	void appExit()
	{
		using namespace std;

#ifdef _WIN32
		// If console is closed, then don't even bother 
		// Windows app will fault if closed before main returns
		if (_consoleTerminated)
		{
			return;
		}
#endif
		
		
		cerr << endl << endl;
//			rtmath::debug::listuniqueobj(cerr, false);
//#endif

		if (doWaitOnExit)
		{
//#ifdef _WIN32
//				_CrtDumpMemoryLeaks();
//#endif
			cerr << endl << "Program terminated. Press return to exit." << endl;
			std::getchar();
		}

	}

#ifdef _WIN32
	/// Windows handler for window close events.
	BOOL WINAPI _CloseHandlerRoutine( DWORD dwCtrlType )
	{
		/*
		if (dwCtrlType == CTRL_CLOSE_EVENT)
		{
			_consoleTerminated = true;
			//return true;
		}
		*/
		_consoleTerminated = true;
		return false;
	}
#endif
	/// Checks whether a process with the given pid exists.
	bool pidExists(int pid)
	{
		// Function needed because Qt is insufficient, and Windows / Unix have 
		// different methods of ascertaining this.
#ifdef _WIN32
		HANDLE h;
		h = OpenProcess( PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid );
		if (h)
		{
			DWORD code = 0;
			if (GetExitCodeProcess(h, &code))
			{
				CloseHandle(h);
				if (code == STILL_ACTIVE)
				{
					return true;
				} else {
					return false;
				}
			} else {
				CloseHandle(h);
				return false;
			}
			CloseHandle(h);
			return true;
		} else {
			return false;
		}
#endif
#ifdef __unix__
		// Need to check existence of directory /proc/pid
		std::ostringstream pname;
		pname << "/proc/" << pid;
		using namespace boost::filesystem;
		path p(pname.str().c_str());
		if (exists(p)) return true;
		return false;
#endif
		// Execution should not reach this point
	}


#ifdef _WIN32
	/// Windows function for getting process name and path
	bool getPathWIN32(DWORD pid, boost::filesystem::path &modPath, boost::filesystem::path &filename)
	{
		HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (NULL == h) return false;
		// Get parent process name
		h = OpenProcess( PROCESS_QUERY_LIMITED_INFORMATION
								//| PROCESS_VM_READ
								,FALSE, pid );
		if (NULL == h) return false;
		TCHAR szModName[600];
		DWORD success = 0;
		DWORD sz = sizeof(szModName) / sizeof(TCHAR);
		success = QueryFullProcessImageName(h,0,szModName,&sz);
		//success = GetModuleFileNameEx(h,NULL,szModName,sizeof(szModName) / sizeof(TCHAR));

		// If using unicode and not multibyte...
		// Convert wchar to char in preparation for string and path conversion
#ifdef UNICODE

		size_t origsize = wcslen(szModName) + 1;

		const size_t newsize = 600;
		size_t convertedChars = 0;
		char nstring[newsize];
		wcstombs_s(&convertedChars, nstring, origsize, szModName, _TRUNCATE);
		boost::filesystem::path modPathm(nstring);
#else
		boost::filesystem::path modPathm(szModName);
#endif
		boost::filesystem::path filenamem = modPathm.filename();

		modPath = modPathm;
		filename = filenamem;

		CloseHandle(h);
		if (!success) 
		{
			success = GetLastError();
			std::cout << "Failure\n" << success << std::endl;
			return false;
		}
		return true;
	}
#endif


	/// Allows the linked app to force / prohibit waiting on exit
	void waitOnExit(bool val)
	{
		doWaitOnExit = val;
	}

	/// Will this app execution result in waiting on exit?
	bool waitOnExit()
	{
		static bool check = true;
		if (check) waitOnExit(waitOnExitForce());
		return doWaitOnExit;
	}

	/// Checks parent PID and name to see if the app should wait on exit.
	bool waitOnExitForce()
	{
#ifdef _WIN32
		// Get pid and parent pid
		DWORD pid = 0, ppid = 0;
		HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (NULL == h) return false;
		PROCESSENTRY32 pe = { 0 };
		pe.dwSize = sizeof(PROCESSENTRY32);
		pid = GetCurrentProcessId();
		if( Process32First(h, &pe)) {
			do {
				if (pe.th32ProcessID == pid) {
					ppid = pe.th32ParentProcessID;
					//printf("PID: %i; PPID: %i\n", pid, pe.th32ParentProcessID);
				}
			} while( Process32Next(h, &pe));
		}

		//std::cout << "Pid " << pid << "\nPPID " << ppid << std::endl;
		CloseHandle(h);

		// Get parent process name
		boost::filesystem::path filename, filepath;
		getPathWIN32(ppid, filepath, filename);

		//std::cout << filename.string() << std::endl;
		// If run from cmd, no need to wait
		if (filename.string() == "cmd.exe") return false;
		// Don't need these due to end return. Just for reference.
		//if (filename.string() == "devenv.exe") return true;
		//if (filename.string() == "explorer.exe") return true;
		return true;
#else
		return false;
#endif
	}

	/// OS-independent function that returns the PID of the running process
	int getPID()
	{
#ifdef _WIN32
		DWORD pid = 0;
		HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (NULL == h) return false;
		PROCESSENTRY32 pe = { 0 };
		pe.dwSize = sizeof(PROCESSENTRY32);
		pid = GetCurrentProcessId();
		CloseHandle(h);
		return (int) pid;
#endif
#ifdef __unix__
		//pid_t getpid(void);
		return (int) getpid();
#endif
		return 0;
	}

	/// OS-independent function that returns the PID of the parent of a given process
	int getPPID(int pid)
	{
#ifdef _WIN32
		DWORD Dpid = pid, ppid = 0;
		HANDLE h = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
		if (NULL == h) return false;
		PROCESSENTRY32 pe = { 0 };
		pe.dwSize = sizeof(PROCESSENTRY32);
		if (!pid) Dpid = GetCurrentProcessId();
		if( Process32First(h, &pe)) {
			do {
				if (pe.th32ProcessID == Dpid) {
					ppid = pe.th32ParentProcessID;
					//printf("PID: %i; PPID: %i\n", pid, pe.th32ParentProcessID);
				}
			} while( Process32Next(h, &pe));
		}

		//std::cout << "Pid " << pid << "\nPPID " << ppid << std::endl;
		CloseHandle(h);
		return (int) ppid;
#endif
#ifdef __unix__
		// pid_t getppid(void);
		return (int) getppid();
#endif
		return 0;
	}

	/**
	 * \brief Provides information about the given process.
	 *
	 * Reads in process information from /proc or by querying the os.
	 * Returns a structure containing the process:
	 * - PID
	 * - PPID
	 * - Executable name
	 * - Executable path
	 * - Current working directory
	 * - Environment
	 * - Command-line
	 * - Process start time
	 *
	 * \throws std::exception if the process does not exist
	 **/
	processInfo getInfo(int pid)
	{
		processInfo res;
		res.pid = pid;
		if (!pidExists(pid)) throw std::string("PID does not exist"); // TODO: exception
		res.ppid = getPPID(pid);
#ifdef __unix__
		{
			using namespace boost::filesystem;
			using namespace std;
			ostringstream procpath;
			procpath << "/proc/" << pid;
			string sp = procpath.str();
			// exe and cwd are symlinks. Read them.
			// path = exe. executable name is the file name from the path.
			path pp(sp);
			path pexe = pp / "exe";
			path pcwd = pp / "cwd";

			path psexe = read_symlink(pexe);
			path pscwd = read_symlink(pcwd);
			path pexename = psexe.filename();

			res.name = pexename.string();
			res.path = psexe.string();
			res.cwd = pscwd.string();

			// environ and cmdline can be read as files
			// both internally use null-terminated strings
			// TODO: figure out what to do with this.
			path pcmd(pp / "cmdline");
			ifstream scmdline(pcmd.string().c_str());
			const int length = 1024;
			char *buffer = new char[length];
			while (scmdline.good())
			{
				scmdline.read(buffer,length);
				res.cmdline.append(buffer,scmdline.gcount());
			}
			//scmdline >> res.cmdline;
			// Replace command-line null symbols with spaces
			std::replace(res.cmdline.begin(),res.cmdline.end(),
					'\0', ' ');

			path penv(pp / "environ");
			ifstream senviron(penv.string().c_str());

			while (senviron.good())
			{
				senviron.read(buffer,length);
				res.environ.append(buffer,senviron.gcount());
			}
			delete[] buffer;

			// start time is the timestamp of the /proc/pid folder.
			std::time_t st = last_write_time(pp);
			string ct(ctime(&st));
			res.startTime = ct;

		}
		return res;
#endif
#ifdef _WIN32
		//throw std::string("Unimplemented on WIN32"); // unimplemented
		boost::filesystem::path filename, filepath;
		getPathWIN32((DWORD) pid, filepath, filename); // int always fits in DWORD
		res.name = filename.string();
		res.path = filepath.string();
		res.cmdline;
		res.cwd;
		res.startTime;
		res.environ;
		return res;
#endif
		// Should only reach here if not unix or win32. An odd possibility.
		throw std::string("Unimplemented OS");
	}

	/**
	 * \brief Prints compiler and library information that was present when the 
	 * Ryan-Debug library was compiled.
	 */
	void printDebugInfo()
	{
		debug_preamble(std::cerr);
	}
}

/**
 * \brief Provides functaionality to write processInfo in a stream.
 **/
std::ostream & operator<<(std::ostream &stream, const Ryan_Debug::processInfo &obj)
{
	using std::endl;
	stream << "Process Info for " << obj.name << endl;
	stream << "\tName:\t" << obj.name << endl;
	stream << "\tPID:\t" << obj.pid << endl;
	stream << "\tPPID:\t" << obj.ppid << endl;
	stream << "\tPath:\t" << obj.path << endl;
	stream << "\tCWD:\t" << obj.cwd << endl;
	stream << "\tCmd Line:\t" << obj.cmdline << endl;
	stream << "\tStart:\t" << obj.startTime << endl;
	// stream << "\tEnviron:\n";
	// TODO: parse and write the environment

	stream << endl;

	return stream;
}

