// Os-dependent functions that provide tons of useful information

// Tells if, on windows, the parent process is cmd or explorer
// Will be used in apps on exit to determine if the program should wait 
// before terminating or not
#define RYAN_DEBUG_EXPORTING
#define RYAN_DEBUG_NO_LINK

#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>
#include <cstdio>
#ifdef _WIN32
//#include "../../rtmath/rtmath/ROOTlink.h"
#include <Windows.h>
#include <TlHelp32.h>
#include <Psapi.h>

#pragma comment(lib, "Psapi")

#define _CRTDBG_MAP_ALLOC
#include <cstdlib>
#include <crtdbg.h>

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

#include "../Ryan-Debug/debug.h"
#include "../Ryan-Debug/info.h"

// DLL binding and unbinding code
#ifndef _MSC_FULL_VER //__GNUC__, __llvm__, __clang__, __INTEL_COMPILER, ...
void __attribute__ ((constructor)) debug_gcc_init()
{
	ryan_debug::appEntry();
}

void __attribute__ ((destructor)) debug_gcc_fini()
{
}
#endif
#ifdef _MSC_FULL_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	if (dwReason == DLL_PROCESS_ATTACH)
	{
		ryan_debug::appEntry();
		// DisableThreadLibraryCalls(hInstance);
	}
	return true;
}
#endif

extern "C" {
	void RYAN_DEBUG_DLEXPORT ryan_debug_dummy()
	{
		// Dummy function that is used in auto-link
		int i=0;
		i++;
	}
}

namespace ryan_debug {

		// Don't export this symbol (not in header)
#ifdef _WIN32
		BOOL WINAPI _CloseHandlerRoutine( DWORD dwCtrlType ); // Helps gracefully close console
		bool _consoleTerminated = false;
#endif

		bool doWaitOnExit = false;

		// Regular defs start here
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

		void waitOnExit(bool val)
		{
			doWaitOnExit = val;
		}

		bool waitOnExit()
		{
			static bool check = true;
			if (check) waitOnExit(waitOnExitForce());
			return doWaitOnExit;
		}

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
			h = OpenProcess( PROCESS_QUERY_LIMITED_INFORMATION
									//| PROCESS_VM_READ
									,FALSE, ppid );
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
			boost::filesystem::path modPath(nstring);
#else
			boost::filesystem::path modPath(szModName);
#endif
			boost::filesystem::path filename = modPath.filename();
			CloseHandle(h);
			if (!success) 
			{
				success = GetLastError();
				std::cout << "Failure\n" << success << std::endl;
			}
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

		void printDebugInfo()
		{
			debug_preamble(std::cerr);
		}
}

