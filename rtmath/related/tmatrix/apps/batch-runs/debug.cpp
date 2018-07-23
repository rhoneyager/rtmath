#include <boost/version.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include <cstdlib>
#include "debug.h"
#ifdef _WIN32
#include <Windows.h>
#include <TlHelp32.h>
#include <Psapi.h>
#pragma comment(lib, "Psapi")
#endif
#ifdef __unix__
#include <unistd.h>
#include <sys/types.h>
#endif

namespace tmatrix
{
	namespace debug
	{

		void debug_preamble(std::ostream &out)
		{
			out << "tmatrix library" << std::endl;
			out << "Compiled on " << __DATE__ << " at " << __TIME__ << std::endl;
#ifdef SUB_REV
			out << "SVN Revision " << SUB_REV << std::endl;
			out << "SVN Revision Date: " << SUB_DATE << std::endl;
//			out << "SVN Working Copy Range: " << SUB_WCRANGE << std::endl;
			out << "SVN Source: " << SUB_SOURCE << std::endl;
#else
			out << "SVN Repository Information Unknown" << std::endl;
#endif
#ifdef _DEBUG
			out << "Debug Version" << std::endl;
#else
			out << "Release Version" << std::endl;
#endif
#ifdef _OPENMP
			out << "OpenMP Supported" << std::endl;
#else
			out << "OpenMP Disabled" << std::endl;
#endif
#ifdef __amd64
			out << "64-bit build" << std::endl;
#endif
#ifdef _M_X64
			out << "64-bit build" << std::endl;
#endif
#ifdef __unix__
			out << "Unix / Linux Compile" << std::endl;
#endif
#ifdef __APPLE__
			out << "Mac Os X Compile" << std::endl;
#endif
#ifdef _WIN32
			out << "Windows Compile" << std::endl;
#endif
#ifdef __GNUC__
			out << "GNU Compiler Suite " << __GNUC__ << "." << __GNUC_MINOR__ << "." << __GNUC_PATCHLEVEL__ << std::endl;
#endif
#ifdef __MINGW32__
			out << "MinGW Compiler Suite " << __MINGW32_MAJOR_VERSION << "." << __MINGW32_MINOR_VERSION << std::endl;
#endif
#ifdef __SUNPRO_CC
			out << "Sun Studio Compiler " << __SUNPRO_CC << std::endl;
#endif
#ifdef __PATHCC__
			out << "EKOPath Compiler " << __PATHCC__ << "." << __PATHCC_MINOR__ << "." << __PATHCC_PATCHLEVEL__ << std::endl;
#endif
#ifdef __llvm__
			out << "LLVM Compiler Suite" << std::endl;
#endif
#ifdef __clang__
			out << " clang " << __clang_major__ << "." << __clang_minor__ << "." << __clang_patchlevel__ << std::endl;
#endif
#ifdef __INTEL_COMPILER
			out << "Intel Compiler Version " << __INTEL_COMPILER << std::endl;
			out << " Compiler build date: " << __INTEL_COMPILER_BUILD_DATE << std::endl;
#endif
#ifdef _MSC_FULL_VER
			out << "Microsoft Visual Studio Compiler Version " << _MSC_FULL_VER << std::endl;
#endif
			out << "Boost version " << BOOST_LIB_VERSION << std::endl;
			out << std::endl;
		};


		// Don't export this symbol (not in header)
#ifdef _WIN32
		BOOL WINAPI _CloseHandlerRoutine( DWORD dwCtrlType ); // Helps gracefully close console
		bool _consoleTerminated = false;
#endif

		// Regular defs start here
		void appEntry(int argc, char** argv)
		{
			// Process parameters

			// Set appexit
			atexit(appExit);

			// ROOT info message suppression
			//gErrorIgnoreLevel = 2000;

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

			// If debugging, display the debug mark information

			
#ifdef _WIN32
			// If console is closed, then don't even bother 
			// Windows app will fault if closed before main returns
			if (_consoleTerminated)
			{
				return;
			}
#endif
			
			
			cerr << endl << endl;
			//debug::listuniqueobj(cerr, false);
//#endif

			bool wait = debug::waitOnExit();
			if (wait)
			{
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

		bool waitOnExit()
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



	}
}

