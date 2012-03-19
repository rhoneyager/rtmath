// Os-dependent functions that provide tons of useful information

// Tells if, on windows, the parent process is cmd or explorer
// Will be used in apps on exit to determine if the program should wait 
// before terminating or not

#include "../rtmath/Stdafx.h"
#include <boost/filesystem.hpp>
#include <iostream>
#include <string>
#include <cstdlib>
#ifdef _WIN32
#include <Windows.h>
#include <TlHelp32.h>
#include <Psapi.h>
#pragma comment(lib, "Psapi")
#endif
#include <TError.h> // ROOT info message suppression
#include "../rtmath/error/debug.h"
#include "../rtmath/config.h"
#include "../rtmath/command.h"

namespace rtmath {
	namespace debug {

		void appEntry(int argc, char** argv)
		{
			// Process parameters
			config::parseParams p(argc,argv);
			// - Give library information?
			if (p.readParam("--vlib"))
			{
				std::cerr << "rtmath library information:\n";
				debug::debug_preamble();
				//exit(1);
			}
			// - Set default config file path?
			{
				std::string rtconfpath;
				if (p.readParam<std::string>("--rtconfpath", rtconfpath))
				{
					// Check for file existence
					if (!exists(boost::filesystem::path(rtconfpath))) 
						throw debug::xMissingFile(rtconfpath.c_str());
					// Attempt to load this file
					// This will block any other rtconf root load calls!
					config::loadRtconfRoot(rtconfpath);
				}
			}

			// Set appexit
			atexit(appExit);

			// ROOT info message suppression
			gErrorIgnoreLevel = 2000;

			// Prevent ROOT from renaming the console title on Windows
			// Do this by setting the window name to its file name and path
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
#endif
		}

		void appExit()
		{
			using namespace std;
			bool wait = rtmath::debug::waitOnExit();
			if (wait)
			{
				cerr << endl << "Program terminated. Press return to exit." << endl;
				std::getchar();
			}
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

	}; // end namespace debug
}; // end namespace rtmath
