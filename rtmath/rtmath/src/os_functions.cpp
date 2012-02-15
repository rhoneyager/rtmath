// Os-dependent functions that provide tons of useful information

// Tells if, on windows, the parent process is cmd or explorer
// Will be used in apps on exit to determine if the program should wait 
// before terminating or not

#include "../rtmath/Stdafx.h"
#include <boost/filesystem.hpp>
#include <iostream>
#ifdef _WIN32
#include <Windows.h>
#include <TlHelp32.h>
#include <Psapi.h>
#pragma comment(lib, "Psapi")
#endif
#include "../rtmath/error/debug.h"

namespace rtmath {
	namespace debug {

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
			// Convert wchar to char in preparation for string and path conversion
			
			size_t origsize = wcslen(szModName) + 1;
			const size_t newsize = 600;
			size_t convertedChars = 0;
			char nstring[newsize];
			wcstombs_s(&convertedChars, nstring, origsize, szModName, _TRUNCATE);
			

			boost::filesystem::path modPath(nstring);
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
