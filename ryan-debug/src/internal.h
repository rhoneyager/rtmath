#pragma once

#include <string>

#ifdef _WIN32
#include <Windows.h>
#endif

namespace Ryan_Debug
{
	namespace debug {
		extern std::string sConfigDefaultFile; // in debug.cpp

		//bool checkDuplicateRyan_Debug();
	}
#ifdef _WIN32
	/// Convert from unicode string to multibyte
	std::string convertStr(const LPTSTR instr);
#endif
}


