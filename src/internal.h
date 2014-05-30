#pragma once

#include <string>

#ifdef _WIN32
#include <Windows.h>
#endif

namespace Ryan_Debug
{

	void splitNullMap(
		const std::string &instr, std::map<std::string, std::string> &out);

#ifdef _WIN32
	/// Convert from unicode string to multibyte
	std::string convertStr(const LPTSTR instr);
#endif
}


