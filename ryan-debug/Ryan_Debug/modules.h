#pragma once

#include "defs.h"

namespace boost
{
	namespace filesystem {
		class path;
	}
}

namespace Ryan_Debug
{
	boost::filesystem::path RYAN_DEBUG_DLEXPORT convertUnix(const boost::filesystem::path &p);

#ifdef _WIN32
	bool RYAN_DEBUG_DLEXPORT findCygwinBaseDir(boost::filesystem::path &p);
#endif
	bool RYAN_DEBUG_DLEXPORT findUserHomeDir(boost::filesystem::path &p);
}


