#pragma once

#include "defs.h"

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
	boost::filesystem::path RYAN_DEBUG_DLEXPORT convertUnix(const boost::filesystem::path &p);
}


