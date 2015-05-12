
#include "../Ryan_Debug/logging.h"

namespace Ryan_Debug
{
	namespace log {
		const char* stringify(severity_level lev)
		{
			static const char* levels[] = {
				"debug_3", "debug_2", "debug_1",
				"normal", "notification", "warning",
				"error", "critical"
			};
			return levels[lev];
		}

	}

}
