
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
			if (lev > 7) return levels[7];
			return levels[(size_t) lev];
		}
	}
}
std::ostream& operator<< (std::ostream& strm, ::Ryan_Debug::log::severity_level level) {
	const char* res = ::Ryan_Debug::log::stringify(level);
	strm << res;
	return strm;
}

::boost::log::formatting_ostream& operator<< (
		::boost::log::formatting_ostream& strm,
		::boost::log::to_log_manip< ::Ryan_Debug::log::severity_level, ::Ryan_Debug::log::tag::severity > const& manip) {
	::Ryan_Debug::log::severity_level level = manip.get();
	const char* res = ::Ryan_Debug::log::stringify(level);
	strm << res;
	return strm;
}

//	}

//}


