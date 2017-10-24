#pragma once
#include <string>

namespace Ryan_Debug
{
	namespace log {

		enum severity_level
		{
			debug_3,
			debug_2,
			debug_1,
			normal,
			notification,
			warning,
			error,
			critical
		};
	}
	namespace registry {
		/// Internal function used in templates that writes to the registry log
		void RYAN_DEBUG_DLEXPORT emit_registry_log(const std::string&, ::Ryan_Debug::log::severity_level = ::Ryan_Debug::log::debug_2);

	}
}
