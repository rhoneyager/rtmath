/* debug.h - The debugging system for the library */
#pragma once
#include "../defs.h"

#include <iostream>
#include <map>
#include <string>
#include <vector>
#include "error.h"

namespace boost { namespace program_options { 
	class options_description; class variables_map; } 
	}

namespace rtmath
{
	namespace debug
	{
		/// Generate the startup message giving library information
		void DLEXPORT_rtmath_core debug_preamble(std::ostream &out = std::cerr);
		/// Get revision of the code
		int DLEXPORT_rtmath_core rev(void);

		/// \brief Get number of threads available in the system
		/// \todo Transfer to Ryan_Debug.
		/// \todo Finish implementation using Windows and Linux system calls.
		size_t DLEXPORT_rtmath_core getConcurrentThreadsSupported();
		/**
		* \brief Adds options to a program
		*
		* \item cmdline provides options only allowed on the command line
		* \item config provides options available on the command line and in a config file
		* \item hidden provides options allowed anywhere, but are not displayed to the user
		**/
		void DLEXPORT_rtmath_core add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/// Processes static options defined in add_options
		/// \todo Add processor for non-static options
		void DLEXPORT_rtmath_core process_static_options(
			boost::program_options::variables_map &vm);

		/// \brief Private variable indication command-line-specified default rtmath config file
		/// \todo Move into a header-invisible location.
		extern std::string SHARED_PRIVATE sConfigDefaultFile;
	}
}
