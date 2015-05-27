/* registry.h - DLL registration system for extended functionality */
#pragma once
#include "defs.h"

#include <iostream>
#include <string>
#include <Ryan_Debug/logging_base.h>
//#include "error/debug.h"

namespace boost {
	namespace program_options {
		class options_description; class variables_map;
	}
}
namespace Ryan_Debug {
	namespace registry {
		class dllValidatorSet;
	}
}
namespace rtmath
{
	namespace registry
	{
		/// Internal function used in templates that writes to the registry log
		void DLEXPORT_rtmath_core emit_registry_log(const std::string&, ::Ryan_Debug::log::severity_level = ::Ryan_Debug::log::debug_2);

		/**
		* \brief Adds options to a program
		*
		* \item cmdline provides options only allowed on the command line
		* \item config provides options available on the command line and in a config file
		* \item hidden provides options allowed anywhere, but are not displayed to the user
		*
		* \note This gets automatically called by rtmath::debug::add_options
		* \todo Hide this
		**/
		void DLEXPORT_rtmath_core add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/** \brief Processes static options defined in add_options
		 * \todo Add processor for non-static options
		 * \note This gets automatically called by rtmath::debug::process_static_options
		 * \todo Hide this from apps
		 **/
		void DLEXPORT_rtmath_core process_static_options(
			boost::program_options::variables_map &vm);

		namespace dll {
			/// Load a DLL.
			void DLEXPORT_rtmath_core loadDLL(const std::string &filename);
			/// Load DLLs.
			void DLEXPORT_rtmath_core loadDLLs(const std::vector<std::string> &dlls);

		}
	}
}
