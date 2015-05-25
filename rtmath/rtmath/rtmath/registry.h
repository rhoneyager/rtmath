/* registry.h - DLL registration system for extended functionality */
#pragma once
#include "defs.h"

#pragma warning( disable : 4661 ) // Exporting vector

#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include "error/debug.h"

namespace boost {
	namespace program_options {
		class options_description; class variables_map;
	}
}

namespace rtmath
{
	namespace registry
	{
		/// Internal function used in templates that writes to the registry log
		void DLEXPORT_rtmath_core emit_registry_log(const std::string&, ::rtmath::debug::severity_level = ::rtmath::debug::debug_2);

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
	}
}
