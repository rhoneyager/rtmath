/* registry.h - DLL registration system for extended functionality */
#pragma once
#include "defs.h"

#include <iostream>
#include <string>

namespace boost {
	namespace program_options {
		class options_description; class variables_map;
	}
	namespace filesystem { class path; }
}

namespace rtmath
{
	namespace registry
	{
		/** \brief This is the basic structure passed by an rtmath DLL to provide 
		* identity information.
		**/
		struct DLLpreamble
		{
			/// Short DLL name
			const char* name;
			/// Short description
			const char* description;
			/// UUID that prevents different versions of the same DLL from being  
			/// loaded simultaneously.
			const char* uuid;
			/// Path of the loaded module (DLL leaves it blank)
			const char* path;
		};

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

		/// Load a DLL. If path is not absolute, use search paths.
		void DLEXPORT_rtmath_core loadDLL(const std::string &filename);

		/// Print loaded DLLs
		void DLEXPORT_rtmath_core printDLLs(std::ostream &out = std::cerr);

		/// List DLL search paths
		void DLEXPORT_rtmath_core printDLLsearchPaths(std::ostream &out = std::cerr);

		/// Find a DLL
	}
}

extern "C"
{
	/// Provides interface for DLLs to register basic information about themselves
	bool DLEXPORT_rtmath_core rtmath_registry_register_dll(const rtmath::registry::DLLpreamble&);

	/// Provides interface for DLLs to register a function hook
	bool DLEXPORT_rtmath_core rtmath_registry_register_hook(const char* uuid, const char* topic);

	
}

