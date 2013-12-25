/* registry.h - DLL registration system for extended functionality */
#pragma once
#include "defs.h"

#include <functional>
#include <iostream>
//#include <map>
#include <set>
#include <string>
#include <vector>

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
			DLLpreamble(const char* name, const char* desc, const char* uuid)
				: name(name), description(desc), uuid(uuid), path(0) {}
			DLLpreamble() : name(0), description(0), uuid(0), path(0) {}
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

		/// Load a DLL.
		void DLEXPORT_rtmath_core loadDLL(const std::string &filename);
		/// Load DLLs.
		void DLEXPORT_rtmath_core loadDLLs(const std::vector<std::string> &dlls);

		/// Print loaded DLLs
		void DLEXPORT_rtmath_core printDLLs(std::ostream &out = std::cerr);

		/// List DLL search paths
		void DLEXPORT_rtmath_core printDLLsearchPaths(std::ostream &out = std::cerr);

		/// Find all occurances of a subpath in a search path.
		bool DLEXPORT_rtmath_core findPath(std::set<boost::filesystem::path> &matches,
			const boost::filesystem::path &expr,
			const std::set<boost::filesystem::path> &searchPaths, bool recurse);

		/**
		* \brief Locates all DLLs in the search path and loads them.
		*
		* The search path may be specified / manipulated from several locations:
		* - precompiled hints (from cmake)
		* - rtmath.conf
		* - the command line
		*
		* This takes all of the starting points in the initial search paths and recurses through the
		* directories, selecting dll and so files for matching. The load routine is aware of the
		* library build mode (Debug, Release, MinSizeRel, RelWithDebInfo). If one of these terms appears
		* in the path (folder tree + filename) of a library, then the dll is only loaded if its
		* build mode matches the library's mode. When the dll is actually loaded (other function),
		* the build mode is reported by the DLL more directly and checked again.
		*
		* \see searchPaths
		* \see loadSearchPaths
		* \see rtmath::registry::process_static_options
		**/
		void DLEXPORT_rtmath_core searchDLLs(std::vector<std::string> &dlls);
		void DLEXPORT_rtmath_core searchDLLs(std::vector<std::string> &dlls,
			const std::set<boost::filesystem::path> &searchPaths, bool recurse);

		/// Recursive and single-level DLL loading paths
		extern DLEXPORT_rtmath_core std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;

		// The type used to store all hooks for a class. string is topic.
		//typedef std::multimap<std::string, void*> classHookMapType;
		// For a given topic, a set of hooks (types vary by implementation).
		//typedef std::set<void*> topicHookSetType;
		// Query registry for hooks matching the specified class
		//void DLEXPORT_rtmath_core queryClass(const char* classname,
		//	classHookMapType& result);
		// Query registry for hooks matching the specified class and next selector
		//void DLEXPORT_rtmath_core queryClass(const char* classname,
		//	const char* topicID, topicHookSetType& result);

		template<class registryName, typename signature>
		class usesDLLregistry
		{
		public:
			typedef std::vector<const signature> hookStorageType;
		protected:
			usesDLLregistry() {}
			static void getHooks(hookStorageType& s) { s = hooks; }
		private:
			static hookStorageType hooks;
		public:
			virtual ~usesDLLregistry() {}
			static void registerHook(const signature &f) { hooks.push_back(f); }
		};
	}
}

extern "C"
{
	/// Provides interface for DLLs to register basic information about themselves
	bool DLEXPORT_rtmath_core rtmath_registry_register_dll(const rtmath::registry::DLLpreamble&);

	// Provides interface for DLLs to register a function hook
	//bool DLEXPORT_rtmath_core rtmath_registry_register_hook(const char* uuid, const char* topic);

	
}


