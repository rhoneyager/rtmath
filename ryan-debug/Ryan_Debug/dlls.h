/* registry.h - DLL registration system for extended functionality */
#pragma once
#include "defs.h"

#pragma warning( disable : 4251 ) // DLL interface
#pragma warning( disable : 4661 ) // Exporting vector

#include <iostream>
#include <set>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include "info.h"

namespace boost {
	namespace program_options {
		class options_description; class variables_map;
	}
	namespace filesystem { class path; }
}


namespace Ryan_Debug
{
	namespace registry
	{
		/** \brief This is the basic structure passed by an Ryan_Debug DLL to provide
		* identity information.
		**/
		struct DLLpreamble
		{
			/// Short DLL name
			const char* name;
			/// Short description
			const char* description;
			/// UUID that prevents different versions or copies   
			/// of the same DLL from being loaded simultaneously.
			const char* uuid;
			/// Path of the loaded module (DLL leaves it blank)
			//const char* path;
			DLLpreamble(const char* name, const char* desc, const char* uuid)
				: name(name), description(desc), uuid(uuid) {} //, path(0) {}
			DLLpreamble() : name(0), description(0), uuid(0) {} //, path(0) {}
		};

		class DLLhandleImpl;
		class DLLhandle;

		/// Validator for DLLs
		class RYAN_DEBUG_DLEXPORT dllValidator {
		protected:
			dllValidator();
		private:
		public:
			~dllValidator();
			virtual const char* validationSymbol() const = 0;
			virtual bool validate(void* func, bool critical = false) const = 0;
			static boost::shared_ptr<const dllValidator> genDefaultValidator();
		};

		/**
		* custom validator design:
		* validator searches for a given function name only, and then passes the
		* code to the custom validator, which returns a bool.
		* If false, it fails the validation check (validator handles any throws and 
		* logging). If true, then it passes and moves on to the next validator or 
		* completes its load. Only after all validators pass is the dllStart 
		* routine called (the one that is passed by Ryan_Debug).
		**/
		class RYAN_DEBUG_DLEXPORT dllValidatorSet {
		private:
			std::vector<boost::shared_ptr<const dllValidator> > validators;
		protected:
			dllValidatorSet();
		public:
			void append(boost::shared_ptr<const dllValidator>);
			~dllValidatorSet();
			static boost::shared_ptr<dllValidatorSet> generate();
			static boost::shared_ptr<const dllValidatorSet> getDefault();
			bool validate(const DLLhandle*, bool critical = false) const;
		};

		/** \brief Class that loads the DLL in an os-independent manner.
		*
		* Construction is allocation.
		* This class enables the safe closiong of dlHandle if registration
		* fails to initialize properly. Previously, a dangling handle was
		* produced.
		**/
		class RYAN_DEBUG_DLEXPORT DLLhandle
		{
		public:
			DLLhandle(const std::string &filename, 
				boost::shared_ptr<const Ryan_Debug::registry::dllValidatorSet>
				= Ryan_Debug::registry::dllValidatorSet::getDefault(), 
				bool critical = false);
			DLLhandle();
			void open(const std::string &filename, bool critical = false);
			void close();
			~DLLhandle();
			void* getSym(const char* symbol, bool critical = false) const;
			const char* filename() const;
			bool isOpen() const;
		private:
			boost::shared_ptr<DLLhandleImpl> _p;
		};


		/**
		* \brief Adds options to a program
		*
		* \item cmdline provides options only allowed on the command line
		* \item config provides options available on the command line and in a config file
		* \item hidden provides options allowed anywhere, but are not displayed to the user
		*
		* \note This gets automatically called by Ryan_Debug::debug::add_options
		* \todo Hide this
		**/
		void RYAN_DEBUG_DLEXPORT add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/** \brief Processes static options defined in add_options
		* \todo Add processor for non-static options
		* \note This gets automatically called by Ryan_Debug::debug::process_static_options
		* \todo Hide this from apps
		**/
		void RYAN_DEBUG_DLEXPORT process_static_options(
			boost::program_options::variables_map &vm);


		/// Load a DLL.
		void RYAN_DEBUG_DLEXPORT loadDLL(const std::string &filename, boost::shared_ptr<const dllValidatorSet> = dllValidatorSet::getDefault(), bool critical = false);
		/// Load DLLs.
		void RYAN_DEBUG_DLEXPORT loadDLLs(const std::vector<std::string> &dlls, boost::shared_ptr<const dllValidatorSet> = dllValidatorSet::getDefault(), bool critical = false);

		/// Print loaded DLLs
		void RYAN_DEBUG_DLEXPORT printDLLs(std::ostream &out = std::cerr);

		/// List DLL search paths
		void RYAN_DEBUG_DLEXPORT printDLLsearchPaths(std::ostream &out = std::cerr);

		/// Find all occurances of a subpath in a search path.
		bool RYAN_DEBUG_DLEXPORT findPath(std::set<boost::filesystem::path> &matches,
			const boost::filesystem::path &expr,
			const std::set<boost::filesystem::path> &searchPaths, bool recurse);

		/**
		* \brief Locates all DLLs in the search path and loads them.
		*
		* The search path may be specified / manipulated from several locations:
		* - precompiled hints (from cmake)
		* - Ryan_Debug.conf
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
		* \see Ryan_Debug::registry::process_static_options
		**/
		void RYAN_DEBUG_DLEXPORT searchDLLs(std::vector<std::string> &dlls);
		void RYAN_DEBUG_DLEXPORT searchDLLs(std::vector<std::string> &dlls,
			const std::set<boost::filesystem::path> &searchPaths, bool recurse);

		void RYAN_DEBUG_DLEXPORT add_hook_table(const char* tempsig, void* store);
		void RYAN_DEBUG_DLEXPORT dump_hook_table(std::ostream &out = std::cerr);


	}
}


extern "C"
{
	enum dllInitResult {
		SUCCESS,
		DUPLICATE_DLL,
		OTHER_FAILURE
	};

	/// Provides interface for DLLs to register basic information about themselves
	dllInitResult RYAN_DEBUG_DLEXPORT Ryan_Debug_registry_register_dll(
		const Ryan_Debug::registry::DLLpreamble&,
		void* funcInDllForPath);

}
