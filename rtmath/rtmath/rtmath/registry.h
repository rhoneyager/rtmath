/* registry.h - DLL registration system for extended functionality */
#pragma once
#include "defs.h"

#pragma warning( disable : 4661 ) // Exporting vector

#include <functional>
#include <iostream>
#include <map>
//#include <list>
#include <set>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include "error/error.h"

namespace boost {
	namespace program_options {
		class options_description; class variables_map;
	}
	namespace filesystem { class path; }
}

namespace rtmath
{
	namespace io
	{
		template <class obj_class,
		class output_registry_class>
		class implementsStandardWriter;
	}
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
		*
		* \note This gets automatically called by rtmath::debug::add_options
		* \todo Hide this
		**/
		void DLEXPORT_rtmath_core add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden);
		/// Processes static options defined in add_options
		/// \todo Add processor for non-static options
		/// \note This gets automatically called by rtmath::debug::process_static_options
		/// \todo Hide this
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
		//extern DLEXPORT_rtmath_core std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;

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

		/**
		* \brief Template base class that provides a DLL hook registry for a class.
		*
		* The registry is implemented on a per-class basis because different classes 
		* have different requirements for the DLL registry. Some classes will have both 
		* reading and writing functions. Each type of function that is registered may 
		* have a different function signature.
		*
		* \param signature is the type that gets registered by the DLLs
		* \param registryName is a dummy class to provide multiple registries for the same derived class.
		**/
		template<class registryName, typename signature>
		class usesDLLregistry
		{
		public:
			typedef typename std::vector<signature> hookStorageType;
		protected:
			template <class T, class U>
			friend class ::rtmath::io::implementsStandardWriter;
			usesDLLregistry() {}
			/// \note Implemented as a function-internal static function to avoid gcc template issue.
			static boost::shared_ptr<hookStorageType> getHooks()
			{
				static boost::shared_ptr<hookStorageType> hooks;
				if (!hooks) hooks = 
					boost::shared_ptr<hookStorageType>(new hookStorageType);
				return hooks;
			}
		public:
			virtual ~usesDLLregistry() {}
			static void registerHook(const signature &f) { getHooks()->push_back(f); }
		};


		/// Base class to handle multiple IO operations on a single file
		struct IOhandler
		{
		protected:
			IOhandler(const std::string &id) : id(id) {}
			/// Ensures that plugins do not collide
			std::string id;
		public:
			inline std::string getId() { return id; }
			virtual ~IOhandler() { }
			enum class IOtype
			{
				READONLY,
				READWRITE,
				EXCLUSIVE,
				TRUNCATE,
				DEBUG,
				CREATE
			};
		};

		/// \brief Convenient options specification class for use with an IO class registry.
		/// 
		/// Used because std::map doesn't like to go beyond template boundaries
		class options
		{
		private:
			options () {}
			std::map<std::string, std::string> _mapStr;
		public:
			virtual ~options() {}
			static inline std::shared_ptr<options> generate() {return std::shared_ptr<options>(new options);}
			inline bool hasVal(const std::string &key) const
			{
				if (_mapStr.count(key)) return true;
				return false;
			}
			template <class T> T getVal(const std::string &key) const
			{
				if (!hasVal(key)) RTthrow rtmath::debug::xArrayOutOfBounds();
				std::string valS = _mapStr.at(key);
				T res = boost::lexical_cast<T>(valS);
				return res;
			}
			template <class T> T getVal(const std::string &key, const T& defaultval) const
			{
				if (!hasVal(key)) return defaultval;
				return getVal(key);
			}
			template <class T>
			void setVal(const std::string &key, const T &value)
			{
				std::string valS = boost::lexical_cast<std::string>(value);
				_mapStr[key] = valS;
			}
		};

		/// Convenient template pattern for defining an IO class registry
		template<class object>
		struct IO_class_registry
		{
			// Matcher takes two parameters, the output filename and an optional 'type'
			typedef std::function<bool(const char*, const char*)> io_matcher_type;
			typedef std::function<void(const char*, const object*)> io_processor_type;
			/// Determines if a file can be read / written with this registration
			io_matcher_type io_matches;
			
			/// Handler function for the actual IO operation
			io_processor_type io_processor;

			/// If set, indicates that multiple IO operations are possible with this plugin
			typedef std::function<bool(const char*, const char*, std::shared_ptr<IOhandler>)> io_multi_matcher_type;
			io_multi_matcher_type io_multi_matches;
			/// \brief Definition for an object that can handle multiple reads/writes.
			/// \param IOhandler is the plugin-provided opaque object that keeps track of 
			/// the state of the object being accessed.
			/// \param First const char* is a filename / access string
			/// \param object* is a pointer to the object being read/written
			/// \param Second const char* is the object 'key'
			/// \returns Pointer to a IOhandler object (for example after the first write).
			typedef std::function<std::shared_ptr<IOhandler>
				(std::shared_ptr<IOhandler>, const char*, const object*, const char*, 
				IOhandler::IOtype)> io_multi_type;
			io_multi_type io_multi_processor;
		};


		/// Match file type - use bind to bind the 3rd parameter to the desired extension
		bool DLEXPORT_rtmath_core match_file_type(const char* filename, const char* type, const char* ext);

		bool DLEXPORT_rtmath_core match_file_type_multi(const char* filename, const char* type, 
			std::shared_ptr<rtmath::registry::IOhandler> h, const char* pluginid, const char* ext);
	}
}

extern "C"
{
	/// Provides interface for DLLs to register basic information about themselves
	bool DLEXPORT_rtmath_core rtmath_registry_register_dll(const rtmath::registry::DLLpreamble&);

	// Provides interface for DLLs to register a function hook
	//bool DLEXPORT_rtmath_core rtmath_registry_register_hook(const char* uuid, const char* topic);

	
}


