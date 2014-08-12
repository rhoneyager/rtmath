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
		template <class obj_class,
		class input_registry_class>
		class implementsStandardReader;
		template <class obj_class,
		class input_registry_class>
		class implementsStandardSingleReader;
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
		/** \brief Processes static options defined in add_options
		 * \todo Add processor for non-static options
		 * \note This gets automatically called by rtmath::debug::process_static_options
		 * \todo Hide this from apps
		 **/
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

		// Recursive and single-level DLL loading paths
		//extern DLEXPORT_rtmath_core std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;

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
			usesDLLregistry() {}
			template <class T, class U>
			friend class ::rtmath::io::implementsStandardWriter;
			template <class T, class U>
			friend class ::rtmath::io::implementsStandardReader;
			template <class T, class U>
			friend class ::rtmath::io::implementsStandardSingleReader;
			
		public:
			/// \note Implemented as a function-internal static function to avoid gcc template issue.
			static boost::shared_ptr<hookStorageType> getHooks()
			{
				static boost::shared_ptr<hookStorageType> hooks;
				if (!hooks) hooks = 
					boost::shared_ptr<hookStorageType>(new hookStorageType);
				return hooks;
			}
		//public:
			virtual ~usesDLLregistry() {}
			static void registerHook(const signature &f)
			{
				boost::shared_ptr<hookStorageType> hookstore = getHooks();
				hookstore->push_back(f); 
			}
		};

		/// Base class for external data access, throgh file I/O or database
		struct DLEXPORT_rtmath_core handler_external
		{
		protected:
			handler_external(const char* id);
			/// Ensures that plugins do not collide
			const char* id;
		public:
			inline const char* getId() { return id; }
			virtual ~handler_external() {}
		};

		/// Base class to handle multiple IO operations on a single file
		struct DLEXPORT_rtmath_core IOhandler : public handler_external
		{
		protected:
			IOhandler(const char* id);
		public:
			virtual ~IOhandler() {}
			/// If modifying these, change IO_options::setVal and getVal.
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

		/// Base class to handle database access
		struct DLEXPORT_rtmath_core DBhandler : public handler_external
		{
		protected:
			DBhandler(const char* id);
		public:
			virtual ~DBhandler() {}
			enum class DBtype
			{
				READONLY,
				NOUPDATE,
				NOINSERT,
				READWRITE
			};
		};
		
		/**
		* \param base is the raw pointer type (IOhandler or DBhandler)
		* \param derived is the pointer type being cast to / constructed
		* \param constructor is a function that creates a new object (shared_ptr<derived>).
		*		 Use std::bind and lambdas to feed it any necessary parameters.
		**/
		template<class base, class derived>
		std::shared_ptr<derived> construct_handle
			(const std::shared_ptr<base> sh, const char* id,
			const std::function<std::shared_ptr<derived>()> constructor)
		{
			std::shared_ptr<derived> h;
			if (!sh)
				h = std::shared_ptr<derived>(constructor());
			else {
				if (std::string(sh->getId()) != std::string(id)) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<derived>(sh);
			}
			return h;
		}

		/// \brief Convenient options specification class for use with an IO class registry.
		/// 
		/// Used because std::map doesn't like to go beyond template boundaries
		class DLEXPORT_rtmath_core options
		{
		protected:
			options();
			std::map<std::string, std::string> _mapStr;
		public:
			virtual ~options();
			void enumVals(std::ostream &out) const;
			static inline std::shared_ptr<options> generate() 
			{ auto res = std::shared_ptr<options>(new options); return res; }
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
				return getVal<T>(key);
			}
			template <class T>
			void setVal(const std::string &key, const T &value)
			{
				std::string valS = boost::lexical_cast<std::string>(value);
				_mapStr[key] = valS;
			}
			inline void setVal(const std::string &key, const IOhandler::IOtype val) { setVal<IOhandler::IOtype>(key, val); }

		};

		class DLEXPORT_rtmath_core IO_options : public options
		{
		private:
			IO_options();
		public:
			virtual ~IO_options();
			static inline std::shared_ptr<IO_options> generate(IOhandler::IOtype v = IOhandler::IOtype::TRUNCATE)
			{
				auto res = std::shared_ptr<IO_options>(new IO_options); res->iotype(v); return res;
			}
			
			// Some convenient definitions
			void filename(const std::string& val) { setVal<std::string>("filename", val); }
			std::string filename() const { return getVal<std::string>("filename", ""); }
			void extension(const std::string& val) { setVal<std::string>("extension", val); }
			std::string extension() const { return getVal<std::string>("extension", ""); }
			void filetype(const std::string &val) { setVal<std::string>("filetype", val); }
			std::string filetype() const { return getVal<std::string>("filetype", ""); }
			void exportType(const std::string &val) { setVal<std::string>("exportType", val); }
			std::string exportType() const { return getVal<std::string>("exportType", ""); }
			void iotype(IOhandler::IOtype val) { setVal<IOhandler::IOtype>("ioType", val); }
			IOhandler::IOtype iotype() const { return getVal<IOhandler::IOtype>("ioType", IOhandler::IOtype::TRUNCATE); }
		};

		class DLEXPORT_rtmath_core DB_options : public options
		{
		private:
			DB_options();
		public:
			virtual ~DB_options();
			static inline std::shared_ptr<DB_options> generate(DBhandler::DBtype v = DBhandler::DBtype::READWRITE)
			{
				auto res = std::shared_ptr<DB_options>(new DB_options); res->dbtype(v); return res;
			}

			// Some convenient definitions
			void username(const std::string& val) { setVal<std::string>("username", val); }
			std::string username() const { return getVal<std::string>("username", ""); }
			void password(const std::string& val) { setVal<std::string>("password", val); }
			std::string password() const { return getVal<std::string>("password", ""); }
			void hostname(const std::string &val) { setVal<std::string>("hostname", val); }
			std::string hostname() const { return getVal<std::string>("hostname", ""); }
			void dbname(const std::string &val) { setVal<std::string>("dbname", val); }
			std::string dbname() const { return getVal<std::string>("dbname", ""); }
			void sslmode(const std::string &val) { setVal<std::string>("sslmode", val); }
			std::string sslmode() const { return getVal<std::string>("sslmode", ""); }
			void dbtype(DBhandler::DBtype val) { setVal<DBhandler::DBtype>("dbType", val); }
			DBhandler::DBtype dbtype() const { return getVal<DBhandler::DBtype>("dbType", DBhandler::DBtype::READWRITE); }
		};

		/// Convenient template pattern for defining an IO class registry
		template<class object>
		struct IO_class_registry
		{
			virtual ~IO_class_registry() {}
			/** \brief If set, indicates that multiple IO operations are possible with this plugin.
			 * \param IO_options specifies the filename, file type, type of object to export, ...
			 * It catches everything because of the limitation in MSVC2012 regarding std::bind number of params.
			 * \param IOhandler is the plugin-provided opaque object that keeps track of 
			 * the state of the object being accessed.
			 **/
			typedef std::function<bool(std::shared_ptr<IOhandler>, std::shared_ptr<IO_options>
				)> io_multi_matcher_type;
			io_multi_matcher_type io_multi_matches;
		};

		template<class object>
		struct IO_class_registry_writer : IO_class_registry<object>
		{
			virtual ~IO_class_registry_writer() {}
			/** \brief Definition for an object that can handle multiple reads/writes.
			 * \param IOhandler is the plugin-provided opaque object that keeps track of 
			 * the state of the object being accessed.
			 * \param object* is a pointer to the object being read/written
			 * \param IO_options specifies the filename, file type, type of object to export, ...
			 * \returns Pointer to a IOhandler object (for example after the first write).
			 **/
			typedef std::function<std::shared_ptr<IOhandler>
				(std::shared_ptr<IOhandler>, std::shared_ptr<IO_options>, const object*)> io_multi_type;
			io_multi_type io_multi_processor;
		};

		template<class object>
		struct IO_class_registry_reader : IO_class_registry<object>
		{
			virtual ~IO_class_registry_reader() {}
			/** \brief Definition for an object that can handle multiple reads/writes.
			 * \param IOhandler is the plugin-provided opaque object that keeps track of 
			 * the state of the object being accessed.
			 * \param object* is a pointer to the object being read/written
			 * \param IO_options specifies the filename, file type, type of object to export, ...
			 * \returns Pointer to a IOhandler object (for example after the first read).
			 **/
			typedef std::function<std::shared_ptr<IOhandler>
				(std::shared_ptr<IOhandler>, std::shared_ptr<IO_options>, 
				object*)> io_multi_type;
			io_multi_type io_multi_processor;

			/** \brief Handles reading multiple objects from a single source
			 * \param IOhandler is the plugin-provided opaque object that keeps track of 
			 * the state of the object being accessed.
			 * \param std::vector<boost::shared_ptr<object> > & is a pointer to the object container.
			 * \param IO_options specifies the filename, file type, type of object to export, ...
			 * It also provides the ability to select objects from the source matching various criteria.
			 * Selection abilities vary based on the plugin.
			 * \returns Pointer to a IOhandler object (for example after the first read).
			 **/
			typedef std::function<std::shared_ptr<IOhandler>
				(std::shared_ptr<IOhandler>, std::shared_ptr<IO_options>, 
				std::vector<boost::shared_ptr<object> > &)> io_vector_type;
			io_vector_type io_vector_processor;
		};


		/** \brief Match file type (basic model)
		* \param filename is the file name
		* \param ext is the extension to match (provided by the plugin)
		* \param type is the file's extension (provided by the filename / saver)
		* \param op is the export operation (provided by the lib caller)
		* \param opref is the export operation to match (provided by the plugin)
		**/
		bool DLEXPORT_rtmath_core match_file_type(
			const char* filename, 
			const char* type, const char* ext, 
			const char* op = "", const char* opref = "");

		/// Matches
		bool DLEXPORT_rtmath_core match_file_type_multi(
			std::shared_ptr<rtmath::registry::IOhandler> h,
			const char* pluginid,
			std::shared_ptr<IO_options> opts,
			std::shared_ptr<IO_options> opts2);
	}
}

extern "C"
{
	/// Provides interface for DLLs to register basic information about themselves
	bool DLEXPORT_rtmath_core rtmath_registry_register_dll(const rtmath::registry::DLLpreamble&);
}

// GCC sure has odd attribute positioning rules...
namespace rtmath { namespace registry {
std::ostream DLEXPORT_rtmath_core & operator<<(std::ostream&, const ::rtmath::registry::IOhandler::IOtype&);
std::istream DLEXPORT_rtmath_core & operator>>(std::istream&, ::rtmath::registry::IOhandler::IOtype&);
std::ostream DLEXPORT_rtmath_core & operator<<(std::ostream&, const ::rtmath::registry::DBhandler::DBtype&);
std::istream DLEXPORT_rtmath_core & operator>>(std::istream&, ::rtmath::registry::DBhandler::DBtype&);

std::ostream DLEXPORT_rtmath_core & operator<<(std::ostream&, const ::rtmath::registry::options&);
} }

