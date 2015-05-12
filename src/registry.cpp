/**
* \brief Contains registry functions for extending functionality through DLLs.
*
* Contains both general and OS-specific functions.
**/

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/error.h"
#include "../Ryan_Debug/fs.h"
#include "../Ryan_Debug/config.h"
#include "../Ryan_Debug/splitSet.h"
#include "../Ryan_Debug/registry.h"
#include <boost/log/sources/global_logger_storage.hpp>

#ifdef _WIN32
#include "windows.h"
#endif
#ifdef __unix__
#include <dlfcn.h>
#endif

#ifdef _WIN32
typedef HINSTANCE dlHandleType;
#endif
#ifdef __APPLE__
typedef void* dlHandleType;
#endif
#ifdef __unix__
typedef void* dlHandleType;
#endif

//using namespace Ryan_Debug::debug;
namespace Ryan_Debug
{
	namespace registry
	{
		/// Recursive and single-level DLL loading paths
		std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;

		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_reg,
			blog::sources::severity_channel_logger_mt< >,
			(blog::keywords::severity = Ryan_Debug::log::error)(blog::keywords::channel = "registry"));

		void emit_registry_log(const std::string &m, ::Ryan_Debug::log::severity_level sev)
		{
			auto& lg = Ryan_Debug::registry::m_reg::get();
			BOOST_LOG_SEV(lg, sev) << m;
		}
	}
}

/// DLL information tables
namespace {
	using Ryan_Debug::registry::searchPathsOne;
	using Ryan_Debug::registry::searchPathsRecursive;

	class DLLhandle;
	/// Lists the paths of all loaded dlls
	std::set<std::string> DLLpathsLoaded;
	/// DLL information structure
	std::vector<Ryan_Debug::registry::DLLpreamble> preambles;
	/// Container for the handles of all loaded dlls
	std::vector<boost::shared_ptr<DLLhandle> > handles;

	
	//bool autoLoadDLLs = true;

	/// Checks if a file is a dll file
	bool isDynamic(const boost::filesystem::path &f)
	{
		using namespace boost::filesystem;
		std::string s = f.string(); // Faster, though less accurate
		if (s.find(".so") != std::string::npos) return true;
		if (s.find(".dll") != std::string::npos) return true;
		if (s.find(".dylib") != std::string::npos) return true;
		/*
		boost::filesystem::path p = f;
		while (p.has_extension())
		{
			path ext = p.extension();
			if (ext.string() == ".so" || ext.string() == ".dll"
				|| ext.string() == ".dylib") return true;
			p.replace_extension();
		}
		*/
		return false;
	};

	/// Checks if a dll file matches the build settings of the Ryan_Debug library, by file path
	bool correctVersionByName(const std::string &s)
	{
		std::string slower = s;
		std::transform(slower.begin(), slower.end(), slower.begin(), ::tolower);

		using namespace std;
		// The DLL case probably sould never occur.
#ifdef _DLL
		if (slower.find("static") != string::npos) return false;
#else
		if (slower.find("dynamic") != string::npos) return false;
#endif
		// Debug vs release dlls
		std::string buildtype; // defined in cmake config (addlib.cmake)

		// TODO: move these definitions to an appropriate header?
#define BUILDTYPE_Debug 1
#define BUILDTYPE_Release 2
#define BUILDTYPE_MinSizeRel 3
#define BUILDTYPE_RelWithDebInfo 4

#if BUILDTYPE == BUILDTYPE_Debug
		buildtype = "Debug";
#elif BUILDTYPE == BUILDTYPE_Release
		buildtype = "Release";
#elif BUILDTYPE == BUILDTYPE_MinSizeRel
		buildtype = "MinSizeRel";
#elif BUILDTYPE == BUILDTYPE_Release
		buildtype = "Release";
#else
		buildtype = BUILDCONF;
#endif
		std::transform(buildtype.begin(), buildtype.end(), buildtype.begin(), ::tolower);

		if (slower.find(buildtype) == string::npos)
		{
			if (slower.find("release") != string::npos) return false;
			if (slower.find("minsizerel") != string::npos) return false;
			if (slower.find("debug") != string::npos) return false;
			if (slower.find("relwithdebinfo") != string::npos) return false;
		}

		/// Check for x86 vs x64
#if __amd64 || _M_X64
		if (slower.find("x86") != string::npos) return false;
#else
		if (slower.find("x64") != string::npos) return false;
#endif
		/// \todo Check against windows system crt vs version-specific one
		/// \todo Figure out how to get crt lib name from loaded dll
		return true;
	}

	/**
	 * \brief Determines the search paths for dlls in the Ryan_Debug.conf file and in environment variables
	 *
	**/
	void constructSearchPaths(bool use_cmake = false, bool use_Ryan_Debug_conf = true, bool use_environment = true)
	{
		using std::vector;
		using std::set;
		using std::string; 
		auto& lg = Ryan_Debug::registry::m_reg::get();
		BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Constructing search paths" ;


		// Checking cmake pre-defined locations
#ifdef REGISTRY_PLUGINS_OVERRIDE_DIR
		{
			string regdir(REGISTRY_PLUGINS_OVERRIDE_DIR);
			if (regdir.size())
			{
				BOOST_LOG_SEV(lg, normal) << "REGISTRY_PLUGINS_OVERRIDE_DIR defined, so using: " << regdir;
				searchPathsRecursive.emplace(boost::filesystem::path(regdir));
			}
		}
#endif
		using namespace Ryan_Debug;
		boost::shared_ptr<const processInfo> info(getInfo(getPID()), freeProcessInfo);
	
		// Default locations
		// Install path apps

		//Ryan_Debug::registry::searchPathsRecursive.emplace(boost::filesystem::path("plugins"));
		//Ryan_Debug::registry::searchPathsRecursive.emplace(boost::filesystem::path("../plugins"));
		//searchPathsRecursive.emplace(boost::filesystem::path("../../plugins"));
		// Not in install path apps
		//Ryan_Debug::registry::searchPathsRecursive.emplace(boost::filesystem::path("../../plugins"));
		//Ryan_Debug::registry::searchPathsRecursive.emplace(boost::filesystem::path("../../../plugins"));
		//Ryan_Debug::registry::searchPathsRecursive.emplace(boost::filesystem::path("../../../../plugins"));

		// Relative to application
		// Install path apps
		boost::filesystem::path appBin(Ryan_Debug::getPath(info.get()));
		appBin.remove_filename();
		//Ryan_Debug::registry::searchPathsRecursive.emplace(appBin / "plugins");
		//Ryan_Debug::registry::searchPathsRecursive.emplace( appBin / "../plugins" );
		// Build path apps (linux)
		//Ryan_Debug::registry::searchPathsRecursive.emplace( appBin / "../../plugins" );

		// Relative to library
		using namespace Ryan_Debug;
		auto modinfo = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*) constructSearchPaths), freeModuleInfo);
		boost::filesystem::path libpath(getPath(modinfo.get()));
		libpath.remove_filename();
		Ryan_Debug::registry::searchPathsOne.emplace(libpath / "plugins");
		BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding library-relative path: " << libpath / "plugins";


		// Checking Ryan_Debug.conf
		if (use_Ryan_Debug_conf)
		{
			auto rtconf = Ryan_Debug::config::loadRtconfRoot();
			if (!rtconf) RDthrow(Ryan_Debug::error::xMissingRyan_DebugConf());
			string srecursivePaths, sonePaths;
			auto rtgeneral = rtconf->getChild("General");
			if (rtgeneral) {
				auto rtplugins = rtgeneral->getChild("Plugins");
				if (rtplugins) {
					rtplugins->getVal("Recursive", srecursivePaths);
					rtplugins->getVal("OneLevel", sonePaths);
				}
			}
			// Split loading paths based on semicolons and commas. Do not trim spaces.
			set<string> CrecursivePaths, ConePaths;
			Ryan_Debug::splitSet::splitSet(srecursivePaths, CrecursivePaths);
			Ryan_Debug::splitSet::splitSet(sonePaths, ConePaths);

			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding paths from Ryan_Debug config file";
			for (auto &p : CrecursivePaths)
			{
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding " << p;
				searchPathsRecursive.emplace(boost::filesystem::path(p));
			}
			for (auto &p : ConePaths)
			{
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding " << p;
				searchPathsOne.emplace(boost::filesystem::path(p));
			}
		}

		// Checking environment variables
		if (use_environment)
		{

			size_t sEnv = 0;
			const char* cenv = getEnviron(info.get(), sEnv);
			std::string env(cenv, sEnv);

			//Ryan_Debug::processInfo info = Ryan_Debug::getInfo(Ryan_Debug::getPID());
			std::map<std::string, std::string> mEnv;
			Ryan_Debug::splitSet::splitNullMap(env, mEnv);
			//std::vector<std::string> mCands;
			auto searchFunc = [](const std::pair<std::string, std::string> &pred, const std::string &mKey)
			{
				std::string key = pred.first;
				std::transform(key.begin(), key.end(), key.begin(), ::tolower);
				if (key == mKey) return true;
				return false;
			};
			auto searchEnviron = [&](const std::string &evar, std::set<boost::filesystem::path> &res)
			{
				auto it = std::find_if(mEnv.cbegin(), mEnv.cend(),
					std::bind(searchFunc, std::placeholders::_1, evar));
				if (it != mEnv.cend())
				{
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(",;");

					std::string ssubst;
					tokenizer tcom(it->second, sep);
					for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
					{
						using namespace boost::filesystem;
						path testEnv(it->second);
						if (exists(testEnv))
						{
							res.emplace(testEnv);
							BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding " << testEnv;
						}
					}
				}
			};

			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding paths from environment";
			searchEnviron("Ryan_Debug_plugins_DIR", searchPathsRecursive);
			searchEnviron("Ryan_Debug_dlls_recursive", searchPathsRecursive);
			searchEnviron("Ryan_Debug_dlls_onelevel", searchPathsOne);
		}
	}

	

	/** \brief Class that loads the DLL in an os-independent manner.
	*
	* Construction is allocation.
	* This class enables the safe closiong of dlHandle if registration 
	* fails to initialize properly. Previously, a dangling handle was
	* produced.
	**/
	class DLLhandle
	{
		Ryan_Debug::log::my_logger_mt m_dll;
	public:
		DLLhandle(const std::string &filename, bool critical = false) : dlHandle(nullptr), m_dll(blog::keywords::channel = "dll")
		{
			open(filename, critical);
		}
		DLLhandle() : dlHandle(nullptr), m_dll(blog::keywords::channel = "dll")
		{
		}
		void open(const std::string &filename, bool critical = false)
		{
			BOOST_LOG_SEV(m_dll, Ryan_Debug::log::normal) << "Loading dll " << filename << " with critical flag " << critical << ".";
			if (DLLpathsLoaded.count(filename))
			{
				BOOST_LOG_SEV(m_dll, Ryan_Debug::log::error) << "DLL is already loaded (" << filename << ", " << critical << ")!";
				RDthrow(Ryan_Debug::error::xDuplicateHook())
					<< Ryan_Debug::error::file_name_b(fname)
					<< Ryan_Debug::error::file_name(filename)
					<< Ryan_Debug::error::is_Critical(critical);
			}
			if (dlHandle)
			{
				BOOST_LOG_SEV(m_dll, Ryan_Debug::log::error) << "DLL handle is already in use (opening "
					<< filename << ") (already opened " << fname << ") (critical " << critical << ")!";
				RDthrow(Ryan_Debug::error::xHandleInUse())
					<< Ryan_Debug::error::file_name_b(fname.c_str())
					<< Ryan_Debug::error::file_name(filename)
					<< Ryan_Debug::error::is_Critical(critical);
			}
			fname = filename;

// ifdef _DEBUG
			// std::cerr << "Loading DLL " << filename << std::endl;
// endif
#ifdef __unix__ // Indicates that DLSYM is provided (unix, linux, mac, etc. (sometimes even windows))
			//Check that file exists here
			this->dlHandle = dlopen(filename.c_str(), RTLD_LAZY);
			const char* cerror = dlerror();
			if (cerror)
			{
				Ryan_Debug::debug::severity_level sl = (critical) ? Ryan_Debug::debug::critical : Ryan_Debug::debug::error;
				BOOST_LOG_SEV(m_dll, sl) << "dlopen error (opening " 
					<< filename << "): " << cerror;
				//std::string serror(cerror);
				RDthrow(Ryan_Debug::debug::xDLLerror())
					<< Ryan_Debug::debug::otherErrorText(std::string(cerror)) 
					<< Ryan_Debug::debug::file_name(filename) 
					<< Ryan_Debug::debug::is_Critical(critical);
			}
#endif
#ifdef _WIN32
			this->dlHandle = LoadLibrary(filename.c_str());
			// Could not open the dll for some reason
			if (this->dlHandle == NULL)
			{
				DWORD err = GetLastError();
				Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
				BOOST_LOG_SEV(m_dll, sl) << "LoadLibrary error (opening "
					<< filename << "): error code " << err << ".";
				if (critical)
					RDthrow(Ryan_Debug::error::xDLLerror())
						<< Ryan_Debug::error::otherErrorText("LoadLibrary") 
						<< Ryan_Debug::error::file_name(filename)
						<< Ryan_Debug::error::is_Critical(critical)
						<< Ryan_Debug::error::otherErrorCode(err);
			}
#endif
			DLLpathsLoaded.insert(filename);
			BOOST_LOG_SEV(m_dll, Ryan_Debug::log::normal) << "Loaded dll " << filename << " with critical flag " << critical << ".";
		}
		void close()
		{
			if (!dlHandle) return;
			//BOOST_LOG_SEV(m_dll, normal) << "Closing dll " << fname << "." << "\n";
			DLLpathsLoaded.erase(fname);
#ifdef __unix__
			dlclose(this->dlHandle);
#endif
#ifdef _WIN32
			FreeLibrary(this->dlHandle);
#endif
			//BOOST_LOG_SEV(m_dll, normal) << "Closed dll " << fname << "." << "\n";
		}
		~DLLhandle()
		{
			if (dlHandle) close();
		}
		void* getSym(const char* symbol)
		{
			BOOST_LOG_SEV(m_dll, Ryan_Debug::log::normal) << "Finding symbol " << symbol << " in dll " << fname << ".";
			if (dlHandle == NULL)
			{
				BOOST_LOG_SEV(m_dll, Ryan_Debug::log::error) << "DLL handle is closed when finding symbol " << symbol << " in dll " << fname << ".";
				RDthrow(Ryan_Debug::error::xHandleNotOpen())
					<< Ryan_Debug::error::file_name(fname);
			}
			void* sym;
#ifdef __unix__
			sym = dlsym(dlHandle, symbol);
#endif
#ifdef _WIN32
			sym = GetProcAddress(dlHandle, symbol);
#endif
			if (!sym)
			{
				BOOST_LOG_SEV(m_dll, Ryan_Debug::log::error) << "Cannot find symbol " << symbol << " in dll " << fname << ".";
				RDthrow(Ryan_Debug::error::xSymbolNotFound()) 
					<< Ryan_Debug::error::file_name(fname)
					<< Ryan_Debug::error::symbol_name(symbol);
			}
			return (void*)sym;
		}
	private:
		std::string fname;
		dlHandleType dlHandle;
	};
}

namespace Ryan_Debug
{
	namespace registry
	{
		bool findPath(std::set<boost::filesystem::path> &matches, const boost::filesystem::path &expr,
			const std::set<boost::filesystem::path> &searchPaths, bool recurse)
		{
			using namespace boost::filesystem;
			using namespace std;

			path pexpr(expr);

			for (const auto &p : searchPaths)
			{
				if (!exists(p)) continue;
				vector<path> recur;
				//recur.reserve(50000);
				if (!is_directory(p))
					recur.push_back(p);
				else {
					if (recurse)
						copy(recursive_directory_iterator(p, symlink_option::recurse),
						recursive_directory_iterator(), back_inserter(recur));
					else
						copy(directory_iterator(p),
						directory_iterator(), back_inserter(recur));
				}
				for (const auto &r : recur)
				{
					/// \todo Debug expression evaluation
					if (absolute(r, p) == pexpr) matches.emplace(r);
				}

			}

			if (matches.size()) return true;
			return false;
		}

		void searchDLLs(std::vector<std::string> &dlls, const std::set<boost::filesystem::path> &searchPaths, bool recurse)
		{
			using namespace boost::filesystem;
			using namespace std;

			auto& lg = m_reg::get();
			for (const auto &sbase : searchPaths)
			{
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Finding dlls in " << sbase << " with recursion=" << recurse << ".";
				path base(sbase);
				base = Ryan_Debug::fs::expandSymlink<path,path>(base);
				if (!exists(base)) continue;
				if (is_regular_file(base)) dlls.push_back(base.string());
				else if (is_directory(base))
				{
					vector<path> recur;
					//recur.reserve(50000);
					if (recurse)
						copy(recursive_directory_iterator(base, symlink_option::recurse),
						recursive_directory_iterator(), back_inserter(recur));
					else
						copy(directory_iterator(base),
						directory_iterator(), back_inserter(recur));
					for (const auto &p : recur)
					{
						if (!is_regular_file(p)) continue;
						// Convenient function to recursively check extensions to see if one is so or dll.
						// Used because of versioning.

						if (isDynamic(p))
						{
							// Check to see if build type is present in the path
							std::string slower = p.string();
							// Convert to lower case and do matching from there (now in func)
							//std::transform(slower.begin(), slower.end(), slower.begin(), ::tolower);

							if (correctVersionByName(slower))
								dlls.push_back(p.string());
						}
					}
				}
			}
		}

		void searchDLLs(std::vector<std::string> &dlls)
		{
			using namespace boost::filesystem;
			using namespace std;

			searchDLLs(dlls, searchPathsRecursive, true);
			searchDLLs(dlls, searchPathsOne, false);
		}

		void add_options(
			boost::program_options::options_description &cmdline,
			boost::program_options::options_description &config,
			boost::program_options::options_description &hidden)
		{
			namespace po = boost::program_options;
			using std::string;

			cmdline.add_options()
				;

			config.add_options()
				;

			hidden.add_options()
				("dll-load-onelevel", po::value<std::vector<std::string> >()->multitoken(),
				"Specify dlls to load. If passed a directory, it loads all dlls present (one-level). ")
				("dll-load-recursive", po::value<std::vector<std::string> >()->multitoken(),
				"Specify dlls to load. If passed a directory, it loads all dlls present (recursing). ")
				//("dll-no-default-locations", "Prevent non-command line dll locations from being read")
				("print-dll-loaded", "Prints the table of loaded DLLs.")
				("print-dll-search-paths", "Prints the search paths used when loading dlls.")
				;
		}

		void process_static_options(
			boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;

			auto& lg = m_reg::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Initializing registry system";

			//if (vm.count("dll-no-default-locations"))
			//	autoLoadDLLs = false;

			if (vm.count("dll-load-onelevel"))
			{
				std::vector<std::string> sPaths = vm["dll-load-onelevel"].as<std::vector<std::string> >();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll paths (1)";
				for (const auto s : sPaths)
				{
					searchPathsOne.emplace(s);
					BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll path (1): " << s;
				}
			}

			if (vm.count("dll-load-recursive"))
			{
				std::vector<std::string> sPaths = vm["dll-load-recursive"].as<std::vector<std::string> >();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll paths (1)";
				for (const auto s : sPaths)
				{
					searchPathsRecursive.emplace(s);
					BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll path (r): " << s;
				}
			}

			constructSearchPaths(false, true, true);

			if (vm.count("print-dll-search-paths"))
				printDLLsearchPaths(std::cerr);

			std::set<boost::filesystem::path> rPaths, oPaths;
			findPath(rPaths, boost::filesystem::path("default"), searchPathsRecursive, true);
			findPath(oPaths, boost::filesystem::path("default"), searchPathsOne, false);
			std::vector<std::string> toLoadDlls;
			// If a 'default' folder exists in the default search path, then use it for dlls.
			// If not, then use the base plugins directory.
			// Any library version / name detecting logic is in loadDLL (called by loadDLLs).

			if (rPaths.size() || oPaths.size())
			{
				searchDLLs(toLoadDlls, rPaths, true);
				searchDLLs(toLoadDlls, oPaths, false);
			}
			else { searchDLLs(toLoadDlls); }

			loadDLLs(toLoadDlls);
			

			if (vm.count("print-dll-loaded"))
				printDLLs();
		}

		void loadDLLs(const std::vector<std::string> &dlls)
		{
			for (const auto &dll : dlls)
				loadDLL(dll);
		}

		/// \todo Check for duplicate load
		/** \todo Check Ryan_Debug libraries loaded (core, ddscat, mie, ...). 
		 * Only load dlls that depend on a loaded Ryan_Debug lib, and ignore the rest.
		 **/
		void loadDLL(const std::string &filename)
		{
			auto& lg = m_reg::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loading DLL: " << filename;
			auto doLoad = [](const std::string &f)
			{
				boost::shared_ptr<DLLhandle> h(new DLLhandle(f));
				handles.push_back(h);
			};
			// Search for the dll
			using namespace boost::filesystem;
			path p(filename);
			//if (p.is_absolute())
			{
				if (exists(p)) doLoad(p.string());
				else {
					BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "DLL does not exist: " << filename;
					RDthrow(Ryan_Debug::error::xMissingFile())
						<< Ryan_Debug::error::otherErrorText("Cannot find dll to load")
						<< Ryan_Debug::error::file_name(p.string());
				}
			}
		}

		void printDLLs(std::ostream &out)
		{
			out << "Ryan_Debug DLL registry table:\n--------------------\n";
			for (const auto p : preambles)
			{
				out << "Name:\t\t\t" << p.name << "\n"
					<< "UUID:\t\t\t" << p.uuid << "\n"
					<< "Description:\t" << p.description << "\n";

				if (p.path)
					out << "Path:\t" << p.path << std::endl;
			}
			out << std::endl;
			out << "DLL paths loaded:\n----------------\n";
			for (const auto p : DLLpathsLoaded)
				out << p << "\n";
			out << std::endl;

			/*out << "\nHook Table:\n-----------------\nClass\tTopic\tPointer";
			for (const auto hm : hookRegistry)
			{
				out << hm.first << "\n";
				for (const auto h : hm.second)
				{
					out << "\t" << h.first << "\t" << h.second << "\n";
				}
			}
			*/

			out << std::endl;
		}

		void printDLLsearchPaths(std::ostream &out)
		{
			out << "Ryan_Debug DLL registry recursive search paths:\n--------------------\n";
			for (const auto p : searchPathsRecursive)
			{
				out << p.string() << "\n";
			}
			out << std::endl;
			out << "Ryan_Debug DLL registry one-level search paths:\n--------------------\n";
			for (const auto p : searchPathsOne)
			{
				out << p.string() << "\n";
			}
			out << std::endl;
		}

		handler_external::handler_external(const char* id) : id(id) {}
		options::options() {}
		options::~options() {}
		void options::enumVals(std::ostream &out) const
		{
			using namespace Ryan_Debug::registry;
			out << "Options definition:\n\tName\tValue" << std::endl;
			for (const auto &v : _mapStr)
			{
				if (v.first != "password")
				{
					out << "\t" << v.first << ":\t" << v.second << std::endl;
				}
				else {
					out << "\t" << v.first << ":\t" << "********" << std::endl;
				}
			}
		}
		IO_options::IO_options() {}
		IO_options::~IO_options() {}
		DB_options::DB_options() {}
		DB_options::~DB_options() {}

		IOhandler::IOhandler(const char* id) : handler_external(id) {}

		DBhandler::DBhandler(const char* id) : handler_external(id) {}
		
		bool match_file_type(const char* filename, const char* type, 
			const char* ext, const char *op, const char *opref)
		{
			using namespace boost::filesystem;
			using std::string;
			using std::ofstream;

			// Checking to see if a given export operation is supported.
			// For regular file saving, this falls through.
			string sop(op), sopref(opref);
			if (sop.compare(sopref) != 0) return false;

			// Actually comparing the type and file extension.
			string sext(ext);
			string sext2(".");
			sext2.append(sext);
			string stype(type);
			path pPrefix(filename);
			if (stype.compare(sext) == 0 || stype.compare(sext2) == 0) return true;

			else if (pPrefix.extension().string().compare(sext2) == 0) return true;
			return false;
		}

		bool match_file_type_multi(std::shared_ptr<Ryan_Debug::registry::IOhandler> h,
			const char* pluginid,
			std::shared_ptr<IO_options> opts,
			std::shared_ptr<IO_options> opts2)
		//bool match_file_type_multi(const char* filename, const char* type, 
		//	std::shared_ptr<Ryan_Debug::registry::IOhandler> h, const char* pluginid, const char* ext)
		{
			std::string spluginid(pluginid);
			if (h)
			{
				if (h->getId() != spluginid) return false;
				return true;
			} else {
				std::string filename = opts->filename();
				std::string ext = opts2->extension();
				std::string type = opts->filetype();
				return match_file_type(filename.c_str(), type.c_str(), ext.c_str(), 
					opts->exportType().c_str(), opts2->exportType().c_str());
			}
		}
	}
}

extern "C"
{
	bool Ryan_Debug_registry_register_dll(const Ryan_Debug::registry::DLLpreamble &p)
	{
		Ryan_Debug::registry::DLLpreamble b = p;

		auto& lg = Ryan_Debug::registry::m_reg::get();
		BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "DLL info for: " << p.path << "\n"
			<< "Name: " << p.name << "\n"
			<< "UUID: " << p.uuid << "\n"
			<< "Description: " << p.description << "\n";

		// Verify dll version of Ryan_Debug matches the current version!
		Ryan_Debug::versioning::versionInfo rdver;
		Ryan_Debug::versioning::genVersionInfo(rdver);
		auto match = Ryan_Debug::versioning::compareVersions(rdver, *(p.rdversion));
		if (match < Ryan_Debug::versioning::COMPATIBLE_2) {
			std::ostringstream mine, theirs;
			Ryan_Debug::versioning::debug_preamble(rdver, mine);
			Ryan_Debug::versioning::debug_preamble(*(p.rdversion), theirs);
			BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "Incompatible dll versions!\n"
				<< "Ryan_Debug library is: \n"
				<< mine.str()
				<< "Library at " << p.path << " uses version: \n"
				<< theirs.str();
		}

		// \todo Add in dll path to preamble!
		preambles.push_back(b);
		return true;
	}

}

namespace Ryan_Debug { namespace registry {

std::istream& operator>>(std::istream& in, ::Ryan_Debug::registry::IOhandler::IOtype& val)
{
	using namespace Ryan_Debug::registry;
	char data[50];
	in >> data;
	//in.getline(data,48);
	std::string v(data);
	if ("READONLY" == v) val = IOhandler::IOtype::READONLY;
	else if ("READWRITE" == v) val = IOhandler::IOtype::READWRITE;
	else if ("EXCLUSIVE" == v) val = IOhandler::IOtype::EXCLUSIVE;
	else if ("TRUNCATE" == v) val = IOhandler::IOtype::TRUNCATE;
	else if ("DEBUG" == v) val = IOhandler::IOtype::DEBUG;
	else if ("CREATE" == v) val = IOhandler::IOtype::CREATE;
	else RDthrow(::Ryan_Debug::error::xBadInput())
		<< ::Ryan_Debug::error::otherErrorText("Unlisted IOtype value")
		<< ::Ryan_Debug::error::symbol_name(v);
	return in;
}

std::ostream& operator<<(std::ostream &out, const ::Ryan_Debug::registry::IOhandler::IOtype& val)
{
	using namespace Ryan_Debug::registry;
	std::string v;
	if (val == IOhandler::IOtype::READONLY) v = "READONLY";
	else if (val == IOhandler::IOtype::READWRITE) v = "READWRITE";
	else if (val == IOhandler::IOtype::EXCLUSIVE) v = "EXCLUSIVE";
	else if (val == IOhandler::IOtype::TRUNCATE) v = "TRUNCATE";
	else if (val == IOhandler::IOtype::DEBUG) v = "DEBUG";
	else if (val == IOhandler::IOtype::CREATE) v = "CREATE";
	else RDthrow(::Ryan_Debug::error::xBadInput())
		<< ::Ryan_Debug::error::otherErrorText("Unlisted IOtype value");
	out << v;
	return out;
}

std::istream& operator>>(std::istream& in, ::Ryan_Debug::registry::DBhandler::DBtype& val)
{
	using namespace Ryan_Debug::registry;
	char data[50];
	in >> data;
	//in.getline(data,48);
	std::string v(data);
	if ("READONLY" == v) val = DBhandler::DBtype::READONLY;
	else if ("READWRITE" == v) val = DBhandler::DBtype::READWRITE;
	else if ("NOUPDATE" == v) val = DBhandler::DBtype::NOUPDATE;
	else if ("NOINSERT" == v) val = DBhandler::DBtype::NOINSERT;
	else RDthrow(::Ryan_Debug::error::xBadInput())
		<< ::Ryan_Debug::error::otherErrorText("Unlisted DBtype value") << ::Ryan_Debug::error::symbol_name(v);
	return in;
}

std::ostream& operator<<(std::ostream &out, const ::Ryan_Debug::registry::DBhandler::DBtype& val)
{
	using namespace Ryan_Debug::registry;
	std::string v;
	if (val == DBhandler::DBtype::READONLY) v = "READONLY";
	else if (val == DBhandler::DBtype::READWRITE) v = "READWRITE";
	else if (val == DBhandler::DBtype::NOUPDATE) v = "NOUPDATE";
	else if (val == DBhandler::DBtype::NOINSERT) v = "NOINSERT";
	else RDthrow(::Ryan_Debug::error::xBadInput())
		<< ::Ryan_Debug::error::otherErrorText("Unlisted DBtype value");
	out << v;
	return out;
}

std::ostream& operator<<(std::ostream &out, const ::Ryan_Debug::registry::options& val)
{
	val.enumVals(out);
	return out;
}

} }

