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
#include "../Ryan_Debug/logging.h"
#include "../Ryan_Debug/debug.h"
#include "../Ryan_Debug/error.h"
#include "../Ryan_Debug/fs.h"
#include "../Ryan_Debug/config.h"
#include "../Ryan_Debug/splitSet.h"
#include "../Ryan_Debug/dlls.h"
#include "../Ryan_Debug/info.h"
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

namespace Ryan_Debug {
	namespace registry {
		/// Recursive and single-level DLL loading paths
		std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;


		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_reg,
			boost::log::sources::severity_channel_logger_mt< >,
			(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "registry"));

	}
}

namespace {
	std::map<void*, std::string> hookTable;
	/// Lists the paths of all loaded dlls
	std::set<std::string> DLLpathsLoaded;
	/// DLL information structure
	std::vector<Ryan_Debug::registry::DLLpreamble> preambles;
	/// Container for the handles of all loaded dlls
	std::vector<boost::shared_ptr<Ryan_Debug::registry::DLLhandle> > handles;


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
		BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Constructing search paths";


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
		auto modinfo = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*)constructSearchPaths), freeModuleInfo);
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
				Ryan_Debug::registry::searchPathsRecursive.emplace(boost::filesystem::path(p));
			}
			for (auto &p : ConePaths)
			{
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding " << p;
				Ryan_Debug::registry::searchPathsOne.emplace(boost::filesystem::path(p));
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
			searchEnviron("Ryan_Debug_plugins_DIR", Ryan_Debug::registry::searchPathsRecursive);
			searchEnviron("Ryan_Debug_dlls_recursive", Ryan_Debug::registry::searchPathsRecursive);
			searchEnviron("Ryan_Debug_dlls_onelevel", Ryan_Debug::registry::searchPathsOne);
		}
	}

}

//using namespace Ryan_Debug::debug;
namespace Ryan_Debug
{
	namespace registry
	{
		
		void emit_registry_log(const std::string &m, ::Ryan_Debug::log::severity_level sev)
		{
			auto& lg = Ryan_Debug::registry::m_reg::get();
			BOOST_LOG_SEV(lg, sev) << m;
		}

		void add_hook_table(const char* tempsig, void* store) {
			hookTable[store] = std::string(tempsig);
		}

		void dump_hook_table(std::ostream &out) {
			auto h = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*) dump_hook_table), freeModuleInfo);
			out << "Hook table for Ryan_Debug dll at "
				<< getPath(h.get()) << std::endl
				<< "Store\t - \tSignature\n";
			for (const auto &i : hookTable)
			{
				out << i.first << "\t - \t" << i.second << std::endl;
			}
		}

		using Ryan_Debug::registry::searchPathsOne;
		using Ryan_Debug::registry::searchPathsRecursive;

		class DLLhandle;

		class DLLhandleImpl
		{
			friend class DLLhandle;
			DLLhandle *parent;
			std::string fname;
			dlHandleType dlHandle;
			boost::shared_ptr<const Ryan_Debug::registry::dllValidatorSet> validators;
			DLLhandleImpl(boost::shared_ptr<const Ryan_Debug::registry::dllValidatorSet> dvs,
				DLLhandle *p)
				: dlHandle(nullptr), validators(dvs), parent(p) {
			}
			void close() {
				if (!dlHandle) return;
				//BOOST_LOG_SEV(m_reg, normal) << "Closing dll " << fname << "." << "\n";
				DLLpathsLoaded.erase(fname);
#ifdef __unix__
				dlclose(this->dlHandle);
#endif
#ifdef _WIN32
				FreeLibrary(this->dlHandle);
#endif
				//BOOST_LOG_SEV(m_reg, normal) << "Closed dll " << fname << "." << "\n";
			}
			void* getSym(const char* symbol, bool critical = false) const 
			{
				auto& lg = m_reg::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Finding symbol " << symbol << " in dll " << fname << ".";
				if (dlHandle == NULL)
				{
					BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "DLL handle is closed when finding symbol " << symbol << " in dll " << fname << ".";
					RDthrow(Ryan_Debug::error::xHandleNotOpen())
						<< Ryan_Debug::error::file_name(fname);
				}
				void* sym = nullptr;
#ifdef __unix__
				sym = dlsym(dlHandle, symbol);
#endif
#ifdef _WIN32
				sym = GetProcAddress(dlHandle, symbol);
#endif
				if (!sym)
				{
#ifdef _WIN32
					long long errcode = 0;
					errcode = (long long)GetLastError();
#endif
					BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "Cannot find symbol " << symbol << " in dll " << fname << ".";
					if (critical)
						RDthrow(Ryan_Debug::error::xSymbolNotFound())
						<< Ryan_Debug::error::file_name(fname)
						<< Ryan_Debug::error::symbol_name(symbol)
#ifdef _WIN32
						<< Ryan_Debug::error::otherErrorCode(errcode)
#endif
						;
				}
				return (void*)sym;
			}
			void open(const std::string &filename, bool critical = false)
			{
				auto& lg = m_reg::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loading dll " << filename << " with critical flag " << critical << ".";
				if (DLLpathsLoaded.count(filename))
				{
					BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "DLL is already loaded (" << filename << ", " << critical << ")!";
					RDthrow(Ryan_Debug::error::xDuplicateHook())
						<< Ryan_Debug::error::file_name_b(fname)
						<< Ryan_Debug::error::file_name(filename)
						<< Ryan_Debug::error::is_Critical(critical);
				}
				if (dlHandle)
				{
					BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "DLL handle is already in use (opening "
						<< filename << ") (already opened " << fname << ") (critical " << critical << ")!";
					RDthrow(Ryan_Debug::error::xHandleInUse())
						<< Ryan_Debug::error::file_name_b(fname.c_str())
						<< Ryan_Debug::error::file_name(filename)
						<< Ryan_Debug::error::is_Critical(critical);
				}
				fname = filename;

				auto attemptLoad = [&]() -> bool {

#ifdef __unix__ // Indicates that DLSYM is provided (unix, linux, mac, etc. (sometimes even windows))
					//Check that file exists here
					this->dlHandle = dlopen(filename.c_str(), RTLD_LAZY);
					const char* cerror = dlerror();
					if (cerror)
					{
						Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
						BOOST_LOG_SEV(lg, sl) << "dlopen error (opening "
							<< filename << "): " << cerror;
						//std::string serror(cerror);
						RDthrow(Ryan_Debug::error::xDLLerror())
							<< Ryan_Debug::error::otherErrorText(std::string(cerror))
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical);
						return false;
					}
#endif
#ifdef _WIN32
					this->dlHandle = LoadLibrary(filename.c_str());
					// Could not open the dll for some reason
					if (this->dlHandle == NULL)
					{
						DWORD err = GetLastError();
						Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
						BOOST_LOG_SEV(lg, sl) << "LoadLibrary error (opening "
							<< filename << "): error code " << err << ".";
						if (critical)
							RDthrow(Ryan_Debug::error::xDLLerror())
							<< Ryan_Debug::error::otherErrorText("LoadLibrary")
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical)
							<< Ryan_Debug::error::otherErrorCode(err);
						return false;
					}
#endif
					return true;
				};
				bool res = attemptLoad();
				if (!res) return;

				DLLpathsLoaded.insert(filename);

				bool res2 = validators->validate(parent, critical);

				if (!res2) {
					close();
					BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "Failed to load dll " << filename << " with critical flag " << critical << ".";
					return;
				}

				// Finally, get and invoke the initialization function

				dllInitResult(*vfStart)() = (dllInitResult(*)()) getSym("dllStart", critical);

				if (vfStart) {
					// It's a C function, so it does not raise exceptions. Might not pass exceptions either.
					dllInitResult res;
					res = vfStart();

					if (DUPLICATE_DLL == res) {
						Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
						BOOST_LOG_SEV(lg, sl) << "Unable to load dll " << filename << " with critical flag " << critical << "."
							<< " Duplicate dll already loaded.";
						if (critical)
							RDthrow(Ryan_Debug::error::xDuplicateHook())
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical);
						close();
						return;
					}
					else if (SUCCESS != res) {
						Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
						BOOST_LOG_SEV(lg, sl) << "Unable to load dll " << filename << " with critical flag " << critical << "."
							<< " dllStart did not return success. It returned " << res << ".";
						if (critical)
							RDthrow(Ryan_Debug::error::xDLLerror())
							<< Ryan_Debug::error::symbol_name("dllStart")
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical)
							<< Ryan_Debug::error::otherErrorCode(res);
						close();
						return;
					}
					//} catch (Ryan_Debug::error::xDuplicateHook &e) {
					// Already logged duplicate detected message.
					//	if (critical)
					//		throw e << Ryan_Debug::error::is_Critical(critical);
					//	return false;
					//}
				}
				else {
					// Will not reach this point if critical flag is set.
					Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
					BOOST_LOG_SEV(lg, sl) << "Unable to load dll " << filename << " with critical flag " << critical << "."
						<< " Missing dllStart function.";
					if (critical)
						RDthrow(Ryan_Debug::error::xSymbolNotFound())
						<< Ryan_Debug::error::symbol_name("dllStart")
						<< Ryan_Debug::error::file_name(filename)
						<< Ryan_Debug::error::is_Critical(critical)
						<< Ryan_Debug::error::otherErrorCode(res);
					close();
					return;
				}
				

				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loaded dll " << filename << " with critical flag " << critical << ".";
			}
		public:
			~DLLhandleImpl() {
				close();
			}
		};

		DLLhandle::DLLhandle(const std::string &filename, 
			boost::shared_ptr<const Ryan_Debug::registry::dllValidatorSet> dvs,
			bool critical)
		{
			_p = boost::shared_ptr<DLLhandleImpl>(new DLLhandleImpl(dvs, this));
			open(filename, critical);
		}
		DLLhandle::DLLhandle()
		{
			_p = boost::shared_ptr<DLLhandleImpl>(new DLLhandleImpl(
				Ryan_Debug::registry::dllValidatorSet::getDefault(), this));
		}
		void DLLhandle::open(const std::string &filename, bool critical)
		{ _p->open(filename, critical); }
		void DLLhandle::close() { _p->close(); }
		DLLhandle::~DLLhandle() {}
		void* DLLhandle::getSym(const char* symbol, bool critical) const
		{ return _p->getSym(symbol, critical); }


		dllValidator::dllValidator() {}
		dllValidator::~dllValidator() {}
		dllValidatorSet::dllValidatorSet() {}
		dllValidatorSet::~dllValidatorSet() {}

		
		
		class dllValidatorRyanDebug : public dllValidator {
		public:
			dllValidatorRyanDebug() {}
			~dllValidatorRyanDebug() {}
			virtual const char* validationSymbol() const { const char* r = "dlVer"; return r; }
			virtual bool validate(void* func, bool critical) const
			{
				auto& lg = m_reg::get();
				Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
				void(*fVer)(Ryan_Debug::versioning::versionInfo&, void**) =
					(void(*)(Ryan_Debug::versioning::versionInfo&, void**)) func;
				auto h = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*) func), freeModuleInfo);
				std::string filename(getPath(h.get()));
				h.reset();
				//dllInitResult(*vfStart)() = nullptr;
				//void* vfStarta = nullptr;
				if (fVer) {
					using namespace Ryan_Debug::versioning;
					versionInfo dllVer, myVer;
					void* rdcheck = nullptr; // Check to make sure the same Ryan_Debug functions are being used.
					getLibVersionInfo(myVer);
					fVer(dllVer, &rdcheck);
					auto verres = compareVersions(dllVer, myVer);
					std::ostringstream dver, mver;
					debug_preamble(dllVer, dver);
					debug_preamble(myVer, mver);
					std::string sdver = dver.str(), smver = mver.str();
					if (verres < COMPATIBLE_2) {
						BOOST_LOG_SEV(lg, sl) << "Dll " << filename << " with critical flag " << critical
							<< " references an incompatible Ryan_Debug version!" << std::endl
							<< "DLL version: \n" << sdver << std::endl
							<< "My version: \n" << smver << std::endl;
						if (critical)
							RDthrow(Ryan_Debug::error::xDLLversionMismatch())
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical);
						return false;
					}
					else if (verres != EXACT_MATCH) {
						BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Dll " << filename << " with critical flag " << critical
							<< " references a slightly different Ryan_Debug version!" << std::endl
							<< "DLL version: \n" << sdver << std::endl
							<< "My version: \n" << smver << std::endl;
					}

					// Check that the DLL's Ryan_Debug function calls are really to the correct code.
					void *mdcheck = (void*) &(Ryan_Debug_registry_register_dll);
					if (rdcheck != mdcheck) {
						using namespace Ryan_Debug::registry;
						using namespace Ryan_Debug;
						auto h = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*)mdcheck), freeModuleInfo);
						std::string myPath(getPath(h.get()));
						auto ho = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*)rdcheck), freeModuleInfo);
						std::string rPath;
						if (rdcheck) rPath = (getPath(ho.get())); else rPath = "Unknown";
						h.reset();
						ho.reset();

						BOOST_LOG_SEV(lg, sl) << "Dll " << filename << " with critical flag " << critical
							<< " is loading the wrong Ryan_Debug version!" << std::endl
							<< "DLL's Ryan_Debug path: \n" << rPath << std::endl
							<< "My Ryan_Debug path: \n" << myPath << std::endl;
						if (critical)
							RDthrow(Ryan_Debug::error::xDLLversionMismatch())
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical);
						return false;
					}
				}
				else {
					BOOST_LOG_SEV(lg, sl) << "Dll " << filename << " with critical flag " << critical
						<< " is missing Ryan_Debug version information!" << std::endl;
					if (critical)
						RDthrow(Ryan_Debug::error::xDLLversionMismatch())
						<< Ryan_Debug::error::file_name(filename)
						<< Ryan_Debug::error::is_Critical(critical);
					return false;
				}
				return true;
			}
		};

		boost::shared_ptr<const dllValidator> dllValidator::genDefaultValidator() {
			static boost::shared_ptr<const dllValidator> def(new dllValidatorRyanDebug());
			return def;
		}

		boost::shared_ptr<dllValidatorSet> dllValidatorSet::generate() {
			return boost::shared_ptr<dllValidatorSet>(new dllValidatorSet);
		}
		boost::shared_ptr<const dllValidatorSet> dllValidatorSet::getDefault()
		{
			static bool gen = false;
			static boost::shared_ptr<const dllValidatorSet> def;
			if (!gen)
			{
				boost::shared_ptr<dllValidatorSet> a = generate();
				a->append(dllValidator::genDefaultValidator());
				def = a;
			}
			return def;
		}
		void dllValidatorSet::append(boost::shared_ptr<const dllValidator> v) {
			validators.push_back(v);
		}
		bool dllValidatorSet::validate(const DLLhandle* p, bool critical) const {
			for (const auto &i : validators) {
				// First, get the dll symbol function
				const char* sym = i->validationSymbol();
				// Get the address of the function
				void* func = p->getSym(sym, critical);

				if (func) {
					bool res = false;
					res = i->validate(func, critical);
					if (!res) return false;
				} else return false;
			}
			return true;
		}

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
				size_t sDlls = dlls.size();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Finding dlls in " << sbase << " with recursion=" << recurse << ".";
				path base(sbase);
				base = Ryan_Debug::fs::expandSymlink<path, path>(base);
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

							if (correctVersionByName(slower)) {
								BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Found candidate " << p.string();
								dlls.push_back(p.string());
							}
						}
					}
				}
				size_t eDlls = dlls.size();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Found " << eDlls - sDlls << " potential dlls in " << sbase << " with recursion=" << recurse << ".";
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

		void loadDLLs(const std::vector<std::string> &dlls, boost::shared_ptr<const dllValidatorSet> dvs)
		{
			for (const auto &dll : dlls)
				loadDLL(dll, dvs);
		}

		void loadDLL(const std::string &filename, boost::shared_ptr<const dllValidatorSet> dvs)
		{
			auto& lg = m_reg::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loading DLL: " << filename;
			auto doLoad = [&](const std::string &f)
			{
				boost::shared_ptr<DLLhandle> h(new DLLhandle(f, dvs));
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

	}
}

extern "C"
{
	dllInitResult Ryan_Debug_registry_register_dll(const Ryan_Debug::registry::DLLpreamble &p, void* ptr)
	{
		using namespace Ryan_Debug;
		Ryan_Debug::registry::DLLpreamble b = p;
		auto h = boost::shared_ptr<const moduleInfo>(getModuleInfo(ptr), freeModuleInfo);
		std::string dllPath(getPath(h.get()));
		h.reset();

		auto& lg = Ryan_Debug::registry::m_reg::get();
		BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Registering DLL info for: " << dllPath << "\n"
			<< "Name: " << p.name << "\n"
			<< "UUID: " << p.uuid << "\n"
			<< "Description: " << p.description << "\n";

		for (const auto & i : preambles)
		{
			if (std::strcmp(i.uuid, p.uuid) == 0) {
				// Duplicate load detected!
				BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "Duplicate DLL load detected for: " << dllPath << "\n"
					<< "Name: " << p.name << "\n"
					<< "UUID: " << p.uuid << "\n"
					<< "Description: " << p.description << "\n";

				// Cannot Throw failure condition.
				//RDthrow(Ryan_Debug::error::xDuplicateHook())
				//	<< Ryan_Debug::error::file_name(dllPath);
				return DUPLICATE_DLL;
			}
		}
		preambles.push_back(b);
		return SUCCESS;
	}

}
