#include "Stdafx-core.h"
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
#include <boost/log/sources/global_logger_storage.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/fs.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/dlls.h>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/config.h>
#include "../rtmath/info.h"
#include "../rtmath/config.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/registry.h"

using namespace rtmath::debug;
namespace rtmath
{
	namespace registry
	{
		/// Recursive and single-level DLL loading paths
		std::set<boost::filesystem::path> searchPathsRecursive, searchPathsOne;

		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_reg,
			boost::log::sources::severity_channel_logger_mt< >,
			(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "registry"));

		/** \todo Need to heavily modify to determine dll search paths from configuration and select 
		* folders. Load the plugins using a custom validator that checks both Ryan_Debug and rtmath.
		**/


		void emit_registry_log(const std::string &m, ::Ryan_Debug::log::severity_level sev)
		{
			auto& lg = rtmath::registry::m_reg::get();
			BOOST_LOG_SEV(lg, sev) << m;
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
				("rtmath-dll-load-onelevel", po::value<std::vector<std::string> >()->multitoken(),
				"Specify dlls to load. If passed a directory, it loads all dlls present (one-level).")
				("rtmath-dll-load-recursive", po::value<std::vector<std::string> >()->multitoken(),
				"Specify dlls to load. If passed a directory, it loads all dlls present (recursing).")
				;
		}

		namespace dll {
			class validateRtmathDLL : public ::Ryan_Debug::registry::dllValidator {
			protected:
				validateRtmathDLL() {}
			public:
				~validateRtmathDLL() {}
				virtual const char* validationSymbol() const {
					static const char* sym = "dlVerRTMATH";
					return sym;
				}

				virtual bool validate(void* func, bool critical = false) const {
					auto& lg = m_reg::get();
					Ryan_Debug::log::severity_level sl = (critical) ? Ryan_Debug::log::critical : Ryan_Debug::log::error;
					void(*fVer)(rtmath::versioning::versionInfo&, void**) =
						(void(*)(rtmath::versioning::versionInfo&, void**)) func;
					auto h = boost::shared_ptr<const Ryan_Debug::moduleInfo>(Ryan_Debug::getModuleInfo((void*)func), Ryan_Debug::freeModuleInfo);
					std::string filename(Ryan_Debug::getPath(h.get()));
					h.reset();
					//dllInitResult(*vfStart)() = nullptr;
					//void* vfStarta = nullptr;
					if (fVer) {
						using namespace rtmath::versioning;
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
								<< " references an incompatible rtmath version!" << std::endl
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
								<< " references a slightly different rtmath version!" << std::endl
								<< "DLL version: \n" << sdver << std::endl
								<< "My version: \n" << smver << std::endl;
						}

						// Check that the DLL's Ryan_Debug function calls are really to the correct code.
						void *mdcheck = (void*)&(Ryan_Debug_registry_register_dll);
						if (rdcheck != mdcheck) {
							using namespace Ryan_Debug::registry;
							using namespace Ryan_Debug;
							auto h = boost::shared_ptr<const Ryan_Debug::moduleInfo>(Ryan_Debug::getModuleInfo((void*)mdcheck), Ryan_Debug::freeModuleInfo);
							std::string myPath(getPath(h.get()));
							auto ho = boost::shared_ptr<const Ryan_Debug::moduleInfo>(Ryan_Debug::getModuleInfo((void*)rdcheck), Ryan_Debug::freeModuleInfo);
							std::string rPath;
							if (rdcheck) rPath = (Ryan_Debug::getPath(ho.get())); else rPath = "Unknown";
							h.reset();
							ho.reset();

							BOOST_LOG_SEV(lg, sl) << "Dll " << filename << " with critical flag " << critical
								<< " is loading the wrong rtmath version!" << std::endl
								<< "DLL's rtmath path: \n" << rPath << std::endl
								<< "My rtmath path: \n" << myPath << std::endl;
							if (critical)
								RDthrow(Ryan_Debug::error::xDLLversionMismatch())
								<< Ryan_Debug::error::file_name(filename)
								<< Ryan_Debug::error::is_Critical(critical);
							return false;
						}
					}
					else {
						BOOST_LOG_SEV(lg, sl) << "Dll " << filename << " with critical flag " << critical
							<< " is missing rtmath version information!" << std::endl;
						if (critical)
							RDthrow(Ryan_Debug::error::xDLLversionMismatch())
							<< Ryan_Debug::error::file_name(filename)
							<< Ryan_Debug::error::is_Critical(critical);
						return false;
					}
					return true;
				}
				static boost::shared_ptr<const dllValidator> genRtmathValidator()  {
					static boost::shared_ptr<const dllValidator> def(new validateRtmathDLL());
					return def;
				}
			};

			boost::shared_ptr<const ::Ryan_Debug::registry::dllValidatorSet> getDebugRtmathValidatorDefault()
			{
				static bool gen = false;
				static boost::shared_ptr<const ::Ryan_Debug::registry::dllValidatorSet> def;
				if (!gen)
				{
					boost::shared_ptr<::Ryan_Debug::registry::dllValidatorSet> a = ::Ryan_Debug::registry::dllValidatorSet::generate();
					a->append(::Ryan_Debug::registry::dllValidator::genDefaultValidator());
					a->append(validateRtmathDLL::genRtmathValidator());
					def = a;
				}
				return def;
			}

			void loadDLL(const std::string &filename) {
				Ryan_Debug::registry::loadDLL(filename, getDebugRtmathValidatorDefault());
			}

			void loadDLLs(const std::vector<std::string> &dlls) {
				Ryan_Debug::registry::loadDLLs(dlls, getDebugRtmathValidatorDefault());
			}

			/**
			* \brief Determines the search paths for dlls in the Ryan_Debug.conf file and in environment variables
			*
			**/
			void constructSearchPaths(bool use_cmake = false, bool use_rtmath_conf = true, bool use_environment = true)
			{
				using std::vector;
				using std::set;
				using std::string;
				auto& lg = m_reg::get();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Constructing search paths";

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
				searchPathsOne.emplace(libpath / "plugins");
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding library-relative path: " << libpath / "plugins";


				// Checking Ryan_Debug.conf
				if (use_rtmath_conf)
				{
					auto rtconf = rtmath::config::loadRtconfRoot();
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

					BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Adding paths from rtmath config file";
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
					searchEnviron("rtmath_plugins_DIR", searchPathsRecursive);
					searchEnviron("rtmath_dlls_recursive", searchPathsRecursive);
					searchEnviron("rtmath_dlls_onelevel", searchPathsOne);
				}
			}

		}

		void process_static_options(boost::program_options::variables_map &vm)
		{
			namespace po = boost::program_options;
			using std::string;

			auto& lg = m_reg::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Initializing rtmath registry system\n";

			//if (vm.count("dll-no-default-locations"))
			//	autoLoadDLLs = false;

			if (vm.count("rtmath-dll-load-onelevel"))
			{
				std::vector<std::string> sPaths = vm["rtmath-dll-load-onelevel"].as<std::vector<std::string> >();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll paths (1)\n";
				for (const auto s : sPaths)
				{
					searchPathsOne.emplace(s);
					BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll path (1): " << s << "\n";
				}
			}

			if (vm.count("rtmath-dll-load-recursive"))
			{
				std::vector<std::string> sPaths = vm["rtmath-dll-load-recursive"].as<std::vector<std::string> >();
				BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll paths (1)" << "\n";
				for (const auto s : sPaths)
				{
					searchPathsRecursive.emplace(s);
					BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Loading custom dll path (r): " << s << "\n";
				}
			}

			dll::constructSearchPaths(false, true, true);

			//if (vm.count("print-dll-search-paths"))
			//	Ryan_Debug::registry::printDLLsearchPaths(std::cerr);

			std::set<boost::filesystem::path> rPaths, oPaths;
			Ryan_Debug::registry::findPath(rPaths, boost::filesystem::path("default"), searchPathsRecursive, true);
			Ryan_Debug::registry::findPath(oPaths, boost::filesystem::path("default"), searchPathsOne, false);
			std::vector<std::string> toLoadDlls;
			// If a 'default' folder exists in the default search path, then use it for dlls.
			// If not, then use the base plugins directory.
			// Any library version / name detecting logic is in loadDLL (called by loadDLLs).

			if (rPaths.size() || oPaths.size())
			{
				Ryan_Debug::registry::searchDLLs(toLoadDlls, rPaths, true);
				Ryan_Debug::registry::searchDLLs(toLoadDlls, oPaths, false);
			}
			else { Ryan_Debug::registry::searchDLLs(toLoadDlls); }

			dll::loadDLLs(toLoadDlls);
			

			if (vm.count("print-dll-loaded"))
				Ryan_Debug::registry::printDLLs();
		}

	}
}
