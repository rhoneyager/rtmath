#include "Stdafx-core.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

#include <Ryan_Debug/io.h>
#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging.h>
#include <Ryan_Debug/fs.h>
#include <Ryan_Debug/config.h>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/Serialization.h>

#include "../rtmath/error/debug.h"
#include "cmake-settings.h"

namespace {
	std::set<std::string> mtypes, mnewtypes;
	std::mutex cmlock;
	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_config,
		boost::log::sources::severity_channel_logger_mt< >,
		(boost::log::keywords::severity = Ryan_Debug::log::error)(boost::log::keywords::channel = "config"));

	boost::shared_ptr<::Ryan_Debug::config::configsegment> _rtconfroot = nullptr;

}

namespace rtmath {
	namespace config {

		/**
		* \brief Function that returns the location of the rtmath.conf file
		*
		* Finding the default config file has become a rather involved process.
		* First, check the application execution arguments (if using appEntry).
		* Second, check the environment variables. Uses the key RTMATH_CONF, and accepts
		* multiple files, separated by semicolons. Searches for file existence in 
		* left-to-right order.
		*
		* \todo Third, check the system registry (if using Windows).
		*
		* Finally, check using the precompiled paths.
		**/
		DLEXPORT_rtmath_core void getConfigDefaultFile(std::string &filename)
		{
			filename = "";
			using namespace boost::filesystem;
			auto& lg = m_config::get();

			BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Finding rtmath configuration file";

			// Check application execution arguments
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking app command line";
			path testCMD(rtmath::debug::sConfigDefaultFile);
			if (exists(testCMD))
			{
				filename = rtmath::debug::sConfigDefaultFile;
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << filename;
				return;
			}

			// Checking environment variables
			using namespace Ryan_Debug;
			boost::shared_ptr<const processInfo> info(getInfo(getPID()), freeProcessInfo);

			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking rtmath_conf environment variable";
			size_t sEnv = 0;
			const char* cenv = getEnviron(info.get(), sEnv);
			std::string env(cenv, sEnv);

			//Ryan_Debug::processInfo info = Ryan_Debug::getInfo(Ryan_Debug::getPID());
			std::map<std::string, std::string> mEnv;
			splitSet::splitNullMap(env, mEnv);
			//std::vector<std::string> mCands;
			auto findEnv = [&](const std::string &fkey, std::string &outname) -> bool {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Parsing environment variable: " << fkey;

				std::string flkey = fkey;
				std::transform(flkey.begin(), flkey.end(), flkey.begin(), ::tolower);
				auto it = std::find_if(mEnv.cbegin(), mEnv.cend(),
					[&flkey](const std::pair<std::string, std::string> &pred)
				{
					std::string key = pred.first;
					std::transform(key.begin(), key.end(), key.begin(), ::tolower);
					if (key == flkey) return true;
					return false;
				});
				if (it != mEnv.cend())
				{
					typedef boost::tokenizer<boost::char_separator<char> >
						tokenizer;
					boost::char_separator<char> sep(";");

					BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Candidates are: " << it->second;
					std::string ssubst;
					tokenizer tcom(it->second, sep);
					for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
					{
						path testEnv(it->second);
						if (exists(testEnv))
						{
							outname = it->second;
							BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using " << outname;
							return true;
						}
					}
				}
				else BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Cannot find environment variable: " << fkey;
				return false;
			};
			if (findEnv("rtmath_conf", filename)) return;

			// Check the system registry
			// TODO

			// Check a few other places
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking app data directories... ";

			std::string sAppConfigDir(Ryan_Debug::getAppConfigDir());
			std::string sHomeDir(Ryan_Debug::getHomeDir());
			auto hm = boost::shared_ptr<const moduleInfo>(getModuleInfo((void*)&getConfigDefaultFile), freeModuleInfo);
			std::string dllPath(getPath(hm.get()));

			auto hp = boost::shared_ptr<const processInfo>(Ryan_Debug::getInfo(Ryan_Debug::getPID()), freeProcessInfo);
			std::string appPath(getPath(hp.get()));

			std::string sCWD(Ryan_Debug::getCwd(hp.get()));
			// For all of these places, search for file names matching rtmath.xml, rtmath.conf and .rtmath.
			// Compression is allowed.
			auto searchPath = [&](const std::string &base, const std::string &suffix, bool searchParent) -> bool
			{
				using namespace boost::filesystem;
				path pBase(base);
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Getting search path based on: " << pBase.string();
				if (base.size() == 0) return false;
				if (!is_directory(pBase))
					pBase.remove_filename();
				if (searchParent) pBase.remove_leaf();
				if (suffix.size()) pBase = pBase / path(suffix);

				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Searching in: " << pBase.string();

				path p1 = pBase / "rtmath.xml";
				path p2 = pBase / "rtmath.conf";
				path p3 = pBase / ".rtmath";

				bool res = false;
				path pRes;
				std::string meth;
				res = Ryan_Debug::serialization::detect_compressed<path>(p1, meth, pRes);
				if (!res) res = Ryan_Debug::serialization::detect_compressed<path>(p2, meth, pRes);
				if (!res) res = Ryan_Debug::serialization::detect_compressed<path>(p3, meth, pRes);
				if (!res) return false;
				filename = pRes.string();
				return true;
			};
			bool found = false; // junk variable

			if (searchPath(sCWD, "", true)) found = true;
			else if (searchPath(sAppConfigDir, "rtmath", false)) found = true;
			else if (searchPath(sHomeDir, "", false)) found = true;
			else if (searchPath(dllPath, "", true)) found = true;
			else if (searchPath(appPath, "", true)) found = true;

			if (!filename.size()) {
				std::string RDCpath;
				findEnv("rtmath_DIR", RDCpath);
				if (searchPath(RDCpath, "../../../../share", false)) found = true;
			}
			if (searchPath(dllPath, "../../share", false)) found = true;
			if (filename.size()) {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using conf file: " << filename;
				return;
			}


			// Finally, just use the default os-dependent path
			//filename = "/home/rhoneyag/.Ryan_Debug";
			// Macro defining the correct path
			//BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking compile-time paths: "
				//<< "RTC: " << RTC << "\nRTCB: " << RTCB << "\nRTCC: " << RTCC
			//	<< "\nSYS_RTC: " << SYS_RTC;
			//path testUser(RTC);
			//path testUserB(RTCB);
			//path testUserC(RTCC);
			//path testSys(SYS_RTC);
			//if (exists(testUser))
			//	filename = RTC;
			//else if (exists(testUserB))
			//	filename = RTCB;
			//else if (exists(testUserC))
			//	filename = RTCC;
			//if (exists(testSys))
			//	filename = SYS_RTC;
			//if (filename.size()) BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using conf file: " << filename;
			//else 
			BOOST_LOG_SEV(lg, Ryan_Debug::log::critical) << "Unable to find Ryan_Debug configuration file. "
				<< "Log channel config at severity debug_2 lists the searched paths. You can specify the file by "
				"command-line (option --Ryan_Debug-config-file), environment variable (Ryan_Debug_conf), "
				"or place one in an at-compile-time-specified path.";

			return;
		}

		DLEXPORT_rtmath_core boost::shared_ptr<::Ryan_Debug::config::configsegment> getRtconfRoot()
		{
			return _rtconfroot;
		}

		DLEXPORT_rtmath_core void setRtconfRoot(boost::shared_ptr<::Ryan_Debug::config::configsegment> &root)
		{
			_rtconfroot = root;
		}

		DLEXPORT_rtmath_core boost::shared_ptr<::Ryan_Debug::config::configsegment> loadRtconfRoot(const std::string &filename)
		{
			std::lock_guard<std::mutex> lock(cmlock);
			if (_rtconfroot != nullptr) return _rtconfroot;
			auto& lg = m_config::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loading rtmath config file.\n";
			if (filename.size())
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Override filename " << filename << "\n";
			std::string fn = filename;
			if (!fn.size()) getConfigDefaultFile(fn);
			if (fn.size()) BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Found rtmath config file " << fn << "\n";
			if (!fn.size()) RDthrow(Ryan_Debug::error::xMissingRyan_DebugConf())
				<< Ryan_Debug::error::file_name(filename) << Ryan_Debug::error::default_file_name(fn);
			//boost::shared_ptr<configsegment> cnf = configsegment::loadFile(fn.c_str(), nullptr);
			auto opts = Ryan_Debug::registry::IO_options::generate(Ryan_Debug::registry::IOhandler::IOtype::READONLY);
			opts->filename(fn);

			std::vector< boost::shared_ptr<Ryan_Debug::config::configsegment> > rootcands;
			Ryan_Debug::config::configsegment::readVector(nullptr, opts, rootcands, nullptr);
			boost::shared_ptr<Ryan_Debug::config::configsegment> cnf;
			for (const auto &r : rootcands)
			{
				if (r->name() == "RTMATH" || r->name() == "ROOT" || (r->name() == "" && rootcands.size() == 1)) cnf = r;
			}
			if (cnf) {
				_rtconfroot = cnf;
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loaded rtmath config file successfully. "
					"Searching for hash databases. Top level children are: ";
				// Enumerate top level children.
				auto ccnf = cnf->getChild("RTMATH");
				std::multiset<std::string> children;
				ccnf->listChildren(children);
				std::ostringstream schd;
				for (const auto &i : children)
					schd << "\t" << i << "\n";
				BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << schd.str();
				// Search for hash databases to load
				auto hashes = ccnf->getChild("ddscat");
				if (!hashes) { hashes = ccnf; BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "No ddscat key"; }
				hashes = hashes->getChild("hashes");
				if (hashes) {
					BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Found hashes key";
					Ryan_Debug::hash::hashStore::loadStoresFromSource(hashes, "rtmath");
				} else {
					BOOST_LOG_SEV(lg, Ryan_Debug::log::warning) << "When loading the rtmath "
						"configuration file, no /ddscat/hashes or /hashes keys were found. "
						"This is undesirable, as no hash databases are loaded.";
				}
			}
			return cnf;
		}

	}
}

