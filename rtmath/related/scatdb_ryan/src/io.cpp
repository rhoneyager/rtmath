#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>

#include <Ryan_Debug/io.h>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/fs.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/Serialization.h>

#include "../scatdb_ryan/scatdb_ryan.hpp"

namespace scatdb_ryan {
	/**
	* \brief Function that returns the location of the scattering database file.
	*
	* Finding the default config file has become a rather involved process.
	* First, check the application execution arguments (if available).
	* Second, check the environment variables. Uses the key "scatdb_ryan_db", and accepts
	* multiple files, separated by semicolons. Searches for file existence in
	* left-to-right order.
	*
	* Finally, check using the precompiled paths.
	**/
	bool db::findDB(std::string &filename) {
		filename = "";
		using namespace boost::filesystem;

		// Check application execution arguments
		//path testCMD(rtmath::debug::sConfigDefaultFile);
		//if (exists(testCMD))
		//{
		//	filename = rtmath::debug::sConfigDefaultFile;
		//	return true;
		//}

		// Checking environment variables
		using namespace Ryan_Debug;
		boost::shared_ptr<const processInfo> info(getInfo(getPID()), freeProcessInfo);

		size_t sEnv = 0;
		const char* cenv = getEnviron(info.get(), sEnv);
		std::string env(cenv, sEnv);

		std::map<std::string, std::string> mEnv;
		splitSet::splitNullMap(env, mEnv);
		auto findEnv = [&](const std::string &fkey, std::string &outname) -> bool {
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

				std::string ssubst;
				tokenizer tcom(it->second, sep);
				for (auto ot = tcom.begin(); ot != tcom.end(); ot++)
				{
					path testEnv(it->second);
					if (exists(testEnv))
					{
						outname = it->second;
						return true;
					}
				}
			}
			return false;
		};
		if (findEnv("scatdb_ryan_db", filename)) return true;

		// Check a few other places
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
}

