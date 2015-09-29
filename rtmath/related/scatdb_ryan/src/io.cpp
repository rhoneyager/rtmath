#include <cctype>
#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>

#include <Ryan_Debug/io.h>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/fs.h>
#include <Ryan_Debug/macros.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/Serialization.h>

#include "../scatdb_ryan/scatdb_ryan.hpp"

namespace {
	const char scatdb_name[] = "scatdb_ag_ryan.csv";
	const char scatdb_db_env[] = "scatdb_ryan_db";
	const char scatdb_dir_env[] = "scatdb_ryan_DIR";
	const char scatdb_config_dir[] = "scatdb_ryan";

	/// Always points to the first loaded database file.
	std::shared_ptr<const scatdb_ryan::db> loadedDB;
	std::mutex m_db;
}

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
		if (findEnv(std::string(scatdb_db_env), filename)) return true;

		// Check a few other places
		std::string sAppConfigDir(Ryan_Debug::getAppConfigDir());
		std::string sHomeDir(Ryan_Debug::getHomeDir());
		auto hm = boost::shared_ptr<const moduleInfo>
			(getModuleInfo((void*)&(db::findDB)), freeModuleInfo);
		std::string dllPath(getPath(hm.get()));

		auto hp = boost::shared_ptr<const processInfo>(Ryan_Debug::getInfo(Ryan_Debug::getPID()), freeProcessInfo);
		std::string appPath(getPath(hp.get()));

		std::string sCWD(Ryan_Debug::getCwd(hp.get()));
		auto searchPath = [&](const std::string &base, const std::string &suffix, bool searchParent) -> bool
		{
			using namespace boost::filesystem;
			path pBase(base);
			if (base.size() == 0) return false;
			if (!is_directory(pBase))
				pBase.remove_filename();
			if (searchParent) pBase.remove_leaf();
			if (suffix.size()) pBase = pBase / path(suffix);


			path p1 = pBase / std::string(scatdb_name);

			bool res = false;
			path pRes;
			std::string meth;
			res = Ryan_Debug::serialization::detect_compressed<path>(p1, meth, pRes);
			if (!res) return false;
			filename = pRes.string();
			return true;
		};
		bool found = false; // junk variable

		if (searchPath(sCWD, "", true)) found = true;
		else if (searchPath(sAppConfigDir, std::string(scatdb_config_dir), false)) found = true;
		else if (searchPath(sHomeDir, "", false)) found = true;
		else if (searchPath(dllPath, "", true)) found = true;
		else if (searchPath(appPath, "", true)) found = true;

		if (!filename.size()) {
			std::string RDCpath;
			findEnv(std::string(scatdb_dir_env), RDCpath);
			if (searchPath(RDCpath, "../../../../share", false)) found = true;
		}
		if (searchPath(dllPath, "../../share", false)) found = true;
		if (filename.size()) return true;

		std::cerr << "Unable to find the " << scatdb_name << " database file. " << std::endl;

		return false;
	}

	std::shared_ptr<const db> db::loadDB(const char* dbfile) {
		std::lock_guard<std::mutex> lock(m_db);
		if (!dbfile && loadedDB) return loadedDB;

		// Load the database
		std::string dbf;
		if (dbfile) dbf = std::string(dbfile);
		if (!dbf.size()) findDB(dbf); // If dbfile not provided, take a guess.
		if (!dbf.size()) {
			// Scattering database cannot be found.
		} else {
			using namespace boost::filesystem;
			path p(dbf);
			if (!exists(p)) {
				// Supplied path to scattering database does not exist.
				// Scattering database cannot be found.
			}
		}

		// From this point, it is established that the scattering database does exist.
		std::shared_ptr<db> newdb(new db);

		std::ifstream in(dbf.c_str());
		// Ignore all lines that start with text. Ignore all blank lines.
		// Read line-by-line, converting from text into integers and floats. Store in
		// a vector, which gets copied elementwise into matrices.
		size_t line = 0;
		const size_t approxNumLines = 1000 * 10 * 2; // Overestimation
		std::vector<int> ints;
		std::vector<float> floats;
		ints.reserve(approxNumLines * data_entries::NUM_DATA_ENTRIES_INTS);
		floats.reserve(approxNumLines * data_entries::NUM_DATA_ENTRIES_FLOATS);
		size_t numLines = 0; // Actual number of entries
		while(in.good()) {
			std::string lin;
			std::getline(in,lin);
			line++;
			if (!lin.size()) continue; // Skip blank lines
			if (!std::isdigit(lin.at(0))) continue; // Skip comment lines
			// Line format is a bunch of comma-separated values. Expand based on commas.
			std::vector<std::string> ssplit;
			Ryan_Debug::splitSet::splitVector(lin,ssplit,',');
			if (ssplit.size() != data_entries::NUM_DATA_ENTRIES_FLOATS
					+ data_entries::NUM_DATA_ENTRIES_INTS) {
				std::cerr << "Bad entry formatting on line " << line << std::endl;
				continue;
			}
			// Iterate over column numbers
			for (size_t i=0; i< ssplit.size(); ++i) {
				using namespace Ryan_Debug::macros;
				switch (i) {
					case 0: // flaketype
						ints.push_back(fastCast<int>(ssplit[i]));
						break;
					default: // All other columns
						floats.push_back(fastCast<float>(ssplit[i]));
						break;
				}
			}
			numLines++;
		}

		// The raw data hase been loaded. It now needs to be placed into the relevent
		// matrices.
		newdb->floatMat.resize(numLines, data_entries::NUM_DATA_ENTRIES_FLOATS);
		newdb->intMat.resize(numLines, data_entries::NUM_DATA_ENTRIES_INTS);
		for(size_t i=0; i<numLines; ++i) {
			newdb->intMat(i,0) = ints.at(i);
			for (size_t j=0; j<data_entries::NUM_DATA_ENTRIES_FLOATS; ++j)
				newdb->floatMat(i,j) = floats.at( (i*data_entries::NUM_DATA_ENTRIES_FLOATS) + j );
		}

		// The usual case is that the database is loaded once. If so, store a copy for
		// subsequent function calls.
		if (!loadedDB) loadedDB = newdb;

		return newdb;
	}
}

