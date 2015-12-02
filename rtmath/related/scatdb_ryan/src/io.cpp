#include <cctype>
#include <iostream>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

#include <Ryan_Debug/io.h>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/fs.h>
#include <Ryan_Debug/logging.h>
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

	BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
		m_config,
		boost::log::sources::severity_channel_logger_mt< >,
		(boost::log::keywords::severity = Ryan_Debug::log::error)
		(boost::log::keywords::channel = "db"));

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
		auto& lg = m_config::get();
		if (filename.size()) {
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using database file: " << filename;
			return true;
		}

		filename = "";
		using namespace boost::filesystem;

		BOOST_LOG_SEV(lg, Ryan_Debug::log::notification) << "Finding scatdb_ag_ryan.csv file";
		// Check application execution arguments
		//BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking app command line";
		//path testCMD(rtmath::debug::sConfigDefaultFile);
		//if (exists(testCMD))
		//{
		//	filename = rtmath::debug::sConfigDefaultFile;
		//BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << filename;
		//	return true;
		//}

		// Checking environment variables
		using namespace Ryan_Debug;
		boost::shared_ptr<const processInfo> info(getInfo(getPID()), freeProcessInfo);
		BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking environment variable "
			<< std::string(scatdb_db_env);
		size_t sEnv = 0;
		const char* cenv = getEnviron(info.get(), sEnv);
		std::string env(cenv, sEnv);

		std::map<std::string, std::string> mEnv;
		splitSet::splitNullMap(env, mEnv);
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
			} else BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Cannot find environment variable: " << fkey;
			return false;
		};
		if (findEnv(std::string(scatdb_db_env), filename)) return true;

		// Check a few other places
		BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Checking app data directories... ";
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
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Getting search path based on: " << pBase.string();
			if (base.size() == 0) return false;
			if (!is_directory(pBase))
				pBase.remove_filename();
			if (searchParent) pBase.remove_leaf();
			if (suffix.size()) pBase = pBase / path(suffix);

			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Searching in: " << pBase.string();
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

		if (searchPath(sCWD, "", false)) found = true;
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
		if (filename.size()) {
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Using database file: " << filename;
			return true;
		}

		std::cerr << "Unable to find the " << scatdb_name << " database file. " << std::endl;

		return false;
	}

	void db::print(std::ostream &out) const {
		out << "flaketype,frequencyghz,temperaturek,aeffum,max_dimension_mm,"
			"cabs,cbk,cext,csca,g,ar" << std::endl;
		int rows = floatMat.rows();
		int icols = intMat.cols();
		int fcols = floatMat.cols();
		for (int i=0; i < rows; ++i) {
			int col = 0;
			for (int j=0; j<icols;++j) {
				if (col) out << ",";
				out << intMat(i,j);
				col++;
			}
			for (int j=0; j<fcols; ++j) {
				if (col) out << ",";
				out << floatMat(i,j);
				col++;
			}
			out << std::endl;
		}
	}

	std::shared_ptr<const db> db::loadDB(const char* dbfile) {
		std::lock_guard<std::mutex> lock(m_db);
		if (!dbfile && loadedDB) return loadedDB;

		auto& lg = m_config::get();
		// Load the database
		BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "loadDB called. Need to load database.";
		std::string dbf;
		if (dbfile) dbf = std::string(dbfile);
		if (!dbf.size()) {
			findDB(dbf); // If dbfile not provided, take a guess.
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "dbfile automatically determined as: " << dbf;
		} else {
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "dbfile provided in call: " << dbf;
		}
		if (!dbf.size()) {
			// Scattering database cannot be found.
			BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Scattering database cannot be found.";
			std::cerr << "Scattering database cannot be found.";
			RDthrow(Ryan_Debug::error::xMissingFile())
				<< Ryan_Debug::error::file_name(std::string(scatdb_name));
		} else {
			using namespace boost::filesystem;
			path p(dbf);
			if (!exists(p)) {
				// Supplied path to scattering database does not exist.
				// Scattering database cannot be found.
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2)
					<< "Scattering database cannot be found at " << dbf;
				std::cerr << "Scattering database cannot be found at " << dbf;
				RDthrow(Ryan_Debug::error::xMissingFile())
					<< Ryan_Debug::error::file_name(p.string());
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
				BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Bad entry formatting on line " << line;
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
		BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Overall database has "
			<< numLines << " lines of data that were successfully read.";
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

		BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_2) << "Database loaded successfully.";
		return newdb;
	}
}

