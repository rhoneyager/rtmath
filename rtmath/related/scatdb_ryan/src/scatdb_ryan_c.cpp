#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../scatdb_ryan/scatdb_ryan.h"
#include "../scatdb_ryan/scatdb_ryan.hpp"

namespace {
	std::string lastErr;
	std::vector<std::shared_ptr<const scatdb_ryan::scatdb_base> > ptrs;
}

extern "C" {

	bool SDBR_free(SDBR_HANDLE h) {
		using namespace scatdb_ryan;
		for (auto it = ptrs.begin(); it != ptrs.end(); ++it) {
			if (it->get() == (scatdb_base*) (h)) {
				//*it = nullptr;
				it->reset();
				return true;
			}
		}
		return false;
	}

	int SDBR_err_len() {
		if (lastErr.size()) return lastErr.size() + 1;
		return 0;
	}

	int SDBR_err_msg(int maxlen, char* buffer) {
		int cplen = maxlen;
		if (lastErr.size() + 1 < maxlen) cplen = lastErr.size();
		std::strncpy(buffer, lastErr.c_str(), cplen);
		buffer[maxlen-1] = '\0';
	}

	SDBR_HANDLE SDBR_loadDB(const char* dbfile) {
		using namespace scatdb_ryan;
		try {
			auto d = db::loadDB(dbfile);
			ptrs.push_back(d);
			lastErr = "";
			return (SDBR_HANDLE)(d.get());
		} catch (std::exception &e) {
			lastErr = std::string(e.what());
			return 0;
		}
	}

	bool SDBR_writeDB(SDBR_HANDLE handle, const char* outfile) {
		using namespace scatdb_ryan;
		try {
			db* h = ( db* )(handle);
			std::ofstream out(outfile);
			(h)->print(out);
			lastErr="";
		} catch (std::bad_cast &) {
			lastErr = "Passed handle in SDBR_writeDB is not a database handle.";
			return false;
		} catch (std::exception &e) {
			lastErr = std::string(e.what());
			return false;
		}
		return true;
	}

	bool SDBR_start(int argc, char** argv) {
		try {
			namespace po = boost::program_options;
			po::options_description desc("Allowed options"), cmdline("Command-line options"),
				config("Config options"), hidden("Hidden options"), oall("all options");

			Ryan_Debug::add_options(cmdline, config, hidden);

			desc.add(cmdline).add(config);
			oall.add(cmdline).add(config).add(hidden);

			po::variables_map vm;
			po::store(po::command_line_parser(argc, argv).
				options(oall).run(), vm);
			po::notify(vm);

			Ryan_Debug::process_static_options(vm);
		} catch (std::exception &e) {
			lastErr = std::string(e.what());
			return false;
		}
		return true;
	}
}


