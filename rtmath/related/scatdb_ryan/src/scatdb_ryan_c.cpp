#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
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
	std::map<const scatdb_ryan::scatdb_base*, std::shared_ptr<const scatdb_ryan::scatdb_base> > ptrs;
	//std::vector<std::shared_ptr<const scatdb_ryan::scatdb_base> > ptrs;
	// TODO: add casting checks again
}

extern "C" {

	bool SDBR_free(SDBR_HANDLE h) {
		using namespace scatdb_ryan;
		scatdb_base* hp = (scatdb_base*) (h);
		if (ptrs.count(hp)) {
			ptrs[hp].reset();
			return true;
		}
		return false;
		/*
		for (auto it = ptrs.begin(); it != ptrs.end(); ++it) {
			if (it->get() == (scatdb_base*) (h)) {
				//*it = nullptr;
				it->reset();
				return true;
			}
		}
		*/
		return false;
	}

	int SDBR_err_len() {
		if (lastErr.size()) return lastErr.size() + 1;
		return 0;
	}

	void prepString(int maxlen, char* buffer, const std::string &s) {
		int cplen = maxlen;
		if (s.size() + 1 < maxlen) cplen = s.size();
		std::strncpy(buffer, s.c_str(), cplen);
		buffer[cplen] = '\0';
	}

	int SDBR_err_msg(int maxlen, char* buffer) {
		prepString(maxlen, buffer, lastErr);
	}

	int SDBR_findDB(int maxlen, char* buffer) {
		using namespace scatdb_ryan;
		try {
			if (!buffer) {
				lastErr = "SDBR_findDB buffer is null";
				return 0;
			}
			std::string dbloc(buffer, maxlen);
			bool res = db::findDB(dbloc);
			if (!res) return 0;
			prepString(maxlen, buffer, dbloc);
		} catch (std::exception &e) {
			lastErr = std::string(e.what());
			return 0;
		}
	}

	SDBR_HANDLE SDBR_loadDB(const char* dbfile) {
		using namespace scatdb_ryan;
		try {
			auto d = db::loadDB(dbfile);
			ptrs[d.get()] = d;
			//ptrs.push_back(d);
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
			const scatdb_base* hp = ( const scatdb_base* )(handle);
			const db* h = dynamic_cast<const db*>(hp);
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


