#include <iostream>
#include <vector>
#include <sstream>
#include <string>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>
#include "../scatdb_ryan/scatdb_ryan.hpp"

int main(int argc, char** argv) {
	try {
		using namespace std;
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		Ryan_Debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message");

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		Ryan_Debug::process_static_options(vm);

		auto doHelp = [&](const std::string& s)
		{
			cout << s << endl;
			cout << desc << endl;
			exit(3);
		};

		if (vm.count("help") || vm.size() == 0) doHelp("");

		using namespace scatdb_ryan;
		std::string dbfile;
		db::findDB(dbfile);
		if (!dbfile.size()) doHelp("Unable to detect database file.");
		cerr << "Database file is at: " << dbfile << endl;
		auto sdb = db::loadDB();
		cerr << "Database loaded." << endl;
	} catch (std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
