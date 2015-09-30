#include <iostream>
#include <fstream>
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
			("help,h", "produce help message")
			("stats", "Print stats for selected data")
			("output,o", po::value<string>(), "Output file")
			("db-file,d", "Manually specify database location")
			("flaketypes,y", po::value<string>(), "Filter flaketypes by number range")
			("frequencies,f", po::value<string>(), "Filter frequencies (GHz) by range")
			("temp,T", po::value<string>(), "Filter temperatures (K) by range")
			("aeff,a", po::value<string>(), "Range filter by effective radius (um)")
			("max-dimension,m", po::value<string>(), "Range filter by maximum dimension (mm)")
			("cabs", po::value<string>(), "Range filter by absorption cross-section (m)")
			("cbk", po::value<string>(), "Range filter by backscattering cross-section (m)")
			("cext", po::value<string>(), "Range filter by extinction cross-section (m)")
			("csca", po::value<string>(), "Range filter by scattering cross-section (m)")
			("asymmetry,g", po::value<string>(), "Range filter by asymmetry parameter")
			("as-xy", po::value<string>(), "Range filter by x-y aspect ratio")
			("as-xz", po::value<string>(), "Range filter by x-z aspect ratio")
			("as-yz", po::value<string>(), "Range filter by y-z aspect ratio")
			;

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
		if (vm.count("db-file")) dbfile = vm["db-file"].as<string>();
		db::findDB(dbfile);
		if (!dbfile.size()) doHelp("Unable to detect database file.");
		cerr << "Database file is at: " << dbfile << endl;
		auto sdb = db::loadDB();
		cerr << "Database loaded. Performing filtering." << endl;

		auto f = filter::generate();

		if (vm.count("flaketypes")) f->addFilterInt(db::data_entries::FLAKETYPE, vm["flaketypes"].as<string>());
		if (vm.count("frequencies")) f->addFilterFloat(db::data_entries::FREQUENCY_GHZ, vm["frequencies"].as<string>());
		if (vm.count("temp")) f->addFilterFloat(db::data_entries::TEMPERATURE_K, vm["temp"].as<string>());
		if (vm.count("aeff")) f->addFilterFloat(db::data_entries::AEFF_UM, vm["aeff"].as<string>());
		if (vm.count("max-dimension")) f->addFilterFloat(db::data_entries::MAX_DIMENSION_MM, vm["max-dimension"].as<string>());
		if (vm.count("cabs")) f->addFilterFloat(db::data_entries::CABS_M, vm["cabs"].as<string>());
		if (vm.count("cbk")) f->addFilterFloat(db::data_entries::CBK_M, vm["cbk"].as<string>());
		if (vm.count("cext")) f->addFilterFloat(db::data_entries::CEXT_M, vm["cext"].as<string>());
		if (vm.count("csca")) f->addFilterFloat(db::data_entries::CSCA_M, vm["csca"].as<string>());
		if (vm.count("asymmetry")) f->addFilterFloat(db::data_entries::G, vm["asymmetry"].as<string>());
		if (vm.count("as-xy")) f->addFilterFloat(db::data_entries::AS_XY, vm["as-xy"].as<string>());
		if (vm.count("as-xz")) f->addFilterFloat(db::data_entries::AS_XZ, vm["as-xz"].as<string>());
		if (vm.count("as-yz")) f->addFilterFloat(db::data_entries::AS_YZ, vm["as-yz"].as<string>());

		auto sdb_filtered = f->apply(sdb);
		if (vm.count("stats")) {
			auto stats = sdb_filtered->getStats();
			cerr << "Stats tables:" << endl;
			stats->print(cerr);
		}

		if (vm.count("output")) {
			std::string fout = vm["output"].as<std::string>();
			cerr << "Writing output to " << fout << endl;
			std::ofstream out(fout.c_str());
			sdb_filtered->print(out);
		}
	} catch (std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}
