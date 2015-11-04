#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <string>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/splitSet.h>

#include "read.h"

int main(int argc, char** argv) {
	try {
		using namespace std;
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
			config("Config options"), hidden("Hidden options"), oall("all options");

		Ryan_Debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("cross,c", po::value<string>(), "cross file")
			("phase,p", po::value<string>(), "phase file")
			("phys,y", po::value<string>(), "phys file")
			("output,o", po::value<string>(), "Output file")
			("delimiter,d", po::value<string>()->default_value("\t"), "Field separator (defaults to tab)")
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
		if (!vm.count("output")) doHelp("An output file must be specified.");
		if (!vm.count("cross")) doHelp("Cross file must be specified.");
		if (!vm.count("phase")) doHelp("Phase file must be specified.");
		if (!vm.count("phys")) doHelp("Phys file must be specified.");
		string dc = vm["delimiter"].as<string>();
		if (dc == "tab" || dc == "\\t") dc = "\t";
		if (dc.size() != 1) doHelp("Field delimiter must be a single character.");
		string sOutput = vm["output"].as<string>();
		string sCross = vm["cross"].as<string>();
		string sPhase = vm["phase"].as<string>();
		string sPhys = vm["phys"].as<string>();

		ofstream out(sOutput.c_str());
		out << "ID" << dc << "Source" << dc << "Type" << dc
			<< "aeff (um)" << dc << "Frequency (GHz)" << dc
			<< "Wavelength (um)" << dc << "Dipole Spacing (um)" << dc
			<< "Maximum Diameter (mm)" << dc
			<< "Aspect Ratio" << dc
			<< "Qabs" << dc << "Qbk" << dc << "Qext" << dc << "Qsca" << dc << "g" << dc
			<< endl;

		vector<data_entry> data;
		data.reserve(1000 * 100);
		readFile(sCross, sPhase, sPhys, data);
		for (const auto &i : data) {
			out << i.id << dc << i.source << dc << i.aeff << dc << i.freq << dc
				<< i.wave << dc << i.dspacing << dc << i.md << dc << i.ar << dc
				<< i.qabs << dc << i.qbk << dc << i.qext << dc << i.qsca << dc << i.g << endl;
		}


	} catch (std::exception &e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}

