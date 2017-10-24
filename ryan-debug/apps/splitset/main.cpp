#include <memory>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/error.h"
#include "../../Ryan_Debug/splitSet.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		cerr << "Parameter expansion program" << endl;

		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "Produce help message")
			("split,s", po::value<std::string>(), "The values to be expanded")
			;

		po::positional_options_description p;
		p.add("split",1);
		
		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);

		Ryan_Debug::process_static_options(vm);

		auto doHelp = [&]()
		{
			cerr << desc << std::endl;
			exit(1);
		};

		if (vm.count("help")) doHelp();

		string src, nodename;

		if (!vm.count("split")) doHelp();
		src = vm["split"].as<std::string>();

		set<double> expanded;
		Ryan_Debug::splitSet::splitSet<double>(src, expanded);
		
		cout << "Expansion of " << src << ":\n\n";

		for (auto it = expanded.begin(); it != expanded.end(); it++)
		{
			cout << *it << "\t";
		}
		cout << endl << endl;
	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}


