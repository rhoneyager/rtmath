#include <memory>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/error.h"
#include "../../Ryan_Debug/hash.h"
#include "../../Ryan_Debug/splitSet.h"

int main(int argc, char** argv)
{
	using namespace std;
	try {
		namespace po = boost::program_options;
		po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "Produce help message")
			("lower,l", po::value<bool>()->default_value(true), "Display lower hash")
			("upper,u", po::value<bool>()->default_value(false), "Display upper hash")
			("filename,f", po::value<string>(), "File to hash. If not specified, "
			 "then hash cin.");
			;

		po::positional_options_description p;
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
		if (argc < 2) doHelp();
		bool lower = vm["lower"].as<bool>(), upper = vm["upper"].as<bool>();
		string fname;
		if (vm.count("filename")) fname = vm["filename"].as<string>();

		Ryan_Debug::hash::HASH_t hash;
		if (fname.size())
			hash = Ryan_Debug::hash::HASHfile(fname);
		else {
			std::cin >> std::noskipws;
			std::istream_iterator<char> it(std::cin);
			std::istream_iterator<char> end;
			std::string results(it, end);
			hash = Ryan_Debug::hash::HASH(results.c_str(), results.size());
		}

		if (lower) cout << hash.lower;
		if (lower && upper) cout << " ";
		if (upper) cout << hash.upper;
		cout << endl;

	}
	catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}


