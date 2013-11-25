#include <iostream>
#include <fstream>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/config.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;

	namespace po = boost::program_options;

	po::options_description desc("Allowed options"), cmdline("Command-line options"),
		config("Config options"), hidden("Hidden options"), oall("all options");
	//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
	rtmath::debug::add_options(cmdline, config, hidden);

	rtmath::debug::debug_preamble();

	po::positional_options_description p;
	//p.add("output",2);

	desc.add(cmdline).add(config);
	oall.add(cmdline).add(config).add(hidden);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
		options(oall).positional(p).run(), vm);
	po::notify(vm);

	rtmath::debug::process_static_options(vm);

	string sConfig;
	rtmath::config::getConfigDefaultFile(sConfig);
	if (sConfig.size())
		cerr << "Using rtmath configuration file: " << sConfig << endl << endl;
	else cerr << "rtmath configuration file not found." << endl << endl;
	
	Ryan_Debug::printDebugInfo();

	return 0;
}
