#include <iostream>
#include <boost/program_options.hpp>
#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/info.h"
#include "Ryan.Debug.DebugAssembly.manifest.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace Ryan_Debug;
	namespace po = boost::program_options;
	po::positional_options_description p;
	cerr << "Ryan Debug Testing Application" << endl << endl;

	po::options_description cmdline("Command-line options"), config("Config options"),
		hidden("Hidden options"), desc("Allowed options"), oall("all options");
	Ryan_Debug::add_options(cmdline, config, hidden);

	desc.add(cmdline).add(config);
	oall.add(cmdline).add(config).add(hidden);

	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).options(oall).
		positional(p).run(), vm);
	po::notify(vm);

	Ryan_Debug::process_static_options(vm);


	cerr << "Library Build Settings:\n";
	Ryan_Debug::printDebugInfo();

	cerr << "Testing app build settings:\n";
	Ryan_Debug::debug_preamble(cerr);
	cerr << endl;
	return 0;
}

