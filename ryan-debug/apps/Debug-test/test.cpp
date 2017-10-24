#include <iostream>
#include <boost/program_options.hpp>
#define IGNORE_MANIFESTS
#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/info.h"
#include "../../Ryan_Debug/config.h"
//#include "Ryan.Debug.DebugAssembly.manifest.h"

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

	cmdline.add_options()
		("write-conf", po::value<std::string>(), "Write the Ryan_Debug config file to specified location.")
		;

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
	Ryan_Debug::versioning::debug_preamble(cerr);
	cerr << endl;

	cerr << "Other functions tested:\n";
	cerr << "\tApp config dir: " << Ryan_Debug::getAppConfigDir() << std::endl;
	cerr << "\tUsername: " << Ryan_Debug::getUsername() << std::endl;
	cerr << "\tHome dir: " << Ryan_Debug::getHomeDir() << std::endl;
	cerr << "\tHostname: " << Ryan_Debug::getHostname() << std::endl;

	if (vm.count("write-conf")) {
		std::string sout = vm["write-conf"].as<std::string>();
		cerr << "Writing config file to: " << sout << std::endl;
		auto conf = Ryan_Debug::config::getRtconfRoot();
		conf->writeFile(sout);
	}
	return 0;
}

