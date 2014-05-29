#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include "../../Ryan_Debug/debug.h"
#include "../../Ryan_Debug/modules.h"


int main(int argc, char **argv)
{
	try {
		using namespace std;
		using namespace Ryan_Debug;
		namespace po = boost::program_options;
		po::positional_options_description p;
		po::options_description cmdline("Command-line options"), config("Config options"), 
			hidden("Hidden options"), desc("Allowed options"), oall("all options");
		Ryan_Debug::add_options(cmdline, config, hidden);
		cmdline.add_options()
			("help,h", "produce help message")
			("cmake", po::value<vector<string> >(), "Set cmake directory for product")
			("lib", po::value<vector<string> >(), "Set library directories")
			("bin", po::value<vector<string> >(), "Set binary directories")
			("include", po::value<vector<string> >(), "Set include directories")
#ifdef _WIN32
			("cygwin-base", po::value<string>(), "Set cygwin base directory")
#endif
			("operation", po::value<string>(), "Specify action to take - "
			"create - creates a new module file matching the target information. If it exists, do nothing. "
			"update - create or update a module file. "
			//"erase - delete a matching module file (searches by key). "
			//"link - updates a module-containing directory with symlinks to the latest version for each tag. "
			//"default - set a given module file as a system default. "
			)
			("target", po::value<string>(), "Specify target file")
			("private", po::value<bool>()->default_value(0), "Is this a private module?")
			;
		p.add("operation", 1);
		p.add("target", 2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).options(oall).
			  positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << endl;
			exit(1);
		};
		if (vm.count("help")) doHelp("");
		Ryan_Debug::process_static_options(vm);

		if (!vm.count("operation")) doHelp("Need to specify an operation.");
		string op = vm["operation"].as<string>();
		if (!vm.count("target")) doHelp("Need to specify a target.");
		string starget = vm["target"].as<string>();

		using boost::filesystem::path;
		multimap<string, path> paths;
		auto addPaths = [&](const std::string &key, bool multi, bool required, bool cygrel)
		{
			if (!vm.count(key))
			{
				if (required) doHelp("Required path not specified.");
				else return;
			}
			vector<string> p = vm[key].as<vector<string> >();
			if (!multi && p.size() > 1) doHelp("Single value allowed for path.");
			for (const auto &i : p)
			{
				path pi(i);
				path pabs = boost::filesystem::absolute(pi);
				if (cygrel)
					pabs = Ryan_Debug::convertUnix(pabs);

				paths.insert(std::pair<string, path>(key, pabs));
			}
		};
		addPaths("cmake", false, false, false);
		addPaths("lib", true, false, true);
		addPaths("bin", true, false, true);
		addPaths("include", true, false, true);

		// Figure out the target path
		bool isPrivate = vm["private"].as<bool>();


		if (op == "create" || op == "update")
		{

		}
		else doHelp("Unrecognized operation.");
	} catch(std::exception & e) {
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}

