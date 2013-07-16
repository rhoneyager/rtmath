#include <memory>
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/error/error.h"
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::debug;
	using namespace rtmath::config;
	try {
		cerr << "Parameter expansion program" << endl;

		namespace po = boost::program_options;
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "Produce help message")
			("split,s", po::value<std::string>(), "The values to be expanded")
			("aliases,k", po::value<std::string>(), "The key location in the config tables that contains the desired expansion table");

		po::positional_options_description p;
		p.add("split",1);
		
		po::variables_map vm;
		po::store(po::command_line_parser(argc,argv).
			options(desc).positional(p).run(), vm);
		po::notify (vm);

		if (vm.count("help"))
		{
			std::cerr << desc << std::endl;
			exit(1);
		}

		std::string src, nodename;
		bool mapAliases = false;
		
		if (vm.count("aliases"))
		{
			mapAliases = true;
			nodename = vm["aliases"].as<std::string>();
		}

		if (vm.count("split"))
		{
			src = vm["split"].as<std::string>();
		}

		// Will attempt to load a parameter and will perform expansion based on 
		// paramset expansion from key/value combinations found in rtmath.conf

		// config::loadRtconfroot is called during appEntry...
		std::shared_ptr<configsegment> root = rtmath::config::getRtconfRoot();
		std::shared_ptr<configsegment> node;
		if (mapAliases)
			node = root->findSegment(nodename);

		paramSet<double>::aliasmap aliases;
		if (mapAliases)
			node->listKeys(aliases);
		paramSet<double> expander(src, &aliases);

		// expander now has the full expanded list of everything
		cout << "Expansion of " << src << ":\n\n";

		for (auto it = expander.begin(); it != expander.end(); it++)
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


