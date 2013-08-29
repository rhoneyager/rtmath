/* This program is designed to process a ddscat run, creating a ddOutput object and storing it in the hash directory. */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/ddscat/ddUtil.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-postprocess\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >(),"specify input directory")
			("output,o", po::value<string>(), "specify output file")
			("hash,s", "Store ddscat output in the hash store")
			;

		po::positional_options_description p;
		p.add("input",-1);
		//p.add("output",2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);    

		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		rtmath::ddscat::ddUtil::process_static_options(vm);
		rtmath::ddscat::shapeFileStats::process_static_options(vm);
		rtmath::ddscat::ddOutput::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		vector<string> vInput;
		string sOutput;
		bool doHash = false;
		if (vm.count("input")) vInput = vm["input"].as<vector<string> >();
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		if (vm.count("hash")) doHash = true;

		if (!vInput.size()) doHelp("Need to specify input directories");

		if (vInput.size() > 1 && vm.count("output")) doHelp("You can only specify "
			"an output file if there is a single input file.");

		for (const std::string &t : vInput)
		{
			using namespace boost::filesystem;
			path p(t);
			cerr << "Processing: " << t << endl;
			if (!is_directory(p)) continue;
			using namespace rtmath::ddscat;
			ddUtil::tagTARGETs(p);
			boost::shared_ptr<ddOutput> ddOut = ddOutput::generate(t);

			if (sOutput.size())
				ddOut->writeFile(sOutput);

			if (doHash)
				ddOut->writeToHash();
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


