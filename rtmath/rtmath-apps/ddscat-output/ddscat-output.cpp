/* This program is designed to create and extract from ddOutput files. */
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
		cerr << "rtmath-ddscat-output\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<string>(),"specify input directory or file")
			("output,o", po::value<string>(), "specify output directory or file")
			("output-shape", "If writing an output directory, also write the shape.")
			;

		po::positional_options_description p;
		p.add("input",1);
		p.add("output",2);

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);    

		rtmath::debug::process_static_options(vm);
		Ryan_Serialization::process_static_options(vm);
		rtmath::ddscat::ddUtil::process_static_options(vm);
		//rtmath::ddscat::shapeFileStats::process_static_options(vm);
		rtmath::ddscat::ddOutput::process_static_options(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		string sInput;
		string sOutput;
		if (vm.count("input")) sInput = vm["input"].as<string>();
		else doHelp("Need to specify input");
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		else ("Need to specify output");

		using namespace boost::filesystem;
		path pInput(sInput);
		cerr << "Processing: " << sInput << endl;
		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				return pf;
			} else {
				return p;
			}
		};
		path ps = expandSymlinks(pInput);
		
		using namespace rtmath::ddscat;
		boost::shared_ptr<ddOutput> ddOut;
		if (is_directory(ps))
		{
			// Input is a ddscat run
			ddOut = ddOutput::generate(ps.string());
		} else {
			// Input may be a ddOutput file
			// Read will fail if it is not the right file type
			ddOut = boost::shared_ptr<ddOutput>(new ddOutput);
			ddOut->readFile(ps.string());
		}

		bool writeDir = false;
		// Check for the existence of an output directory
		path pOut(sOutput);
		if (exists(pOut))
		{
			path pOutS = expandSymlinks(pOut);
			if (is_directory(pOutS)) writeDir = true;
		}

		// Check the output extension
		if (!Ryan_Serialization::known_format(sOutput)) writeDir = true;

		if (writeDir)
		{
			cerr << "Expanding into directory " << sOutput << endl;
			bool outShape = false;
			if (vm.count("output-shape")) outShape = true;
			ddOut->expand(sOutput, outShape);
		} else {
			cerr << "Writing file " << sOutput << endl;
			if (sOutput == sInput)
			{
				cerr << "Output is the same as the input. Doing nothing.\n";
				return 0;
			}
			ddOut->writeFile(sOutput);
		}


	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


