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
#include "../../rtmath/rtmath/ddscat/ddpar.h"
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
			("input,i", po::value<vector<string> >(),"specify input directory or file.")
			("from-summary-files", "Indicate the first input file is an avg file, the "
			"second is a par file, and the third is a shape file. Used for importing "
			"Holly runs.")
			("output,o", po::value<string>(), "specify output directory or file")
			("output-shape", "If writing an output directory, also write the shape.")
			("hash,s", "Store ddscat output in the hash store")
			("tag,t", po::value<vector<string> >(), "Add extra information to output file")
			("description,d", po::value<string>(), "Describe the output file")
			("directory,D", "Write as a directory")
			;

		po::positional_options_description p;
		//p.add("input",1);
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

		bool doHash = false;
		if (vm.count("hash")) doHash = true;

		vector<string> vsInput;
		string sOutput;
		vector<string> tags;
		string sDesc;
		if (vm.count("description")) sDesc = vm["description"].as<string>();
		if (vm.count("tag")) tags = vm["tag"].as<vector<string> >();
		if (vm.count("input")) vsInput = vm["input"].as<vector<string> >();
		else doHelp("Need to specify input");
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		//else if (!doHash)
		//	doHelp("Need to specify output");
		bool fromSummary = false;
		if (vm.count("from-summary-files")) fromSummary = true;

		auto opts = rtmath::registry::IO_options::generate();
		opts->filename(sOutput);
		std::shared_ptr<rtmath::registry::IOhandler> writer;

		using namespace boost::filesystem;
		for (const auto &i : vsInput)
		{
			path pInput(i);
			cerr << "Processing: " << pInput << endl;
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
			if (fromSummary)
			{
				// Input may use the alternate generator
				// Takes the format of avg file, par file, shape file
				// Will fail if the files are not the correct type
				boost::shared_ptr<ddOutputSingle> avg(new ddOutputSingle(vsInput[0]));
				boost::shared_ptr<ddPar> par(new ddPar(vsInput[1]));
				boost::shared_ptr<shapefile::shapefile> shp(new shapefile::shapefile(vsInput[2]));
				ddOut = ddOutput::generate(avg, par, shp);
				path pavg(vsInput[0]), ppar(vsInput[1]), pshp(vsInput[2]);
				path pbavg = absolute(pavg);
				path pbpar = absolute(ppar);
				path pbshp = absolute(pshp);
				ddOut->sources.insert(pbavg.string());
				ddOut->sources.insert(pbpar.string());
				ddOut->sources.insert(pbshp.string());
			} else if (is_directory(ps))
			{
				// Input is a ddscat run
				ddOut = ddOutput::generate(ps.string());
			} else if (Ryan_Serialization::known_format(ps)) {
				// Input may be a ddOutput file
				// Read will fail if it is not the right file type
				ddOut = boost::shared_ptr<ddOutput>(new ddOutput);
				ddOut->readFile(ps.string());
			} else doHelp("Unable to parse input expression.");

			if (sDesc.size())
				ddOut->description = sDesc;
			for (auto &t : tags)
				ddOut->tags.insert(t);

			if (doHash)
				ddOut->writeToHash();
			if (!sOutput.size()) return 0;

			bool writeDir = false;
			// Check for the existence of an output directory
			path pOut(sOutput);
			if (exists(pOut))
			{
				path pOutS = expandSymlinks(pOut);
				if (is_directory(pOutS)) writeDir = true;
			}

			// Check the output extension
			if (vm.count("directory")) writeDir = true;
			//if (!Ryan_Serialization::known_format(sOutput)) writeDir = true;

			if (writeDir)
			{
				cerr << "Expanding into directory " << sOutput << endl;
				bool outShape = false;
				if (vm.count("output-shape")) outShape = true;
				ddOut->expand(sOutput, outShape);
			} else {
				cerr << "Writing file " << sOutput << endl;
				if (sOutput == vsInput.at(0))
				{
					cerr << "Output is the same as the input. Doing nothing.\n";
					return 0;
				}

				writer = ddOut->writeMulti(writer, opts);
				//ddOut->writeFile(sOutput);
			}

			if (fromSummary) break;
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


