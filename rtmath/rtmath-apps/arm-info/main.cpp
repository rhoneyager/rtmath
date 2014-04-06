/* This program is designed to manipulate images. */

#pragma warning( push )
#pragma warning( disable : 4996 ) // Dumb boost uuid warning
#pragma warning( disable : 4800 ) // forcing non-bool type to true or false
#pragma warning( disable : 4521 ) // multiple copy constructors in some PCL stuff
#pragma warning( disable : 4244 ) // warning C4244: '=' : conversion from 'double' to 'float', possible loss of data in FLANN
#pragma warning( disable : 4068 ) // unknown gcc pragmas

#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <algorithm>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/shared_ptr.hpp>

#include <Ryan_Debug/debug.h>

#pragma warning( pop ) 
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-arm-info\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >()->multitoken(), "input files")
			("recursive,r", "Select recursive directory input")
			("output-prefix,o", po::value< string >(), "output file prefix")
			("export-types,e", po::value<vector<string> >()->multitoken(), "Identifiers to export (i.e. image_stats)")
			//("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).positional(p).run(), vm);
		po::notify(vm);  

		auto doHelp = [&](const std::string &message)
		{
			cerr << desc << "\n";
			if (message.size())
				cerr << message << "\n";
			exit(1);
		};

		if (vm.count("help") || argc == 1) doHelp("");

		rtmath::debug::process_static_options(vm);

		if (!vm.count("input")) doHelp("Need to specify input file(s).\n");

		vector<string > inputs = vm["input"].as< vector<string > >();

		string output;
		if (vm.count("output-prefix"))
		{
			output= vm["output-prefix"].as< string >();
			using namespace boost::filesystem;
			path pOut(output);
			if (!boost::filesystem::exists(pOut)) boost::filesystem::create_directory(pOut);
			else if (!boost::filesystem::is_directory(pOut))
				RTthrow rtmath::debug::xPathExistsWrongType(output.c_str());
		}

		vector<string> exportTypes;
		if (vm.count("export-types")) exportTypes = vm["export-types"].as<vector<string> >();

		if (exportTypes.size() && !vm.count("output-prefix")) doHelp("Need to specify output file prefix.\n");

		std::vector<std::shared_ptr<rtmath::registry::IOhandler> > exportHandlers(exportTypes.size());

		bool recurse = false;
		if (vm.count("recursive")) recurse = true;

		auto expandFolders = [&](const vector<string> &src, vector<path> &dest)
		{
			dest.clear();
			for (auto s : src)
			{
				using namespace boost::filesystem;
				path p(s);
				if (is_directory(p))
				{
					if (!recurse)
						copy(directory_iterator(p), 
						directory_iterator(), back_inserter(dest));
					else
						copy(recursive_directory_iterator(p,symlink_option::recurse), 
						recursive_directory_iterator(), back_inserter(dest));
				}
				else dest.push_back(p);
			}
		};

		vector<boost::filesystem::path> vinputs;
		expandFolders(inputs, vinputs);

		for (const auto &si : vinputs)
		{
			// Validate input file
			path pi(si);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(si.string().c_str());
			cerr << "Input file is: " << si << endl;

			using namespace rtmath::data::arm;
			boost::shared_ptr<arm_info> im;
			try {
				im = boost::shared_ptr<arm_info>( new arm_info(si.string()) );
			} catch (...) {
				std::cerr << "Error processing " << si << ". Skipping." << std::endl;
				continue;
			}

			if (output.size())
			{
				boost::filesystem::path pout(output);
				pout = pout / pi.filename();
				pout += ".xml";
				im->write(pout.string());
			}
			for (size_t i = 0; i < exportTypes.size(); ++i)
			{
				auto opts = registry::IO_options::generate();
				boost::filesystem::path p(".");
				if (output.size()) p = boost::filesystem::path(output);
				p = p / exportTypes[i];
				p += boost::filesystem::path(".tsv");
				opts->filename(p.string());
				opts->exportType(exportTypes[i]);
				opts->setVal<std::string>("source", si.string());
				try {
					try {
						if (im->canWriteMulti(exportHandlers[i], opts))
							exportHandlers[i] = im->writeMulti(exportHandlers[i], opts);
						else 
							std::cerr << "Error: unknown export type " << exportTypes[i] 
						<< " when writing " << p.string() << endl;
					} catch (rtmath::debug::xUnknownFileFormat &e) {
						std::cerr << "Error: unknown file format when writing " << p.string() << endl;
						std::cerr << e.what() << endl;
					}
				}
				catch (rtmath::debug::xUnknownFileFormat &e)
				{
					std::cerr << "Error: unknown file format when writing " << p.string() << endl;
					std::cerr << e.what() << endl;
				}
			}
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		cerr << endl;
		return 1;
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

