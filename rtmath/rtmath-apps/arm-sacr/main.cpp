/* This program is designed to process scanning radar data from ARM */

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
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
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
		cerr << "rtmath-arm-sacr\n\n";

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
			("export-types,e", po::value<vector<string> >()->multitoken(), "Identifiers to export (i.e. reflectivity)")
			("show-summary", po::value<bool>()->default_value(true), "Show summary of analysis")
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
		bool summary = vm["show-summary"].as<bool>();
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
			if (is_symlink(pi)) pi = boost::filesystem::read_symlink(pi);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(si.string().c_str());
			if (is_directory(pi)) continue;

			//cerr << "Input: " << si << endl;
			cerr << pi.filename() << endl;

			using namespace rtmath::data::arm;
			boost::shared_ptr<arm_info> im;
			try {
				im = boost::shared_ptr<arm_info>( new arm_info(si.string()) );
			} catch (...) {
				cerr << "Error processing file. Skipping." << endl;
				continue;
			}

			if (summary)
				cerr << "\t" << im->site << "\t" << im->subsite << "\n\t"
				<< im->product << "\t" << im->stream << "\t" << im->datalevel << "\n\t"
				<< im->startTime << "\t" << im->endTime << "\t" << "\n\t"
				<< im->lat << "\t" << im->lon << "\t" << im->alt << endl;

			if (im->product.find("sacr") == string::npos)
			{
				cerr << "File does not contain SACR information. Skipping." << endl;
				continue;
			}

			//boost::shared_ptr<dataStreamHandler> loadedData = im->getHandler();
			boost::shared_ptr<arm_scanning_radar_sacr> data (new arm_scanning_radar_sacr(si.string()));
			//	= boost::dynamic_pointer_cast<arm_scanning_radar_sacr>(loadedData);

			for (size_t i = 0; i < exportTypes.size(); ++i)
			{
				auto opts = registry::IO_options::generate();
				boost::filesystem::path p("."), pextless(pi.filename());
				pextless.replace_extension();
				if (output.size()) p = boost::filesystem::path(output);
				p = p / pextless;
				p += ".";
				p += exportTypes[i];
				p += boost::filesystem::path(".tsv");
				opts->filename(p.string());
				opts->exportType(exportTypes[i]);
				opts->setVal<std::string>("source", si.string());
				try {
					try {
						if (data->canWriteMulti(exportHandlers[i], opts))
						{
							size_t pass = 0;
							//for (size_t pass=0; pass < (size_t) data->sweep_start_ray_index.size(); ++pass)
							{
								opts->setVal<size_t>("pass", pass);
								exportHandlers[i] = data->writeMulti(exportHandlers[i], opts);
							}
						}
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

