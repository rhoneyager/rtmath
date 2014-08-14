/* This program is designed to index arm data. */

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

enum class linkMethod { HARD, SOFT, COPY };


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
			("show-summary", po::value<bool>()->default_value(true), "Show summary of analysis")
			("show-index-location", "Option to generate a unique folder name for indexing / storing the file consistently")
			("index-base", po::value<string>(), "If set, link files into an indexed directory tree, starting at index-base."
			"Linking / copying behavior is set by the link-method flag")
			("link-method", po::value<string>()->default_value("hard,copy"), "Sets link method attempted order. \"hard,copy\" "
			"means that hard links will be attempted first. If these fail, then fall back to just copying the file. "
			"Options are combinations of hard,soft,copy.")
			//("export,e", po::value<string>(), "Export filename (all shapes are combined into this)")
			("update-db", "Insert arm file entries into database")
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

		vector<string> vslinkMethods;
		vector<linkMethod> linkMethods;
		string slinkMethods = vm["link-method"].as<string>();
		config::splitVector(slinkMethods, vslinkMethods, ',');
		for (const auto &slm : vslinkMethods)
		{
			if (slm == "hard") linkMethods.push_back(linkMethod::HARD);
			if (slm == "soft") linkMethods.push_back(linkMethod::SOFT);
			if (slm == "copy") linkMethods.push_back(linkMethod::COPY);
		}
		string sindexBase;
		if (vm.count("index-base")) sindexBase = vm["index-base"].as<string>();
		path indexBase(sindexBase);

		vector<string> exportTypes;
		if (vm.count("export-types")) exportTypes = vm["export-types"].as<vector<string> >();

		if (exportTypes.size() && !vm.count("output-prefix")) doHelp("Need to specify output file prefix.\n");

		std::vector<std::shared_ptr<rtmath::registry::IOhandler> > exportHandlers(exportTypes.size());
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;

		bool recurse = false;
		if (vm.count("recursive")) recurse = true;
		bool index = false;
		if (vm.count("show-index-location")) index = true;
		bool summary = vm["show-summary"].as<bool>();

		vector<boost::filesystem::path> vinputs;
		rtmath::debug::expandFolders(inputs, vinputs, recurse);

		auto dbcollection = rtmath::data::arm::arm_info::makeCollection();

		for (const auto &si : vinputs)
		{
			// Validate input file
			path pi(si);
			if (!exists(pi)) throw rtmath::debug::xMissingFile(si.string().c_str());
			pi = rtmath::debug::expandSymlink(pi);
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

			//boost::shared_ptr<dataStreamHandler> loadedData;
			//loadedData = im->getHandler();

			string sindexLocation = im->indexLocation();
			path indexLocation(sindexLocation);
			if (index) cout << "\t" << sindexLocation << endl;
			if (sindexBase.size())
			{
				try {
					path pdir = indexBase / indexLocation;
					// Create this directory if not found
					if (!exists(pdir)) boost::filesystem::create_directories(pdir);
					if (!is_directory(pdir)) RTthrow debug::xPathExistsWrongType(pdir.string().c_str());

					path pfile = pdir / pi.filename();
					if (!exists(pfile)) {

						bool success = false;
						for (const auto & meth : linkMethods)
						{
							try {
								if (meth == linkMethod::HARD) {
									boost::filesystem::create_hard_link(pi, pfile);
									cerr << "Created hard link.\n";
									success = true;
									break;
								} else if (meth == linkMethod::SOFT) {
									boost::filesystem::create_symlink(pi, pfile);
									cerr << "Created symlink.\n";
									success = true;
									break;
								} else if (meth == linkMethod::COPY) {
									boost::filesystem::copy(pi, pfile);
									cerr << "Copied file.\n";
									success = true;
									break;
								}
							} catch (std::exception &e) {
								cerr << e.what() << endl;
								continue;
							}
						}
						if (!success) RTthrow debug::xUnsupportedIOaction("Cannot create indexed location");
					} else {
						cerr << "File at" << pfile << " already exists. Skipping." << endl;
					}
				} catch (...) {
					std::cerr << "Error indexing file. Skipping index operation." << std::endl;
					continue;
				}
			}

			if (vm.count("update-db"))
			{
				dbcollection->insert(im);

				if (dbcollection->size() > 500) // Do in batches. The final uneven batch is handled at the end of execution.
				{
					dHandler = data::arm::arm_info::updateCollection(dbcollection, 
						rtmath::data::arm::arm_info_registry::updateType::INSERT_ONLY, dHandler);
					dbcollection->clear();
				}
				//dHandler = im->updateEntry(rtmath::data::arm::arm_info_registry::updateType::INSERT_ONLY, dHandler);
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

		if (dbcollection->size())
		{
			dHandler = data::arm::arm_info::updateCollection(dbcollection,
				rtmath::data::arm::arm_info_registry::updateType::INSERT_ONLY, dHandler);
			dbcollection->clear();
			//dHandler = im->updateEntry(rtmath::data::arm::arm_info_registry::updateType::INSERT_ONLY, dHandler);
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

