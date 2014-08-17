/* This program is designed to index arm data by unifying data intervals. */

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
		cerr << "rtmath-arm-coverage\n\n";

		namespace po = boost::program_options;

		po::positional_options_description p;
		p.add("input", -1);

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		rtmath::debug::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("site", po::value<string>(), "Select a single site")
			("subsite", po::value<string>(), "Select a single subsite (requires site)")
			("product", po::value<string>(), "Select a single product")
			("stream", po::value<string>(), "Select a single stream")
			("datalevel", po::value<string>(), "Select a single datalevel")
			("only-extract", "Only export the table, do not attempt to update it")
			("output,o", po::value< string >(), "output filename")
			("export-types,e", po::value<vector<string> >()->multitoken(), "Identifiers to export (i.e. image_stats)")
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


		string output;
		if (vm.count("output"))
		{
			output= vm["output"].as< string >();
			using namespace boost::filesystem;
			path pOut(output);
		}

		if (exportTypes.size() && !vm.count("output")) doHelp("Need to specify output filename.");

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
		auto qExisting = rtmath::data::arm::arm_info::makeQuery();

		// Search first for matching files already in the database. Ignore these.
		for (const auto &si : vinputs)
		{
			// Validate input file
			path pi(si);
			if (!exists(pi)) continue;
			pi = rtmath::debug::expandSymlink(pi);
			if (!exists(pi)) continue;
			if (is_directory(pi)) continue;

			qExisting->filename(pi.filename().string());
		}
		std::pair<rtmath::data::arm::arm_info_registry::arm_info_index::collection, 
			std::shared_ptr<rtmath::registry::DBhandler> > 
			existing = qExisting->doQuery(dHandler);
		// And use a map for easy retrieval of existing entries
		std::map<std::string, std::shared_ptr<rtmath::data::arm::arm_info> > cExist;
		for (const auto &i : *(existing.first))
		{
			cExist[i->filename] = i;
		}
		dHandler = existing.second;

		for (const auto &si : vinputs)
		{
			// Validate input file
			path pi(si);
			if (!exists(pi)) {
				cerr << "Missing file: " << si.string() << std::endl;
				continue;
			}
			pi = rtmath::debug::expandSymlink(pi);
			if (!exists(pi)) {
				cerr << "Missing file: " << si.string() << std::endl;
				continue;
			}
			if (is_directory(pi)) continue;

			//cerr << "Input: " << si << endl;
			cerr << pi.filename() << endl;

			using namespace rtmath::data::arm;
			std::shared_ptr<arm_info> im;
			bool inDb = false;
			if (cExist.count(pi.filename().string()))
			{
				im = cExist.at(pi.filename().string());
				inDb = true;
			} else {
				try {
					im = std::shared_ptr<arm_info>(new arm_info(si.string()));
				}
				catch (...) {
					cerr << "Error processing file. Skipping." << endl;
					(*err) << "\t" << si.string() << std::endl;
					continue;
				}
			}

			if (summary)
				cerr << "\t" << im->site << "\t" << im->subsite << "\n\t"
				<< im->product << "\t" << im->stream << "\t" << im->datalevel << "\n\t"
				<< im->startTime << "\t" << im->endTime << endl;

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

			if (vm.count("update-db") && !inDb)
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
		if (sErrors.size())
			if (err) delete err;
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

