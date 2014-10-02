/* This program is designed to create and extract from ddOutput files. */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <Ryan_Debug/debug.h>
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/hash.h"
#include "../../rtmath/rtmath/config.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOriData.h"
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
//		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		rtmath::ddscat::stats::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >()->multitoken(),"specify input directory or file.")
			("output,o", po::value<string>(), "specify output directory or file")
			("output-export-type", po::value<string>()->default_value(""), "Identifier to export (i.e. isotropic_data, orientation_data)")
			("output-aux", po::value<string>(), "optional auxiliary output file (with separate writing options)")
			("output-shape", "If writing an output directory, also write the shape.")
			//("hash,s", "Store ddscat output in the hash store")
			("tag,t", po::value<vector<string> >(), "Add extra information to output file")
			("description,d", po::value<string>(), "Describe the output file")
			("hostname,H", po::value<string>(), "hostname of system used to generate data")
			("directory,D", "Write as a directory")

			("write-oris", po::value<bool>()->default_value(true), "Write ORI data into output file")
			("write-fmls", po::value<bool>()->default_value(true), "Write FML data into output file")
			("write-fmls-aux", po::value<bool>()->default_value(false), "Write FML data into aux output file")
			("write-shapes", po::value<bool>()->default_value(true), "Write shapefile data into output file")

			("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
			("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
			("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
			("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
			("match-dipole-spacing", po::value<vector<std::string> >()->multitoken(), "Match typical dipole spacings")
			("match-dipole-numbers", po::value<vector<std::string> >()->multitoken(), "Match typical dipole numbers")
			("match-frequency", po::value<vector<std::string> >()->multitoken(), "Match frequencies")
			("match-aeff", po::value<vector<std::string> >()->multitoken(), "Match effective radii")
			("match-temp", po::value<vector<std::string> >()->multitoken(), "Match temperatures")
			("match-betas", po::value<vector<std::string> >()->multitoken(), "Match beta rotation count")
			("match-thetas", po::value<vector<std::string> >()->multitoken(), "Match theta rotation count")
			("match-phis", po::value<vector<std::string> >()->multitoken(), "Match phi rotation count")
			("match-pol", po::value<vector<std::string> >()->multitoken(), "Match polarization")
			("match-run-uuid", po::value<vector<std::string> >()->multitoken(), "Match run id")
			("update-db", "Insert shape file entries into database")

			("hash-output", "Store flake data in hash directory")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		//Ryan_Serialization::process_static_options(vm);
		rtmath::ddscat::ddUtil::process_static_options(vm);
		rtmath::ddscat::stats::shapeFileStats::process_static_options(vm);
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

		string hostname;
		if (vm.count("hostname")) hostname = vm["hostname"].as<string>();
		vector<string> vsInput;
		string sOutput, sOutputAux;
		vector<std::pair<string,string> > tags;
		vector<string> tags_pre;
		string sDesc;
		if (vm.count("description")) sDesc = vm["description"].as<string>();
		if (vm.count("tag")) tags_pre = vm["tag"].as<vector<string> >();
		{ // do tag splitting
			vector<string> out;
			for (const auto &v : tags_pre)
			{
				rtmath::config::splitVector(v, out, '=');
				if (out.size() < 2) out.push_back("");
				tags.push_back(std::pair<string, string>(out[0], out[1]));
				out.clear();
			}
		}
		if (vm.count("input")) vsInput = vm["input"].as<vector<string> >();
		else doHelp("Need to specify input");
		if (vm.count("output")) sOutput = vm["output"].as<string>();
		if (vm.count("output-aux")) sOutputAux = vm["output-aux"].as<string>();
		//else if (!doHash)
		//	doHelp("Need to specify output");
		bool fromSummary = false;
		if (vm.count("from-summary-files")) fromSummary = true;

		bool writeORI = vm["write-oris"].as<bool>();
		bool writeFML = vm["write-fmls"].as<bool>();
		bool writeFMLaux = vm["write-fmls-aux"].as<bool>();
		bool writeShapes = vm["write-shapes"].as<bool>();
		string exportType = vm["output-export-type"].as<string>();

		auto opts = rtmath::registry::IO_options::generate();
		opts->filename(sOutput);
		opts->setVal<bool>("writeORI", writeORI);
		opts->setVal<bool>("writeFML", writeFML);
		opts->setVal<bool>("writeSHP", writeShapes);
		opts->exportType(exportType);
		if (boost::filesystem::exists(boost::filesystem::path(sOutput))) opts->iotype(rtmath::registry::IOhandler::IOtype::READWRITE);

		auto optsaux = rtmath::registry::IO_options::generate();
		optsaux->filename(sOutputAux);
		optsaux->setVal<bool>("writeORI", writeORI);
		optsaux->setVal<bool>("writeFML", writeFMLaux);
		optsaux->setVal<bool>("writeSHP", writeShapes);
		if (boost::filesystem::exists(boost::filesystem::path(sOutputAux))) optsaux->iotype(rtmath::registry::IOhandler::IOtype::READWRITE);

		std::shared_ptr<rtmath::registry::IOhandler> writer, writeraux;
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;
		/*
		("use-db", po::value<bool>()->default_value(true), "Use database to supplement loaded information")
		("from-db", po::value<bool>()->default_value(false), "Perform search on database and select files matching criteria.")
		("match-hash", po::value<vector<string> >()->multitoken(), "Match lower hashes")
		("match-flake-type", po::value<vector<string> >()->multitoken(), "Match flake types")
		("match-dipole-spacing", po::value<vector<std::string> >()->multitoken(), "Match typical dipole spacings")
		("match-dipole-numbers", po::value<vector<std::string> >()->multitoken(), "Match typical dipole numbers")
		("match-frequency", po::value<vector<std::string> >()->multitoken(), "Match frequencies")
		("match-aeff", po::value<vector<std::string> >()->multitoken(), "Match effective radii")
		("match-temp", po::value<vector<std::string> >()->multitoken(), "Match temperatures")
		("match-betas", po::value<vector<std::string> >()->multitoken(), "Match beta rotation count")
		("match-thetas", po::value<vector<std::string> >()->multitoken(), "Match theta rotation count")
		("match-phis", po::value<vector<std::string> >()->multitoken(), "Match phi rotation count")
		("match-pol", po::value<vector<std::string> >()->multitoken(), "Match polarization")
		("match-run-uuid", po::value<vector<std::string> >()->multitoken(), "Match run id")
		("update-db", "Insert shape file entries into database")

		("hash-output", "Store flake data in hash directory")
		*/
		auto conf = rtmath::config::loadRtconfRoot();
		//auto freqmap = conf->getChild("ddscat")->getChild("freqs")->listKeys();

		vector<string> matchHashes, matchFlakeTypes, matchPols, matchRunUuids;
		if (vm.count("match-hash")) matchHashes = vm["match-hash"].as<vector<string> >();
		if (vm.count("match-flake-type")) matchFlakeTypes = vm["match-flake-type"].as<vector<string> >();
		if (vm.count("match-pol")) matchPols = vm["match-pol"].as<vector<string> >();
		if (vm.count("match-run-uuid")) matchRunUuids = vm["match-run-uuid"].as<vector<string> >();
		rtmath::config::intervals<float> iDipoleSpacing, iFrequencies, iAeffs, iTemps;
		rtmath::config::intervals<size_t> iDipoleNumbers, iBetas, iThetas, iPhis;
		if (vm.count("match-dipole-spacing")) iDipoleSpacing.append(vm["match-dipole-spacing"].as<vector<string>>());
		if (vm.count("match-frequency")) iFrequencies.append(vm["match-frequency"].as<vector<string>>()); //, &freqmap);
		if (vm.count("match-aeff")) iAeffs.append(vm["match-aeff"].as<vector<string>>());
		if (vm.count("match-temp")) iTemps.append(vm["match-temp"].as<vector<string>>());
		if (vm.count("match-dipole-numbers")) iDipoleNumbers.append(vm["match-dipole-numbers"].as<vector<string>>());
		if (vm.count("match-betas")) iBetas.append(vm["match-betas"].as<vector<string>>());
		if (vm.count("match-thetas")) iThetas.append(vm["match-thetas"].as<vector<string>>());
		if (vm.count("match-phis")) iPhis.append(vm["match-phis"].as<vector<string>>());

		using std::vector;
		using namespace rtmath::ddscat;

		auto collection = ddOutput::makeCollection();
		auto query = ddOutput::makeQuery();
		query->hashLowers.insert(matchHashes.begin(), matchHashes.end());
		query->flakeTypes.insert(matchFlakeTypes.begin(), matchFlakeTypes.end());
		query->dipoleNumbers = iDipoleNumbers;
		query->freqRanges = iFrequencies;
		query->tempRanges = iTemps;
		query->aeffRanges = iAeffs;
		query->pol.insert(matchPols.begin(), matchPols.end());
		query->runids.insert(matchRunUuids.begin(), matchRunUuids.end());
		query->betaRanges = iBetas;
		query->thetaRanges = iThetas;
		query->phiRanges = iPhis;
		query->dipoleSpacings = iDipoleSpacing;
		
		bool supplementDb = false;
		supplementDb = vm["use-db"].as<bool>();
		bool fromDb = false;
		fromDb = vm["from-db"].as<bool>();


		auto dbcollection = ddOutput::makeCollection();
		auto qExisting = ddOutput::makeQuery();



		auto expandSymlinks = [](const boost::filesystem::path &p) -> boost::filesystem::path
		{
			using namespace boost::filesystem;
			if (is_symlink(p))
			{
				path pf = boost::filesystem::absolute(read_symlink(p), p.parent_path());
				return pf;
			}
			else {
				return p;
			}
		};

		using namespace boost::filesystem;
		for (const auto &i : vsInput)
		{
			cerr << "Processing " << i << endl;
			path ps = expandSymlinks(i);

			auto iopts = rtmath::registry::IO_options::generate();
			iopts->filename(i);
			// Handle not needed as the read context is used only once.
			if (is_directory(ps))
			{
				// Check for recursion
				path pd = ps / "shape.dat";
				if (!exists(pd))
				{
					// Load one level of subdirectories
					vector<path> subdirs;
					copy(directory_iterator(ps),
						directory_iterator(), back_inserter(subdirs));
					for (const auto &p : subdirs)
					{
						if (is_directory(p))
						{
							cerr << " processing " << p << endl;
							boost::shared_ptr<ddOutput> s(new ddOutput);
							s = ddOutput::generate(ps.string());
							if (query->filter(s.get()))
								collection->insert(s);
							//runs.push_back(s);
						}
					}
				} else {
					// Input is a ddscat run
					boost::shared_ptr<ddOutput> s(new ddOutput);
					s = ddOutput::generate(ps.string());
					if (query->filter(s.get()))
						collection->insert(s);
					//runs.push_back(s);
				}
			} else if (ddOutput::canReadMulti(nullptr, iopts)) {
				vector<boost::shared_ptr<ddOutput> > runs;
				ddOutput::readVector(nullptr, iopts, runs, query);
				for (auto &s : runs)
					collection->insert(s);
			}

		}

		// Perform the query, and then process the matched shapefiles
		auto res = query->doQuery(collection, fromDb, supplementDb, dHandler);


		for (auto &r : *(res.first))
		{
			auto run = boost::const_pointer_cast<ddOutput>(r);

			cerr << "Run: " << run->genName() << endl;
			std::cerr << " Frequency: " << run->freq << " GHz\n";
			std::cerr << " Temperature: " << run->temp << " K." << std::endl;

			if (sDesc.size())
				run->description = sDesc;
			run->hostname = hostname;
			for (auto &t : tags)
				run->tags.insert(t);

			// Pull in shape and stats in case they are needed during the write operation (i.e. exports)
			// If hash not found and calculations are prohibited, ignore (it's the plugin's job to check).
			try {
				if (writeShapes)
					run->loadShape(false);
			} catch (rtmath::debug::xMissingHash &e) {
				std::cerr << e.what() << std::endl;
			}

			// TODO: Add Hash Support
			//if (doHash)
			//	run->writeToHash();

			// TODO: update database

			auto doWrite = [&](std::shared_ptr<rtmath::registry::IO_options> &oopts, std::shared_ptr<rtmath::registry::IOhandler> &w)
			{
				if (!oopts->filename().size()) return;

				bool writeDir = false;
				// Check for the existence of an output directory
				path pOut(oopts->filename());
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
					//cerr << "Expanding into directory " << sOutput << endl;
					bool outShape = false;
					if (vm.count("output-shape")) outShape = true;
					run->expand(oopts->filename(), outShape);
				}
				else {
					//cerr << "Writing file " << sOutput << endl;
					if (sOutput == vsInput.at(0))
					{
						cerr << "Output is the same as the input. Doing nothing.\n";
						return;
					}

					w = run->writeMulti(w, oopts);


					//ddOut->writeFile(sOutput);
				}

			};

			if (sOutput.size())
				doWrite(opts, writer);
			if (sOutputAux.size())
				doWrite(optsaux, writeraux);


			// Drop run from memory
			run.reset(); // update collection result
		}

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


