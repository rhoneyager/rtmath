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
#include <rycurses/rycurses.h>
#include <rycurses/ui.h>
#include <rycurses/items.h>

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
		cerr << "rtmath-ddscat-output-ingest-ncurses\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options"), cmdline("Command-line options"), 
			config("Config options"), hidden("Hidden options"), oall("all options");
		//rtmath::ddscat::shapeFileStats::add_options(cmdline, config, hidden);
		rtmath::debug::add_options(cmdline, config, hidden);
//		Ryan_Serialization::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddUtil::add_options(cmdline, config, hidden);
		rtmath::ddscat::ddOutput::add_options(cmdline, config, hidden);

		cmdline.add_options()
			("help,h", "produce help message")
			("input,i", po::value<vector<string> >()->multitoken(),"specify input directory or file.")
			("output,o", po::value<string>(), "specify output directory or file")
			//("tag,t", po::value<vector<string> >(), "Add extra information to output file")
			//("description,d", po::value<string>(), "Describe the output file")
			//("hostname,H", po::value<string>(), "hostname of system used to generate data")

			("write-oris", po::value<bool>()->default_value(true), "Write ORI data into output file")
			("write-fmls", po::value<bool>()->default_value(true), "Write FML data into output file")
			("write-shapes", po::value<bool>()->default_value(false), "Write shapefile data into output file")
			;

		desc.add(cmdline).add(config);
		oall.add(cmdline).add(config).add(hidden);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(oall).run(), vm);
		po::notify(vm);

		rtmath::debug::process_static_options(vm);
		rtmath::ddscat::ddUtil::process_static_options(vm);
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

		bool writeORI = vm["write-oris"].as<bool>();
		bool writeFML = vm["write-fmls"].as<bool>();
		bool writeShapes = vm["write-shapes"].as<bool>();
		//string exportType = vm["output-export-type"].as<string>();

		auto opts = rtmath::registry::IO_options::generate();
		opts->filename(sOutput);
		opts->setVal<bool>("writeORI", writeORI);
		opts->setVal<bool>("writeFML", writeFML);
		opts->setVal<bool>("writeSHP", writeShapes);
		//opts->exportType(exportType);
		if (boost::filesystem::exists(boost::filesystem::path(sOutput))) opts->iotype(rtmath::registry::IOhandler::IOtype::READWRITE);

		std::shared_ptr<rtmath::registry::IOhandler> writer;
		std::shared_ptr<rtmath::registry::DBhandler> dHandler;

		auto conf = rtmath::config::loadRtconfRoot();
		auto freqmap = conf->getChild("ddscat")->getChild("freqs")->listKeys();

		using std::vector;
		using namespace rtmath::ddscat;

		auto collection = ddOutput::makeCollection();
		auto query = ddOutput::makeQuery();
		bool supplementDb = false;
		bool fromDb = false;


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

		using namespace rycurses;

		shared_ptr<ncUi> wndInfo = ncUi::create(30,70,1,1);
		auto lblFname = ncLabel::create(wndInfo, "lblFilename", "Filename: ");
		lblFname->resize(2,2,10,1);
		auto lblFnameV = ncLabel::create(wndInfo, "lblFilenameV", "");
		lblFnameV->resize(13,2,50,3);
		lblFnameV->multiline(true);
		auto lblHostname = ncLabel::create(wndInfo, "lblHostname", "Hostname: ");
		lblHostname->resize(2,5,10,1);
		auto txtHostname = ncTextBox::create(wndInfo, "txtHostname", "");
		txtHostname->resize(13,5,30,1);
		auto lblDDVER = ncLabel::create(wndInfo, "lblDDVER", "DDSCAT Version: ");
		lblDDVER->resize(2,6,16,1);
		auto txtDDVER = ncTextBox::create(wndInfo, "txtDDVER", "");
		txtDDVER->resize(19,6,30,3);
		txtDDVER->multiline(true);
		auto lblDecim = ncLabel::create(wndInfo, "lblDecim", "Decimation: ");
		lblDecim->resize(2,10,14,1);
		auto txtDecim = ncTextBox::create(wndInfo, "txtDecim", "none");
		txtDecim->resize(17,10,16,1);
		auto lblPert = ncLabel::create(wndInfo, "lblPert", "Perturbation: ");
		lblPert->resize(2,11,14,1);
		auto txtPert = ncTextBox::create(wndInfo, "txtPert", "none");
		txtPert->resize(17,11,14,1);
		auto cmdOK = ncCmd::create(wndInfo, "cmdOK", "Ingest");
		cmdOK->resize(3,15,12,1);
		auto cmdSkip = ncCmd::create(wndInfo, "cmdSkip", "Skip");
		cmdSkip->resize(3,16,12,1);
		auto lblNumRuns = ncLabel::create(wndInfo, "lblNumRuns", "Number of runs: ");
		lblNumRuns->resize(2,12,16,1);
		auto lblNumRunsV = ncLabel::create(wndInfo, "lblNumRunsV", "0");
		lblNumRunsV->resize(19,12,5,1);
		auto lblStatus = ncLabel::create(wndInfo, "lblStatus", "Loading file...");
		lblStatus->resize(7,18,30,1);

		auto it = vsInput.begin();
		vector<boost::shared_ptr<ddOutput> > runs;

		auto updateScreen = [&](const std::string &filename)
		{
			lblFnameV->text = filename;
			lblNumRunsV->text = boost::lexical_cast<std::string>(runs.size());;
			txtHostname->text = hostname;
			lblStatus->text = "";
			wndInfo->refresh();
			wndInfo->itemFocus();
		};
		auto loadItem = [&]()
		{
			using namespace boost::filesystem;
			path ps = expandSymlinks(*it);

			auto iopts = rtmath::registry::IO_options::generate();
			iopts->filename(*it);
			// Handle not needed as the read context is used only once.
			if (ddOutput::canReadMulti(nullptr, iopts)) {
				ddOutput::readVector(nullptr, iopts, runs, query);
			}
			updateScreen(*it);
		};
		auto nextItem = [&]()
		{
			if (it == vsInput.end())
			{
				lblStatus->text = "Finished";
				wndInfo->refresh();
				std::cout << "Completed" << std::endl;
				exit(0);
			} else {
				lblStatus->text = "Loading Item...";
				wndInfo->refresh();
				runs.clear();
				loadItem();
			}
		};
		auto skipItem = [&](ncCmd*) -> bool
		{
			++it;
			nextItem();
			return true;
		};
		auto processItem = [&](ncCmd*) -> bool
		{
			// Processing goes here
				lblStatus->text = "Committing Item...";

			// Increment
			++it;
			nextItem();
			return true;
		};
		cmdOK->funcClick(processItem);
		cmdSkip->funcClick(skipItem);

		wndInfo->itemFocus();
		wndInfo->focus();
		nextItem();
		wndInfo->beginMessageLoop();


		/*
		for (auto &r : *(res.first))
		{
			auto run = boost::const_pointer_cast<ddOutput>(r);

			//cerr << "Run: " << run->genName() << endl;
			//std::cerr << " Frequency: " << run->freq << " GHz\n";
			//std::cerr << " Temperature: " << run->temp << " K." << std::endl;

			//if (sDesc.size())
			//	run->description = sDesc;
			//run->hostname = hostname;
			//for (auto &t : tags)
			//	run->tags.insert(t);

			auto doWrite = [&](std::shared_ptr<rtmath::registry::IO_options> &oopts, std::shared_ptr<rtmath::registry::IOhandler> &w)
			{
				if (!oopts->filename().size()) return;

				// Check for the existence of an output directory
				w = run->writeMulti(w, oopts);
				}

			};

			if (sOutput.size())
				doWrite(opts, writer);

			// Drop run from memory
			run.reset(); // update collection result
		}
	*/

	} catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


