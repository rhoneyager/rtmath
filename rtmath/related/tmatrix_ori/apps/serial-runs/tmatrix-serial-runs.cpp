#include <iostream>
// Using boost parameter processing this time to avoid rtmath dependencies
#include <boost/program_options.hpp>
#include <boost/serialization/vector.hpp>

#include <vector>
#include <string>
#include <Ryan_Debug/debug.h>
#include <Ryan_Serialization/serialization.h>
#include "../../src/headers/tmatrix.h"

int main(int argc, char *argv[])
{
	using namespace std;
	
	try {
		cerr << "tmatrix-serial-runs" << endl;

		// Do processing of argv
		namespace po = boost::program_options;
		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("input,i", po::value< vector<string> >(), "input file")
			("output,o", po::value<string>(), "Set output file (required)")
		;
		po::positional_options_description p;
		p.add("input", -1);

		po::variables_map vm;
		po::store(po::command_line_parser(argc,argv).
			options(desc).positional(p).run()
			, vm);
		po::notify(vm);

		auto doHelp = [&](const string &message)
		{
			cerr << desc << endl;
			if (message.size()) cerr << message << endl;
			exit(1);
		};

		// Begin checking parameters
		if (vm.count("help") || vm.size() == 0) doHelp("");

		string ofile;
		vector<string> files;
		if (vm.count("input"))
		{
			files = vm["input"].as< vector<string> >();
		}

		if (vm.count("output"))
		ofile = vm["output"].as<string>();

		
		cerr << "Input files are:";
		for (auto it = files.begin(); it != files.end(); it++)
			cerr << " " << *it;
		cerr << endl;

		cerr << "Outputting to: " << ofile << endl;

		// The actual runs begin here

		using namespace tmatrix;

		vector < boost::shared_ptr<::tmatrix::queue::TMrequest> > tms;
		for (auto it = files.begin(); it != files.end(); ++it)
		{
			vector< boost::shared_ptr<::tmatrix::queue::TMrequest> > loader;
			Ryan_Serialization::read< vector< boost::shared_ptr<::tmatrix::queue::TMrequest> > >(loader,*it);
			tms.resize(tms.size() + loader.size());
			std::copy(loader.begin(), loader.end(), tms.rbegin());
		}

		// Now loaded. Iterate through each tm and calculate.
		
		for (auto it = tms.begin(); it != tms.end(); ++it)
		{
			// Each tmatrixSet has a base set of tmatrixInVars
			// and a set of results (tmatrixAngleRes). Combine the 
			// two to form a valid tmatrix run and execute.
			for (auto ot = it->results.begin(); ot != it->results.end(); ++ot)
			{
				tmatrix::tmatrix run;
				run.vars = *(it->base);
				run.vars.ALPHA = (*ot)->alpha;
				run.vars.BETA = (*ot)->beta;
				run.vars.THET = (*ot)->theta;
				run.vars.THET0 = (*ot)->theta0;
				run.vars.PHI = (*ot)->phi;
				run.vars.PHI0 = (*ot)->phi0;

				run.run();
				(*ot)->res = run.outs;
			}
		}

		// Runs are done. Serialize output.

		Ryan_Serialization::write< vector< boost::shared_ptr<::tmatrix::queue::TM>> > (tms, ofile);

		return 0;

	} 
	catch (const std::exception &e)
	{
		cerr << "Exception: " << e.what() << endl;
		exit(2);
	}
	catch (...)
	{
		cerr << "Caught unidentified error... Terminating." << endl;
		exit(3);
	}
}


