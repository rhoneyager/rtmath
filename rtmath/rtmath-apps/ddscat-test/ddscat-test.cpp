/* This program is designed to compare two ddscat output files (typically .avg) and report 
 * changes that exceed a given threshold.
 */
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <string>
#include <Ryan-Debug/debug.h>
#include "../../rtmath/rtmath/ddscat/ddOutputSingle.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	try
	{
		using namespace std;
		cerr << "rtmath-ddscat-test\n\n";
		namespace po = boost::program_options;

		po::options_description desc("Allowed options");
		desc.add_options()
			("help,h", "produce help message")
			("base,b", po::value<string>(),"specify base file (the reference values)")
			("input,i", po::value<string>(), "specify input file (checking against these values)")
			("tolerance,t", po::value<double>()->default_value(5), "Percent tolerance for value checks")
			("tolerance-abs,a", po::value<double>()->default_value(1.e-6), "Tolerance for near-zero quantities")
			;

		po::positional_options_description p;
		p.add("base",1);
		p.add("input",2);

		po::variables_map vm;
		po::store(po::command_line_parser(argc, argv).
			options(desc).positional(p).run(), vm);
		po::notify(vm);

		auto doHelp = [&](const std::string &message)
		{
			cerr << message << endl;
			cerr << desc << endl;
			exit(1);
		};
		if (vm.count("help") || argc == 1) doHelp("");
		double tolerance, toleranceAbs;
		string sInput, sBase;
		if (vm.count("input")) sInput = vm["input"].as<string>();
		else doHelp("Need to specify input file");
		if (vm.count("base")) sBase = vm["base"].as<string>();
		else doHelp("Need to specify base file");

		tolerance = vm["tolerance"].as<double>();
		toleranceAbs = vm["tolerance-abs"].as<double>();

		int numFailures = 0;

		using namespace rtmath::ddscat;
		ddOutputSingle ddInput(sInput), ddBase(sBase);

		// Check header strings
		//TODO!

		// Check the stat tables
		ddOutputSingle::statTableType statsInput, statsBase;
		ddInput.getStatTable(statsInput);
		ddBase.getStatTable(statsBase);
		for (size_t j=0; j<(size_t) rtmath::ddscat::NUM_STAT_ENTRIES; j++)
		{
			double b = statsInput[j];
			double a = statsBase[j];
			// Check for tolerance in percentage and in abs terms.
			// If both criteria fail, then report a failure.
			if (abs( (a - b) / a) 
					* 100 < tolerance) continue;
			if (abs(a - b) < toleranceAbs) continue;
			// By here, a failure has occurred
			numFailures++;
			// Convert j to the appropriate stat table entry name and 
			// report the failure.
			std::cerr << "Mismatch in " 
				<< getStatNameFromId( (rtmath::ddscat::stat_entries) j)
				<< " - " << a << " versus " << b << std::endl;
		}

		// Check FML / Mueller matrices
		// Get collections of scattering matrices
		// Iterate through each collection. Check that type and values are the same.
		// Ordering should have been preserved.
		ddOutputSingle::scattMatricesContainer scattInput, scattBase;
		ddInput.getScattMatrices(scattInput);
		ddBase.getScattMatrices(scattBase);
		{
			auto it = scattInput.begin();
			auto ot = scattBase.begin();
			while (it != scattInput.end() && ot != scattBase.end())
			{
				if (it->id() != ot->id()) throw;
				// Check freq, theta, phi
				// Cast to correct subtype and compare entries
				++it; ++ot;
			}
		}

		//if (dispScat)
		{
			//cout << "Mueller matrix:\n";
			//ddfile.writeMueller(cout);
			//cout << endl;
		}

		if (numFailures)
		{
			cout << "There were " << numFailures << " failures." << endl;
			return 2;
		} else {
			cout << "All tests passed!" << endl;
		}
	}
	catch (std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return 1;
	}
	return 0;
}


