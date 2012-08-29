#include <iostream>
#include <complex>
#include <time.h>
//#include <netcdf.h>
//#include <netcdfcpp.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <omp.h>
#include <memory>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp> // used for location of output of netcdf

#include "../../rtmath/rtmath/command.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/ddscat/ddOutputEnsemble.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::ddscat;
	using namespace boost::filesystem;

	try {
		rtmath::debug::appEntry(argc, argv);
		
		// Take ddscat name or path from argv, and attempt to load the files
		set<double> orientations; // Contains orientation standard deviations
		bool allFreqs = true;
		set<double> frequencies;
		string ddPath = "./ddscat.par";
		size_t varying_coord = 2;
		string outprefix = "ddparse-";
		string outFormat = "csv";
		bool outCSV = true;
		// Parse parameters
		{
			config::parseParams p(argc, argv);
			bool flag = false;
			
			// Display help?
			if (p.readParam("-h")) doHelp();

			// Input path
			p.readParam<string>("-i",ddPath);

			// Output Format
			p.readParam<string>("-of",outFormat);

			// Output prefix
			p.readParam<string>("-p",outprefix);

			// Frequencies to analyze
			vector<string> freqList;
			flag = p.readParam<string>("-f",freqList);
			if (flag)
			{
				allFreqs = false;
				for (auto it = freqList.begin(); it != freqList.end(); it++)
					rtmath::config::splitSet<double>(*it,frequencies);
			}

			// Orientations to consider
			vector<string> vOri;
			flag = p.readParam<string>("-o",vOri);
			if (flag)
			{
				for (auto it = vOri.begin(); it != vOri.end(); it++)
					rtmath::config::splitSet<double>(*it,orientations);
			} else {
				cerr << "Must list orientations to consider.\n";
				doHelp();
			}

			// Which coord. contains orientation information
			string varying = "phi";
			p.readParam<string>("-v",varying);
			// Convert to lower case, then match up to the coords number
			std::transform(varying.begin(),varying.end(),varying.begin(),::tolower);
			if (varying == "beta") varying_coord = 1;
			else if (varying == "theta") varying_coord = 2;
			else if (varying == "phi") varying_coord = 3;
			else {
				cerr << "Unknown varying coordinate.\n";
				doHelp();
			}
		}

		// Parameters have been parsed. Now to invoke the ddscat-handling functions.
		// Constructor automatically loads all files.
		ddOutput src(ddPath);

		// If all frequencies allowed, take from list of frequencies in src.
		if (allFreqs) src.freqs(frequencies);

		// Loop through orientations
		// Depending on value, take isotropic, gaussian or fully-aligned cases
		for (auto it = orientations.begin(); it != orientations.end(); ++it)
		{
			cout << "Performing orientation at " << *it << endl;
			// Resultant ddOutputSingle
			ddOutputSingle res;

			// Construct orientation provider
			shared_ptr<ddscat::ddOutputEnsemble> oProv;
			// Based on orientation value, select the provider.
			if (*it < 0) // Isotropic provider
			{
				oProv = shared_ptr<ddscat::ddOutputEnsemble>
					(new ddscat::ddOutputEnsembleIsotropic(varying_coord));
			} else if (*it == 0) // Fully-aligned provider
			{
				oProv = shared_ptr<ddscat::ddOutputEnsemble>
					(new ddscat::ddOutputEnsembleAligned());
			} else { // Gaussian provider
				oProv = shared_ptr<ddscat::ddOutputEnsemble>
					(new ddscat::ddOutputEnsembleGaussian(*it,varying_coord));
			}

			// Have provider access ddOutput src and generate ensemble
			src.ensemble(*oProv,res);

			// Write the ensemble results, interpolating wherever necessary
			ostringstream outname;
			outname << outprefix << *it << "." << outFormat;
			res.write(outname.str());
		} // end orientation loop

	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		exit(1);
	}
	return 0;
}

void doHelp()
{
	using namespace std;
	cerr << "ddparse\n";
	cerr << "A program for parsing ddscat output and generating ensembles.\n";
	cerr << "Flags:\n";
	cerr << "-h\n";
	cerr << "\tProduce this help message.\n";
	cerr << "-i\n";
	cerr << "\tSpecify input ddscat.par. If not specified, defaults to execution directory.\n";
	cerr << "-of\n";
	cerr << "\tSpecify output format (csv, nc, evans, ...). Default is csv.\n";
	cerr << "-p\n";
	cerr << "\tSpecify prefix for output files. Default is 'ddparse-'.\n";
	cerr << "-f\n";
	cerr << "\tSpecify frequencies to consider (in GHz). Default is all.\n";
	cerr << "-o\n";
	cerr << "\tSpecify orientations to generate.\n";
	cerr << "\t0 indicates fully-aligned case.\n";
	cerr << "\tless than zero is isotropic.\n";
	cerr << "\tall others are standard deviations for the Gaussian case.\n";
	cerr << "-v\n";
	cerr << "\tSpecify whether varying coordinate for ensembles is 'theta' or 'phi'\n";
	cerr << "\tDefaults to phi.\n";
	exit(1);
}

