#include <iostream>
#include <complex>
#include <time.h>
#include <math.h>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <omp.h>
#include <memory>
#include <string>
#include <algorithm>
#include <boost/filesystem.hpp> // used for location of output of netcdf

#include "../../rtmath/rtmath/rtmath.h"

void doHelp();

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::ddscat;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-mie" << endl;
		rtmath::debug::appEntry(argc, argv);
		
		// Take ddscat name or path from argv, and attempt to load the files
		set<double> sizes; // Contains particle sizes
		set<double> frequencies;
		bool suppressOutput = false;
		string outprefix = "mie-";
		string outFormat = "csv";
		// Parse parameters
		{
			config::parseParams p(argc, argv);
			bool flag = false;
			
			// Display help?
			if (p.readParam("-h")) doHelp();

			// Output prefix
			p.readParam<string>("-p",outprefix);

			// Output format
			p.readParam<string>("-o",outFormat);

			// Suppress output
			suppressOutput = p.readParam("-N");

			// Frequencies to analyze
			vector<string> freqList;
			flag = p.readParam<string>("-f",freqList);
			if (flag)
			{
				for (auto it = freqList.begin(); it != freqList.end(); it++)
					rtmath::config::splitSet<double>(*it,frequencies);
			} else {
				cerr << "Must list frequencies to consider.\n";
				doHelp();
			}

			// radii to consider
			vector<string> sizeList;
			flag = p.readParam<string>("-s",sizeList);
			if (flag)
			{
				for (auto it = sizeList.begin(); it != sizeList.end(); it++)
					rtmath::config::splitSet<double>(*it,sizes);
			} else {
				cerr << "Must list sizes to consider.\n";
				doHelp();
			}
		}

		ostringstream statname;
		statname << outprefix << "stats.csv"; //<< outFormat;
		ofstream so(statname.str().c_str());
		so << "freq, size, nScatt, nNan" << endl;

		ostringstream srangename;
		srangename << outprefix << "envelope.csv";
		ofstream sr(srangename.str().c_str());
		sr << "freq, max error size" << endl;

		for (auto it = frequencies.begin(); it != frequencies.end(); ++it)
		{
			size_t err = 1;
			for (auto ot = sizes.begin(); ot != sizes.end(); ++ot)
			{
				ostringstream outname;
				outname << outprefix << "f-" << *it << "-s-" << *ot << "." << outFormat;
				mie::ddOutputMie target(*it,*ot);
				size_t n = target._sizeRaw();
				size_t nN = target._sizeRawNan();
				so << *it << ", " << *ot << ", " << n << ", " << nN << endl;

				if (nN < err)
				{
					sr << *it << ", " << *ot << endl;
					err = nN;
				}

				if (!suppressOutput)
					target.write(outname.str());
			}
		}
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
	cerr << "rtmath-mie\n";
	cerr << "A program for generating mie sphere outputs.\n";
	cerr << "Flags:\n";
	cerr << "-h\n";
	cerr << "\tProduce this help message.\n";
	cerr << "-p\n";
	cerr << "\tSpecify prefix for output files. Default is 'mie-'.\n";
	cerr << "-f\n";
	cerr << "\tSpecify frequencies to consider (in GHz).\n";
	cerr << "-s\n";
	cerr << "\tSpecify a list of particle radii (in um).\n";
	cerr << "-o\n";
	cerr << "\tSpecify output format (csv,tsv,nc,evans)\n";
	exit(1);
}

