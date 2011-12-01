#include <iostream>
#include "../rtmath/rtmath.h"
#include "../rtmath/rtmath-base.h"
#include "../rtmath/mie.h"
#include "../rtmath/rayleigh.h"
#include <complex>
#include <time.h>
//#include <netcdf.h>
//#include <netcdfcpp.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <omp.h>
#include <memory>
#include <string>
#include <boost/filesystem.hpp> // used for location of output of netcdf

#include "../rtmath/debug_mem.h"
#include "../rtmath/damatrix_quad.h"
#include "../rtmath/rayleigh.h"
#include "../rtmath/ddscat2.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::ddscat;
	using namespace boost::filesystem;


	// Redirect cout
	streambuf *corig, *filebuf;

	try {
		// Take ddscat name or path from argv, and attempt to load the files
		if (argc == 1) 
		{
			cerr << "Error: no files specified.\n";
#ifdef _WIN32
			std::getchar();
#endif
			return 1;
		}
		string file(argv[1]);
		boost::filesystem::path p(file.c_str()), dir, outfile, 
			outiso, outs;
		dir = p.remove_filename();
		cerr << "Loading from: " << dir << endl << endl;

		//ddOutputSingle a(file);
		ddOutput a(file);
		std::map<ddCoords3, ddOutputSingle, ddCoordsComp>::const_iterator it;
		it = a._data.begin();
		//cout << it->first.beta << " " << it->first.theta << " " << it->first.phi << endl;

		// Write output in the same folder as the input
		outfile = dir / "out.nc";
		outiso = dir / "isotropic.nc";
		//outs = dir / "sigma5.nc";

		for (it = a._data.begin(); it != a._data.end(); it++)
		{
			//it->second.print();
			std::string newname = it->second.filename;
			newname.append(".csv");
			outfile = dir / newname;
			ofstream outcsv( outfile.string());
			filebuf = outcsv.rdbuf();
			cout.rdbuf(filebuf);
			it->second.print();
			newname = it->second.filename;
			newname.append(".nc");
			outfile = dir / newname;
			it->second.writeCDF(outfile.string());
			//cerr << "Output written to " << outfile << endl;
		}
		//cout << "Loaded " << a._data.size() << " files" << endl;

		std::set<double> THETAS;
		// Do iteration to see bounds on THETA
		for (it=a._data.begin(); it != a._data.end(); it++)
		{
			double th = it->second._Theta;
			if (THETAS.count(th) == 0)
				THETAS.insert(th);
			//it->second._Theta
		}

		cerr << "THETAS processed:\n";
		std::set<double>::iterator tit;
		for(tit=THETAS.begin(); tit != THETAS.end(); tit++)
		{
			cerr << *tit << "\t";
		}
		cerr << endl << endl;

		std::set<double> PHIS;
		// Do iteration to see bounds on PHI
		for (it=a._data.begin(); it != a._data.end(); it++)
		{
			double th = it->second._Phi;
			if (PHIS.count(th) == 0)
				PHIS.insert(th);
			//it->second._Theta
		}

		cerr << "PHIS processed:\n";
		for(tit=PHIS.begin(); tit != PHIS.end(); tit++)
		{
			cerr << *tit << "\t";
		}
		cerr << endl << endl;



		// Now, try and generate ensemble results
		cerr << "Generating ensembles" << endl;
		ddOutputEnsembleIso ensiso;
		ensiso._ensemble = a._data;
		ensiso.generate();
		ofstream outisof( (dir / "isotropic.csv").string());
		filebuf = outisof.rdbuf();
		cout.rdbuf(filebuf);
		ensiso.res.print();
		ensiso.res.writeCDF(outiso.string());
		cerr << "Isotropic output written to " << outiso << endl;
		cerr << endl << endl;


		double mean = 0;
		double sigmas[] = {5, 10, 15, 30};
		size_t numSigmas = 4;

		ddOutputEnsembleGaussian* gens;

		for (size_t k=0;k<numSigmas;k++)
		{
			// Do a check to see if PHI or THETA is being varied, and use 
			// the appropriate weighting function
			if (PHIS.size() > 1)
			{
				gens = new ddOutputEnsembleGaussianPhi(mean, sigmas[k]);
				if (k==0) cerr << "Varying phi\n";
			} else {
				gens = new ddOutputEnsembleGaussianTheta(mean, sigmas[k]);
				if (k==0) cerr << "Varying theta\n";
			}

			gens->_ensemble = a._data;
			gens->generate();
			//outs = dir / "sigma5.nc";
			ostringstream sfile;
			sfile << "sigma" << sigmas[k] << ".nc";
			outs = dir / sfile.str();
			gens->res.writeCDF(outs.string());
			sfile << ".csv";
			ofstream outcsv( (dir / sfile.str()).string());
			filebuf = outcsv.rdbuf();
			cout.rdbuf(filebuf);
			gens->res.print();
			delete gens;
		}

		cerr << "Sigmas written to files." << endl << endl;



	}
	catch (...)
	{
		cerr << "Error thrown" << endl;

	}
#ifdef _WIN32
	std::getchar();
#endif
	return 0;
}