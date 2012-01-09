#include <iostream>
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

#include <rtmath/rtmath.h>

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;
	using namespace rtmath::ddscat;
	using namespace boost::filesystem;


	// Redirect cout
	//streambuf *corig, *filebuf;

	try {
		// Take ddscat name or path from argv, and attempt to load the files
		if (argc == 1)
		{
			cout << "Error: no files specified.\n";
#ifdef _WIN32
			std::getchar();
#endif
			return 1;
		}
		string file(argv[1]);
		boost::filesystem::path p(file.c_str()), dir, outfile,
			outiso, outs;
		dir = p.remove_filename();
		cout << "Loading from: " << dir << endl << endl;

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
			//if (it == a._data.begin())
				it->second.writeCSV(outfile.string()); // Verified
			newname = it->second.filename;
			newname.append(".nc");
			outfile = dir / newname;
			//it->second.writeCDF(outfile.string()); // Also not necessary
			//cout << "Output written to " << outfile << endl;
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

		cout << "THETAS processed:\n";
		std::set<double>::iterator tit;
		for(tit=THETAS.begin(); tit != THETAS.end(); tit++)
		{
			cout << *tit << "\t";
		}
		cout << endl << endl;

		std::set<double> PHIS;
		// Do iteration to see bounds on PHI
		for (it=a._data.begin(); it != a._data.end(); it++)
		{
			double th = it->second._Phi;
			if (PHIS.count(th) == 0)
				PHIS.insert(th);
			//it->second._Theta
		}

		cout << "PHIS processed:\n";
		for(tit=PHIS.begin(); tit != PHIS.end(); tit++)
		{
			cout << *tit << "\t";
		}
		cout << endl << endl;



		// Now, try and generate ensemble results
		cout << "Generating ensembles" << endl;
		ddOutputEnsembleIso ensiso;
		ensiso._ensemble = a._data;
		ensiso.generate();
		ensiso.res.writeCSV((dir / "isotropic.csv").string());
		ensiso.res.writeCDF(outiso.string());
		cout << "Isotropic output written to " << outiso << endl;
		cout << endl << endl;

		double mean = 0;
		size_t numSigmas = 30;
		double sigmas[numSigmas];
		for (size_t k=1;k<=numSigmas;k++) sigmas[k-1] = k;
		//size_t numSigmas = 6;
		//double sigmas[] = { 5, 10, 15, 20, 25, 30};

		ddOutputEnsembleGaussian* gens;

		for (size_t k=0;k<numSigmas;k++)
		{
			// Do a check to see if PHI or THETA is being varied, and use
			// the appropriate weighting function
			if (PHIS.size() > 1)
			{
				gens = new ddOutputEnsembleGaussianPhi(mean, sigmas[k]);
				if (k==0) cout << "Varying phi\n";
			} else {
				gens = new ddOutputEnsembleGaussianTheta(mean, sigmas[k]);
				if (k==0) cout << "Varying theta\n";
			}

			gens->_ensemble = a._data;
			gens->generate();
			//outs = dir / "sigma5.nc";
			ostringstream sfile;
			sfile << "sigma";
			sfile.fill('0');
			sfile.width(2);
			sfile << sigmas[k];
			sfile << ".nc";
			outs = dir / sfile.str();
			gens->res.writeCDF(outs.string());
			sfile << ".csv";
			gens->res.writeCSV((dir / sfile.str()).string());
			delete gens;
		}

		cout << "Sigmas written to files." << endl << endl;



	}
	catch (...)
	{
		cout << "Error thrown" << endl;

	}
#ifdef _WIN32
	std::getchar();
#endif
	return 0;
}
