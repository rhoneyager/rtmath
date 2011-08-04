//#undef HEAP_CHECK


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

#include "../rtmath/debug_mem.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;
	try {
		rtmath::debug::debug_preamble();
		
		rtmath::debug::memcheck::enabled = false;

		// This part of the code lets me create an answer file, so that I don't 
		// need to copy and paste all the time.
		istream *in = NULL;
		string confrootname;
		if (argc > 1)
		{
			in = &cin;
			confrootname.assign(argv[1]);
		} else {
			in = &cin;
			std::cout << "Config file: ";
			getline(*in,confrootname);
		}
		rtmath::config::configsegment *confroot = rtmath::config::configsegment::loadFile(confrootname.c_str(),NULL);

		std::cout << "Config loading finished\n";
		string errorfile;
		confroot->getVal("stderr", errorfile);
		if (errorfile.length() > 0)
		{
			freopen(errorfile.c_str(),"w",stderr); // Remap standard error for debug
			rtmath::debug::debug_preamble(); // Print again, this time to the file
		}

		// Enable debug logging (needs to be turned on to prevent prog. start errors)
		rtmath::debug::memcheck::enabled = false;

		using namespace rtmath;

		// Gather the input
		string htp, molp, pars, wvlows, wvhighs, wvrates, atmoprof;
		htp = config::queryConfig(confroot, "lbl/hitranFile", "hitran lines file: ");
		molp = config::queryConfig(confroot, "lbl/molparamsFile", "molparam file: ");
		pars = config::queryConfig(confroot, "lbl/parsumFile", "parsum file: ");
		atmoprof = config::queryConfig(confroot, "testing/atmosFile", "Atmospheric Profile and Species Data: ");
		wvlows = config::queryConfig(confroot, "testing/wvlow", "Lower Wavenumber (cm^-1): ");
		wvhighs = config::queryConfig(confroot, "testing/wvhigh", "Upper Wavenumber (cm^-1): ");
		wvrates = config::queryConfig(confroot, "testing/wvspan", "Rate (cm^-1): ");
		double wvnum, wvnumhigh, wvrate; // Conversion of string into doubles
		wvnum = M_ATOF(wvlows.c_str());
		wvnumhigh = M_ATOF(wvhighs.c_str());
		wvrate = M_ATOF(wvrates.c_str());
		
		rtmath::debug::timestamp(false);
		rtmath::lbl::specline::loadlines(htp.c_str(),molp.c_str(),pars.c_str());
		rtmath::lbl::specline::speclineShape = rtmath::lbl::lineshape::LORENTZIAN;

		rtmath::debug::timestamp(false);
		//cout << "molp has " << rtmath::lbl::specline::abundanceMap.size() << " entries.\n";
		//cout << "pars has " << rtmath::lbl::specline::QThigh - rtmath::lbl::specline::QTlow << " temperatures.\n";

		// Read in the main profile, list its name, and ask for the wavenumber range
		rtmath::atmos lblatmos;
		lblatmos.loadProfile(atmoprof.c_str());

		cout << "Profile: " << lblatmos.name << endl;
		// List the memory locations now
		rtmath::debug::memcheck::__Track(3,0,0,0,0,0);
		fflush(stderr);

//		NcFile cdf("c:/test.nc", NcFile::Replace);
//		lblatmos.saveProfile(&cdf, lblatmos.name.c_str());

		std::map<double, double> taus;
		//double *tau = new double( (unsigned int) wvnumhigh - (unsigned int) wvnum + 1);
		double i=0;
		cout << "Performing atmospheric transmission calculations without scattering.\n";
		while (wvnum + i <= wvnumhigh)
		{
			rtmath::debug::timestamp(false);
			// Call functions
			double tau;
			lblatmos.nu(wvnum+i);
			tau = lblatmos.tau();
			taus[wvnum+i] = tau;
			std::cout << (wvnum + i) << "\t" << tau << "\t" << exp(-1.0*tau) << std::endl;
			rtmath::debug::timestamp(true);
			i+= wvrate;
		}

		rtmath::debug::timestamp(false);

		cout << endl;
		cout << endl;
		cout << "Assuming a slightly scattering atmosphere." << endl;
		cout << "Rayleigh scattering with x = 1, m = 1.33-0.01i" << endl;
		rayleigh::rayleighPhaseFunc pfalpha(1.0,complex<double>(1.33,0.01));
		//rtmath::daPfAlpha(rtselec::R, 
		cout << "Test program routines finished." << endl;
#ifdef _WIN32
		for (;;)
		{
			std::getchar();
		}
#endif
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		throw;
	}

	return 0;
}
