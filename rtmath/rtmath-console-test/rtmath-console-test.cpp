//#undef HEAP_CHECK


#include <iostream>
#include "../rtmath/rtmath.h"
//#include "../mie/mie.h"
#include "../rtmath/rtmath-base.h"
#include <complex>
#include <time.h>
//#include <netcdf.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <fstream>
#include <stdio.h>
#include <omp.h>

#include "../rtmath/debug_mem.h"
#include "../rtmath/rayleigh.h"

int main(int argc, char* argv[])
{
	using namespace std;
	try {
		rtmath::debug::debug_preamble();
		
		rtmath::debug::memcheck::enabled = false;

		// Doing the testing of the layer generation functions

		using namespace rtmath;
		double alb = 0.1;
		double tau = 1.0;
		double mu = 1.0, mun = 1.0, phi = 0.0, phin = 0.0;
		double x = 1.0;
		std::complex<double> m;
		m.real(1.33);
		m.imag(0.001);

		rtselec::rtselec rs = rtselec::R;
		// Use the rayleigh-scattering case, for starters
		// Need a layer with a phasefunction matrixop
		rayleigh::rayleighPhaseFunc ray;
		double Pnn[4][4];
		ray.calc(mu,m,x,Pnn);
		// Now, to convert Pnn to a matrixop, pf
		matrixop pf(2,4,4);
		pf.fromDoubleArray(*Pnn);

		//dalayerInit ilayer(0,alb,tau,rs);
		dalayer layer(pf,alb);
		mapid mid(mu,mun,phi,phin);
		layer.tau(tau);
		layer.generateLayer(mid);

		boost::shared_ptr<damatrix> _R = layer.getR();

		

		return 0;

		// This part of the code lets me create an answer file, so that I don't 
		// need to copy and paste all the time.
		istream *in = NULL;
		string confrootname;
		if (argc > 1)
		{
			/*
			static filebuf fb;
			fb.open(argv[1],ios::in);
			static istream indata(&fb);
			in = &indata;
			*/
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

		std::map<double, double> taus;
		//double *tau = new double( (unsigned int) wvnumhigh - (unsigned int) wvnum + 1);
		unsigned int i=0;
//		cout << "Performing atmospheric transmission calculations without scattering.\n";
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
			i+=wvrate;
		}

		rtmath::debug::timestamp(false);

		cout << endl;
		cout << endl;
		
		cout << "Test program routines finished." << endl;
		for (;;)
		{
			std::getline(cin,atmoprof);
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		throw;
	}

	return 0;
}
