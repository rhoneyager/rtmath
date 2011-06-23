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

int main(int argc, char* argv[])
{
	using namespace std;
	try {
		/*
		for (;;)
		{
			std::string inval;
			std::getline(cin,inval);
			std::cout << rtmath::macros::m_atof(inval) << std::endl;
		}
		return 0;
		*/
		rtmath::debug::debug_preamble();
		freopen("strerr.txt","w",stderr); // Remap standard error for debug
		rtmath::debug::debug_preamble(); // Print again, this time to the file
		// Enable debug logging (needs to be turned on to prevent prog. start errors)
		//rtmath::debug::memcheck::enabled = true;


		// This part of the code lets me create an answer file, so that I don't 
		// need to copy and paste all the time.
		istream *in = NULL;
		if (argc > 1)
		{
			static filebuf fb;
			fb.open(argv[1],ios::in);
			static istream indata(&fb);
			in = &indata;
		} else {
			in = &cin;
		}

		//cout << "hitran parse file: ";
		string htp, molp, pars;
		getline(*in,htp);
		//cout << "Molparam file: ";
		getline(*in,molp);
		//cout << "parsum file: ";
		getline(*in,pars);
		rtmath::debug::timestamp(false);
		rtmath::lbl::specline::loadlines(htp.c_str(),molp.c_str(),pars.c_str());
		rtmath::debug::timestamp(false);
		//cout << "molp has " << rtmath::lbl::specline::abundanceMap.size() << " entries.\n";
		//cout << "pars has " << rtmath::lbl::specline::QThigh - rtmath::lbl::specline::QTlow << " temperatures.\n";

		//cout << "Now, to perform calculations with an atmospheric profile\n";
		//cout << "Profile filename: ";
		string mainprof;
		getline(*in,mainprof);

		// Read in the main profile, list its name, and ask for the wavenumber range
		rtmath::atmos lblatmos;
		lblatmos.loadProfile(mainprof.c_str());

		cout << "Profile: " << lblatmos.name << endl;
		// List the memory locations now
		rtmath::debug::memcheck::__Track(3,0,0,0,0,0);
		fflush(stderr);

		double wvnum, wvnumhigh, wvrate;
//		cout << "Lower Wavenumber (cm^-1): ";
		*in >> wvnum;
//		cout << "Upper Wavenumber (cm^-1): ";
		*in >> wvnumhigh;
//		cout << "Rate (cm^-1): ";
		*in >> wvrate;

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
//		cout << "Final results:" << endl;
//		cout << "Wavenumber\t\tTau\tT\n";
		for (std::map<double,double>::const_iterator it=taus.begin(); it != taus.end(); it++)
		{
			// Output data
			//cout << (wvnum + j) << ":\t" << tau[j] << "\t" << exp(-1.0 * tau[j]) << endl;
		//	std::cout << it->first << ":\t" << it->second << std::endl;
		}


		rtmath::debug::timestamp(false);

		cout << endl;
		cout << endl;
		//cout << "Test program routines finished." << endl;
		//for (;;)
		{
			//std::getline(cin,mainprof);
		}
	}
	catch (rtmath::debug::xError &err)
	{
		err.Display();
		throw;
	}

	return 0;
}
