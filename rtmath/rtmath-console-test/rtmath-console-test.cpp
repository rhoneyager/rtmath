#include <iostream>
#include "../rtmath/rtmath.h"
//#include "../mie/mie.h"
#include "../rtmath-base/rtmath-base.h"
#include <complex>
#include <time.h>
//#include <netcdf.h>
#define _USE_MATH_DEFINES
#include <math.h>

int main(int argc, char** argv)
{
	using namespace std;
	rtmath::debug::debug_preamble();

	using namespace std;
	cout << "hitran parse file: ";
	string htp, molp, pars;
	getline(cin,htp);
	cout << "Molparam file: ";
	getline(cin,molp);
	cout << "parsum file: ";
	getline(cin,pars);
	time_t start, finish;
	start = time (NULL);
	cout << "Start time: " << start << endl;
	rtmath::lbl::specline::loadlines(htp.c_str(),molp.c_str(),pars.c_str());
	finish = time(NULL);
	cout << "Finish: " << finish << endl;
	cout << "Total time to load is " << (finish - start) << " seconds.\n";
	cout << "molp has " << rtmath::lbl::specline::abundanceMap.size() << " entries.\n";
	cout << "pars has " << rtmath::lbl::specline::Qmap.at(0).size() << " temperatures.\n";

	cout << "Now, to perform calculations with an atmospheric profile\n";
	cout << "Profile filename: ";
	string mainprof;
	getline(cin,mainprof);

	// Read in the main profile, list its name, and ask for the wavenumber range
	rtmath::atmos lblatmos;
	lblatmos.loadProfile(mainprof.c_str());

	cout << "Profile is: " << lblatmos.name << endl;

	double wvnum, wvnumhigh;
	cout << "Lower Wavenumber (cm^-1): ";
	cin >> wvnum;
	cout << "Upper Wavenumber (cm^-1): ";
	cin >> wvnumhigh;

	double *tau = new double( (unsigned int) wvnumhigh - (unsigned int) wvnum + 1);
	unsigned int i=0;
	while (wvnum + i <= wvnumhigh)
	{
		cout << "Performing atmospheric transmission calculations without scattering.\n";
		start = time(NULL);
		cout << "Start time: " << start << endl;
		// Call functions

		lblatmos.nu(wvnum+i);
		cout << i;
		tau[i] = lblatmos.tau();
		i++;
	}
	finish = time (NULL);
	for (unsigned int j=0;j<i;j++)
	{
		// Output data
		cout << (wvnum + j) << ":\t" << tau[j] << "\t" << exp(-1.0 * tau[j]) << endl;
	}

	
	cout << "End time: " << finish << endl;
	cout << "Total time: " << (finish - start) << " seconds." << endl;

	cout << endl;
	cout << endl;
	cout << "Test program routines finished." << endl;
	return 0;
}
