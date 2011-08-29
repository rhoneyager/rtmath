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
#include "../rtmath/damatrix_quad.h"
#include "../rtmath/rayleigh.h"

int main(int argc, char* argv[])
{
	using namespace std;
	using namespace rtmath;
	try {
		rtmath::debug::debug_preamble();
		
		rtmath::debug::memcheck::enabled = false;
		rtmath::daint::deg = 1; // For now, use only one degree of quadrature for testing
		fflush(stderr);


		double alb = 0.1;
		double tau = 1.0;
		double mu = 1.0, mun = 0.0, phi = 0.3, phin = 0.1;
		double x = 1.0;
		std::complex<double> m;
		m.real(1.33);
		m.imag(0.001);
		mapid mid(mu,mun,phi,phin);	

		// Use the rayleigh-scattering case, for starters
		// Need a layer with a phasefunction matrixop
		rayleigh::rayleighPhaseFunc ray(x,m);
		

		shared_ptr<daInitLayer> iLa( new daInitLayer(ray,alb, tau, rtselec::R) );
		// Special cast is needed to keep shared_ptr happy. Unfortunate.
		// It also fixes the double free issue with daInitLayer cacheing, I think
		shared_ptr<damatrix> iLaBase = static_pointer_cast<damatrix>(iLa); 
		shared_ptr<matrixop> iLaeval = iLaBase->eval(mid);
		cout << "iLa\n";
		iLaeval->print();
		iLaeval = iLaBase->eval(mid);
		cout << "iLa\n";
		iLaeval->print();

		shared_ptr<damatrix> iLaDoubled = damatrix::op(iLaBase,iLaBase,MULT);
		shared_ptr<matrixop> iLaDoubledeval = iLaDoubled->eval(mid);
		cout << "iLaDoubled\n";
		iLaDoubledeval->print();
		iLaDoubledeval = iLaDoubled->eval(mid);
		cout << "iLaDoubled\n";
		iLaDoubledeval->print();
		
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
