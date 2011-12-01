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
		rtmath::daint::deg = 2; // For now, use only one degree of quadrature for testing
		fflush(stderr);

		cout << endl;

		double avals[] = {0,0,0,8,1,0,0,14,0,1,0,3,0,0,1,-4};
		matrixop a(2,4,4), Q(2,3,3), R(2,4,4), uh(2,3,3);
		a.fromDoubleArray(avals);
		//double uhtvals[] = {1,-4,0,3,-1,3,0,-2,3,-7,-2,6,0,4,0,-2};
		double uhtvals[] = {2,-1,1,-1,2,1,1,-1,2};
		Q.fromDoubleArray(uhtvals);

		//cout << "Matrix a:" << endl;
		//a.print();

		cout << "Input matrix: " << endl;
		Q.print();

		uh = Q.upperHessenberg();

		cout << "Upper Hessenberg: " << endl;
		uh.print();

		return 0;

		a.decompositionQR(Q,R);

		cout << "\nQ:" << endl;
		Q.print();
		cout << "\nR:"<<endl;
		R.print();

		cout << "\nCheck\n" << endl;
		(Q*R).print();

		cout << "\nQR Algorithm:\n";
		(R*Q).print();

		matrixop b(2,4,4), c(2,4,4);
		b = (R*Q);

		for (size_t i=0;i<100;i++)
		{
			cout << "i = " << i << endl;
			b.decompositionQR(Q,R);
			b.print();
			c = R*Q;
			b = c;
		}
		/*
		double alb = 0.1;
		double tau = 1.0;
		double mu = 0.1, mun = 0.1, phi = 0.3, phin = 0.1;
		double x = 1.0;
		std::complex<double> m;
		m.real(1.33);
		m.imag(0.001);
		mapid mid(mu,mun,phi,phin);	

		cerr << "Using angles of " << mid.print() << endl;
		// Use the rayleigh-scattering case, for starters
		// Need a layer with a phasefunction matrixop
		shared_ptr<rayleigh::rayleighPhaseFunc> ray (new rayleigh::rayleighPhaseFunc(x,m));
		shared_ptr<phaseFunc> raycast = static_pointer_cast<phaseFunc> (ray);
		shared_ptr<daPfAlpha> pfa( new daPfAlpha(raycast));
		shared_ptr<damatrix> pfacast = static_pointer_cast<damatrix> (pfa);
		shared_ptr<daInitLayer> base  ( new daInitLayer(pfacast, alb, tau, rtselec::R));
		shared_ptr<matrixop> baseEval = base->eval(mid);
		cerr << "base evaled\n";
		baseEval->print();
		cout << endl;

		shared_ptr<damatrix> a, b;
		a = static_pointer_cast<damatrix> (base);
		cerr << "now to test iteration and cacheing" << endl;
		cerr << "for i = 1 ... 3" << endl;

		for (unsigned int i=1;i<4;i++)
		{
			cerr << "i = " << i << endl;
			b = damatrix::op(a,a,MULT);
			shared_ptr<matrixop> evals = b->eval(mid);
			evals->print();
			a = b;
		}

		cout << "\ntesting done. please verify results." << endl;
		*/
		/*

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
		*/
		cout << "\nTest program routines finished." << endl;
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
	catch (...)
	{
		cerr << "\nProgram encountered an unknown / unhandled exception. Terminating." << endl;
		throw;
	}

	return 0;
}
