#pragma once

#include <complex>
#include "enums.h"
#include "error/debug.h"
#include "matrixop.h"
#include "da/damatrix.h"
#include <map>

namespace rtmath {

	class phaseFunc // TODO: rewrite this.
	{
	public:
		// This fuction is designed to be pure virtual, so that mie and other
		// phase functions can be included in atmospheric layers
		phaseFunc(void) {}
		virtual ~phaseFunc(void) {}
		// eval evaluates P for reduced mur and phir (mu-mu0, phi-phi0) to get proper directional data
		//virtual void eval(double mu, double mun, double phir, double Pnn[4][4], double res[4][4]);
		// calc constructs the base P matrix, centered on mu=mu0
		//virtual void calc(double mu, std::complex<double> &m, double x, double Pnn[4][4]) = 0;
		// And, give the same function as a matrixop class
		//virtual void calc(double mu, std::complex<double> &m, double x, matrixop &Pnn);
		virtual std::shared_ptr<matrixop> eval(double alpha) const = 0;
	protected:
		mutable std::map<double,std::shared_ptr<matrixop> > _eval_cache;
	};

	class scattMatrix // TODO: rewrite this to make it easier to understand and be derived from.
	{
	public:
		scattMatrix(void);
		virtual ~scattMatrix(void);
		// Calculate the scattering amplitude matrix for a given mu.
		// Here, mu = cos(alpha), the total scattering angle.
		virtual void calc(double mu, double Snn[4][4], std::complex<double> Sn[4]) = 0;
	public:
		// f is the frequency
		static void _genMuellerMatrix(double Snn[4][4], const std::complex<double> Sn[4]);
		static void _genExtinctionMatrix(double Knn[4][4], const std::complex<double> Sn[4], double fGHz);
		static void _invertS(const double Snn[4][4], const double Knn[4][4], double fGHz, std::complex<double> Sn[4]);
	};

}; // end namespace rtmath
