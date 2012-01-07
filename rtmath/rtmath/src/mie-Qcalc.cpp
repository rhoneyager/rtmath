#include "../rtmath/Stdafx.h"
#include "../rtmath/rtmath.h"

namespace mie {

Qcalc::Qcalc(const std::complex<double> &m)
{
	_m = m;
	_tolerance = 1.01;
	_atol = 1e-8;
}

Qcalc::Qcalc(const std::complex<double> &m, double tolerance, double atol)
{
	_m = m;
	_tolerance = tolerance;
	_atol = atol;
}

Qcalc::~Qcalc(void)
{
}

void Qcalc::calc(double x, double &Qext, double &Qsca, double &Qabs, double &g)
{
	// TODO: save previous results for faster future runs!!!!!!!
	// Construct the abNcalc class
	mie::abNCalc abn(_m, x);
	// The summations will be performed to within the tolerance value
	// That is, the summing stops once the tolerance is reached
	// Q_p are the previous values
	double Qe = 0, Qep, Qepar;
	double Qs = 0, Qsp, Qspar;
	double Qa = 0;
	double gpar = 0;

		// Define memory for an and bn
		std::complex<double> an, anp;
		std::complex<double> bn, bnp;
		std::complex<double> suma, sumb;
	abn.calc(1, an, bn);
	// TODO: use proper bounding formula
	for (unsigned int n=1; n < 400; n++)
	{
		// Set previous values for iteration
		Qep = Qe;
		Qsp = Qs;

		// Do iteration of the sum
		abn.calc(n+1, anp, bnp);

		Qepar = (2.0*n+1.0) * (an.real() + bn.real());
		Qspar = (2.0*n+1.0) * ((std::abs(an) * std::abs(an)) + (std::abs(bn) * std::abs(bn)) );
		
		suma = an*conj(anp)+bn*conj(bnp);
		sumb = an*conj(bn);

		// Do another tolerance check against too small terms messing up sum
		if (Qepar < _atol || Qspar < _atol) break;
		Qe += Qepar;
		Qs += Qspar;
		gpar += ((n*(n+2.0)/(n+1.0)) * suma.real()) + (((2.0*n+1)/(n*n+n))* sumb.real());
		an = anp;
		bn = bnp;
		// Note the inversion of div and inequalities to make this work
		//if ( (Qep / Qe > _tolerance) && (Qsp/Qs >_tolerance) ) break;
	}
	Qe *= 2.0/(x*x);
	Qs *= 2.0/(x*x);
	Qa = Qe-Qs;
	// Set final values
	Qext = Qe;
	Qsca = Qs;
	Qabs = Qa;

	// Now, to calculate g
	// I'm doing this here because the summations are already in place, and g is really a waste of a class
	// abncalc already holds the necessary values
	gpar *= 4.0/(x*x);
	g = gpar/Qsca;
	return;
}

}; // end mie

