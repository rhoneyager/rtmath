#include "../rtmath/Stdafx.h"
#include "../rtmath/mie/mie.h"
#include <complex>

#include <cmath>

namespace mie {

Scalc::Scalc(const std::complex<double> &m, double x)
{
	_m = m;
	_x = x;
	_tolerance = 1.01;
	_atol = 1e-8;
}

Scalc::Scalc(const std::complex<double> &m, double x, double tol, double atol)
{
	_tolerance = tol;
	_atol = atol;
	_m = m;
	_x = x;
}

Scalc::~Scalc(void)
{
}

//void Scalc::calc(double mu, std::complex<double> &Sa, std::complex<double> &Sb, double &Ssq)
void Scalc::calc(double mu, double Snn[4][4], std::complex<double> Sn[4])
{
	// This code is based on the Qcalc code
	// TODO: save previous results for faster future runs!!!!!!!
	// Construct the abNcalc class
	mie::abNCalc abn(_m, _x);

	// The summations will be performed to within the tolerance value
	// That is, the summing stops once the tolerance is reached
	// Q_p are the previous values
	mie::piNCalc piN(mu);
	mie::tauNCalc tauN(mu);

	// Use Sa, Sb, Ssq that are provided
	std::complex<double> Sap, Sapar;
	std::complex<double> Sbp, Sbpar;

	// Zero Sa and Sb
	std::complex<double> zero(0,0);
	for (unsigned int i=0;i<4;i++)
		Sn[i] = zero;
	for (unsigned int i=0; i<4;i++)
		for (unsigned int j=0; j<4; j++)
			Snn[i][j] = 0;

	std::complex<double> *Sa = &Sn[3]; // S11 = S2, and S22 = S1
	std::complex<double> *Sb = &Sn[0];
	*Sa = zero;
	*Sb = zero;
	
	size_t nmax = (size_t) (_x + 4.0*pow(_x,1./3.) + 2.0) + 1;
	//for (unsigned int n=1; n < 400; n++)
	for (unsigned int n=1; n < nmax; n++)
	{
		// Set previous values for iteration
		Sap = *Sa;
		Sbp = *Sb;
		// Define memory for an and bn
		std::complex<double> an;
		std::complex<double> bn;
		double pinRes;
		double taunRes;
		// Do iteration of the sum
		abn.calc(n, an, bn);
		pinRes = piN.calc(n);
		taunRes = tauN.calc(n);

		Sapar = an*pinRes + bn*taunRes;
		Sbpar = an*taunRes + bn*pinRes;
		Sapar *= (2.0*n+1)/(n*n+n);
		Sbpar *= (2.0*n+1)/(n*n+n);
		// Do another tolerance check against too small terms messing up sum
		if (abs(Sapar) < _atol || abs(Sbpar) < _atol) break;
		*Sa += Sapar;
		*Sb += Sbpar;
		// Note the inversion of div and inequalities to make this work
		//if ( (Qep / Qe > _tolerance) && (Qsp/Qs >_tolerance) ) break;
	}

	this->_genMuellerMatrix(Snn, Sn);
	// Ssq is now Snn[0][0]
	//Ssq = 0.5 * (abs(*Sa)*abs(*Sa) + abs(*Sb) * abs(*Sb));

	return;
}

}; // end mie


