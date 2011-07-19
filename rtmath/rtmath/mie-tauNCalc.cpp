#include "Stdafx.h"
#include "mie-tauNCalc.h"
#include "mie-piNCalc.h"

namespace mie {

	tauNCalc::tauNCalc(double mu) {
		this->mu = mu;
		pin = new piNCalc(mu);
	}

	tauNCalc::~tauNCalc()
	{
		delete pin;
	}

	double tauNCalc::calc(unsigned int n)
	{
		double res;
		res = n * mu * pin->calc(n) - (n + 1.0) * pin->calc(n-1);
		return res;
	}


}; // end mie

