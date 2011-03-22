#include "tauNCalc.h"
#include "piNCalc.h"
#include<cmath>

namespace mie {

	tauNCalc::tauNCalc(double theta) {
		this->theta = theta;
		this->mu = cos(theta);
		pin = new piNCalc(theta);
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

