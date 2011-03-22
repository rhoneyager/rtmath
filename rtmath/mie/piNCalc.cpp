#include "piNCalc.h"

#include <cmath>

namespace mie {

	piNCalc::piNCalc(double theta)
	{
		this->theta = theta;
		this->mu = cos(theta);
		pin.push_back(0);
		pin.push_back(1);
	}

	double piNCalc::calc(unsigned int n)
	{
		if (pin.size() > n) return pin[n];
		for (unsigned int i=pin.size(); i<=n; i++)
		{
			double res;
			res = ((2.0*n - 1.0) / (n - 1.0)) * mu * pin[i-1]
			- ((n) / (n - 1.0)) * pin[n-2];
			pin.push_back(res);
		}
		return pin[n];
	}

}; // end mie


