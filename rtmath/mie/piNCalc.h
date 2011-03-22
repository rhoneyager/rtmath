#pragma once


#include<vector>

namespace mie {

	class piNCalc {
	public:
		piNCalc(double theta);
		double theta;
		double mu;
		std::vector<double> pin;
		double calc(unsigned int n);
	};


}; // end mie

