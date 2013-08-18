#include "Stdafx-core.h"
#include <cmath>
#include <boost/math/constants/constants.hpp>
//#include "../rtmath/error/error.h"
#include "../rtmath/thermal.h"
#include "../rtmath/units.h"

namespace rtmath {
	namespace thermal
	{
		// Speed of light (m/s)
		const double c = 2.99792458e8;
		// Planck constant (J.s)
		const double h = 6.62606957e-34;
		// Stefan-Boltzmann Constant (J/K)
		const double kb = 1.3806488e-23;

		double radiancePlanck(double T, double f)
		{
			// T is temperature in K
			// f is frequency in GHz
			const double fim = rtmath::units::conv_spec("GHz", "Hz").convert(f);
			const double e = boost::math::constants::e<double>();
			double res = 2. * h * pow(fim,3.) / pow(c,2.);
			res /= pow(e,h*fim/(kb*T)) - 1;
			return res;
		}

		double radianceRJ(double T, double f)
		{
			const double fim = rtmath::units::conv_spec("GHz", "Hz").convert(f);
			double res = 2. * pow(fim,2.) * kb * T / pow(c,2.);
			return res;
		}


	}
}
