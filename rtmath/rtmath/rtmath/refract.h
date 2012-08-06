#pragma once
#include <complex>
#include <string>
// Liu's refraction code, as translated into c++
//

namespace rtmath {
	namespace refract {
		// Ice complex refractive index
		// Christian Matzler (2006)
		void mice(double f, double t, std::complex<double> &m);


		// diel.tab writer
		void writeDiel(const std::string &filename, 
			const std::complex<double> &m);

		// Refractive index transformations
		// These are used when the ice crystals contain air or water
		// With given volume fractions (f).
		
		// Bohren and Battan (1980)
		void debyeDry(std::complex<double> Ma, std::complex<double> Mb, double f, std::complex<double> &Mres);

		// Maxwell-Garnett - assuming that ice spheres are inclusions and water is the surrounding medium
		void maxwellGarnett(std::complex<double> Mice, std::complex<double> Mwater, std::complex<double> Mair, double fIce, double fWater, std::complex<double> &Mres);

		// Maxwell-Garnet basic two material case
		void maxwellGarnettSimple(std::complex<double> Ma, std::complex<double> Mb, double f, std::complex<double> &Mres);


	};
};
