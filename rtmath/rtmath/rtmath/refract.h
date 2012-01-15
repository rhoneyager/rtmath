#pragma once
// Liu's refraction code, as translated into c++
//

namespace rtmath {
	namespace refract {
		// Ice complex refractive index
		// Christian Matzler (2006)
		void mice(double f, double t, std::complex<double> &m);
	};
};
