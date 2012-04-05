#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include "../rtmath/matrixop.h"
#include "../rtmath/error/error.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"
#include "../rtmath/mie/mieScattMatrix.h"
#include "../rtmath/mie/mie-phaseFunc.h"
#include "../rtmath/mie/mie-Qcalc.h"
#include "../rtmath/refract.h"

namespace rtmath {
	namespace ddscat {

		mieScattMatrix::mieScattMatrix(double freq, double theta, double reff)
		{
			this->_init();
			_theta = theta;
			_phi = 0;
			_freq = freq;
			// freq is in GHz. Convert to wavelength in um.
			units::conv_spec ftowv("GHz","um");
			_wavelength = ftowv.convert(freq);

			_lock = true; // prevent genS from working

			_reff = reff;
			const double M_PI = boost::math::constants::pi<double>();
			// Assume default temp for index of refraction in this constructor.
			// reff and wavelength both in microns.
			_x = 2.0 * M_PI * reff / _wavelength;
			const double temp = 263.0; // Default temperature.
			// Calculate index of refraction from this
			rtmath::refract::mice(freq,temp,_m);

			// Constructor params now set. Can calculate the output right now.
			mie::miePhaseFunc pf(_x, _m);
			// Already have alpha = theta, so
			std::shared_ptr<rtmath::matrixop> res = pf.eval(theta);
			res->toDoubleArray(&_Pnn[0][0]);
			// Also need to figure out extinction
			// Extinction is a diagonal matrix in the case of spheres
			// It is just Qext...
			mie::Qcalc qc(_m);
			double Qext, Qsca, Qabs, g;
			qc.calc(_x,Qext,Qsca,Qabs,g);
			matrixop K = matrixop::diagonal(Qext,2,4,4);
			K.toDoubleArray(&_Knn[0][0]);

		}

		mieScattMatrix::~mieScattMatrix()
		{
		}

	}
}

