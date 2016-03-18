#include "Stdafx-ddscat.h"
#pragma warning( disable : 4996 ) // -D_SCL_SECURE_NO_WARNINGS
#include <string>
#include <boost/math/constants/constants.hpp>

#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/macros.h>
#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/splitSet.h>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/Serialization.h>
#include <Ryan_Debug/error.h>
#include "../rtmath/error/debug.h"
#include "../rtmath/ddscat/shapefile.h"
#include "../rtmath/rga/rga.h"
#include "../rtmath/refract.h"
#include "../rtmath/units.h"

namespace rtmath {
	namespace rga {
		cross_sections::cross_sections() : Cbk(-1), Csca(-1),
			g(-999), Cabs(-1), Cext(-1), valid(false) {}

		rgSimple::rgSimple() : dSpacing(0) {}

		rgSimple::~rgSimple() {}

		boost::shared_ptr<const rgSimple> rgSimple::generate(
			boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> s,
			double dSpacing)
		{
			boost::shared_ptr<rgSimple> res(new rgSimple);

			res->dSpacing = dSpacing;
			res->shp = s;

			return res;
		}

		boost::shared_ptr<const cross_sections>
			rgSimple::run(double freqGHz, double tempK, double betad,
			double thetad, double phid) const
		{
			/// As implemented in Hogan and Westbrook 2014, equation 1
			std::complex<double> mIce;
			using rtmath::refract::_frequency;
			using rtmath::refract::_temperature;
			using rtmath::refract::_m;
			using rtmath::refract::_provider;
			rtmath::refract::mIce(
				_frequency = freqGHz,
				_temperature = tempK,
				_m = mIce);

			const double pi = boost::math::constants::pi<double>();
			double wvlen_m = rtmath::units::conv_spec("GHz","m").convert(freqGHz);
			// 2 pi / wavelength
			double wvnum_m = 2. * pi / wvlen_m;
			std::complex<double> eps, K; // K is Clausius-Mossotti factor
			rtmath::refract::mToE(mIce,eps);
			K = (eps - std::complex<double>(1,0)) / (eps + std::complex<double>(2,0));

			double prefactor = 9 * std::pow(wvnum_m,4.) * (K * conj(K)).real()/ (4. * pi);

			// Performing integration over normalized coordinates
			// TODO: perform rotation of shape file
			// TODO: add shapefile routine to perform rotation, then write back as a shape file
			// Assume slicing over the X axis for now, since the shape is already rotated.
			auto aarray = shp->sliceAll(0);
			std::complex<double> totint;
			const std::complex<double> j(0,1);
			// The independent axis is 1, and the dependent is 3.
			for (int i=0; i < aarray->rows() - 1; ++i) {
				double s = (*aarray)(i,1);
				double As = (*aarray)(i,3);
				std::complex<double> innerint = std::complex<double>(As,0)
					* exp(j*std::complex<double>(2*wvnum_m*s,0));
				double deltas = (*aarray)(i+1,1) - s;
				totint = totint + (innerint * std::complex<double>(deltas,0));
			}
			double totintsq = std::real(totint * std::conj(totint));

			double Cbk = prefactor * totintsq;

			boost::shared_ptr<cross_sections> res(new cross_sections);
			res->Cbk = Cbk;
			res->Cunits = "m^2";
			res->valid = true;

			return res;
		}


	}
}

