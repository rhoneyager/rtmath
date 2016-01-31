/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>
#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"

#include "../../rtmath/rtmath/error/debug.h"

#include <tmatrixRandom/tmatrix.h>

#include "plugin-tmatrix-random.h"

namespace rtmath
{
	namespace plugins
	{
		namespace tmatrix_random
		{
			void doCrossSection(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
			{
				using namespace ::rtmath::phaseFuncs;
				const double pi = boost::math::constants::pi<double>();

				// First, scale the effective radius and refractive index?
				double scaledAeff = i.aeff;
				double rawVolume = 0, scaledVolume = 0; // scaled is ice + air. raw is ice only.
				if (i.aeff_rescale)
				{
					if (i.aeff_version ==
						pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					{
						rawVolume = pow(i.aeff, 3.0);
						scaledVolume = pow(i.aeff, 3.0) / i.vFrac;
						scaledAeff = pow(scaledVolume, 1. / 3.);
					} else {
						double scaledSA = pow(i.aeff, 2.0);
						scaledSA /= i.vFrac;
						scaledAeff = pow(scaledSA, 0.5);
					}
				}
				double aeffRat = scaledAeff / i.aeff;
				double aeffRatSq = aeffRat * aeffRat;

				std::complex<double> mRes = i.m; 
				std::complex<double> mAir(1.0, 0);
				i.rmeth(i.m, mAir, i.vFrac, mRes);

				// Perform the calculation

				using namespace ::tmatrix_random;
				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0;
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				double ar = 1. / i.eps; // Mish code wants oblate > 1
				if (std::abs(ar - 1.0) < 0.00001) ar = 1.000001;
				auto tp = ::tmatrix_random::tmatrixParams::create(
					scaledAeff, rat, s.wavelength,
					std::abs(mRes.real()), std::abs(mRes.imag()),
					ar, np, 0.001, 7);

				const double k = 2. * pi / s.wavelength;
				const double size_p = 2. * pi * scaledAeff / s.wavelength;

				try {
					auto ori = ::tmatrix_random::OriTmatrix::calc(tp);
					/// \todo Move these scalings into the T-matrix core code?
					c.Qsca = ori->qsca * aeffRatSq;
					c.Qext = ori->qext * aeffRatSq;
					c.Qabs = c.Qext - c.Qsca;
					c.g = ori->g;
					c.Qbk = ori->qbk * aeffRatSq;

					//auto isoAng = ::tmatrix::IsoAngleRes::calc(ori);
					//auto ic = ::tmatrix_random::IsoAngleRes::calc(ori);
					//c.Qbk = ::tmatrix::getDifferentialBackscatterCrossSectionUnpol(isoAng);
				} catch (const ::std::exception& t) {
					std::cerr << "A tmatrix_random error has occurred." << std::endl;
					std::cerr << "\t" << t.what() << std::endl;
					throw(t);
				}
			}

		}
	}
}


D_Ryan_Debug_validator();
D_rtmath_validator();


D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::tmatrix_random;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-Tmatrix-Random",
		"Links to Mishchenko T-matrix code (random version)",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::phaseFuncs::pf_class_registry pcc;
	pcc.name = "tmatrix-rnd";
	pcc.fCrossSections = rtmath::plugins::tmatrix_random::doCrossSection;
	//pcc.fPfs = rtmath::plugins::tmatrix_random::doPf;
	rtmath::phaseFuncs::pf_provider::registerHook(pcc);

	return SUCCESS;
}
