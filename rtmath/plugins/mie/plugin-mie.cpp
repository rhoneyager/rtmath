/// \brief Provides mie scattering
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>
#include <cmath>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "mie.h"

#include "plugin-mie.h"

namespace rtmath
{
	namespace plugins
	{
		namespace mie
		{

			void doCrossSection(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
			{
				using namespace ::rtmath::phaseFuncs;
				const double pi = boost::math::constants::pi<double>();

				if (abs(i.eps - 1) > 0.001)
				{
					c.valid = false;
					return;
				}

				// First, scale the effective radius and refractive index?
				double scaledAeff = i.aeff;
				if (i.aeff_rescale)
				{
					if (i.aeff_version ==
						pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					{
						double scaledVolume = pow(i.aeff, 3.0);
						scaledVolume /= i.vFrac;
						scaledAeff = pow(scaledVolume, 1. / 3.);
					} else {
						double scaledSA = pow(i.aeff, 2.0);
						scaledSA /= i.vFrac;
						scaledAeff = pow(scaledSA, 0.5);
					}
				}

				std::complex<double> mRes = i.m; 
                std::complex<double> mAir(1.0, 0);
				i.rmeth(i.m, mAir, i.vFrac, mRes);

				// Perform the calculation

				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0; // TODO: Add support for equiv_sa_spheres.
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				//std::cerr << "mRes " << mRes << " m " << i.m << " vf " << i.vFrac << " mAir " << mAir << std::endl;
				//std::cerr << mRes.real() << "\t\t " << mRes.imag() << std::endl;
				auto tp = mieParams::create(
					scaledAeff, s.wavelength, std::abs(mRes.real()), -1.0 * std::abs(mRes.imag()), 0.001);

				const double k = 2. * pi / s.wavelength;
				const double size_p = 2. * pi * scaledAeff / s.wavelength;

				try {
					auto ori = mieCalc::calc(tp);
					/// \todo Move these scalings into the T-matrix core code?
					c.Qsca = ori->qsca * pow(scaledAeff / i.aeff, 2.);
					c.Qext = ori->qext * pow(scaledAeff / i.aeff, 2.);
					c.Qabs = ori->qabs * pow(scaledAeff / i.aeff, 2.);
					c.g = ori->g;
					c.Qbk = ori->qbk * pow(scaledAeff / i.aeff, 2.);

					double C_sphere = pi * pow(scaledAeff, 2.0);
					//auto ang = mieAngleRes::calc(ori, 180.);
					// 4?
					//c.Qsca = -1; // 4 * 8. * pi / (3. * k * k) * ang->getP(0, 0) / C_sphere / C_sphere; // at theta = 0, phi = pi / 2.
					//c.Qbk = getDifferentialBackscatterCrossSectionUnpol(ori);
					//c.g = -1;


					//c.Qbk_iso = c.Qbk * pow(scaledAeff / i.aeff, 2.);
					// Cext (and thus Qext) can come from the optical theorem...
					// Cext = -4pi/k^2 * Re{S(\theta=0)}
					//c.Qext = -4. * pi * ang->getS(0, 0).real() / (k*k*C_sphere);
					//c.Qabs = c.Qext - c.Qsca;

					/// iso values are validated with solid spheres and soft spheres using liu code
					/// \todo need to validate with ellipsoids

					//std::cerr << c.Qabs_iso << "\t" << c.Qsca_iso << "\t" << c.Qext_iso << "\t" << c.Qbk_iso << std::endl;
				} catch (const ::Ryan_Debug::error::xError& t) {
					std::cerr << "A mie error has occurred." << std::endl;
					std::cerr << "\t" << t.what() << std::endl;
					RDthrow(Ryan_Debug::error::xOtherError());
				}
			}

			void doPf(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::pfs& p)
			{
				using namespace ::rtmath::phaseFuncs;

				if (abs(i.eps - 1) > 0.001)
				{
					p.valid = false;
					return;
				}

				const double pi = boost::math::constants::pi<double>();
				// First, scale the effective radius and refractive index?
				// First, scale the effective radius and refractive index?
				double scaledAeff = i.aeff;
				if (i.aeff_rescale)
				{
					if (i.aeff_version ==
						pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					{
						double scaledVolume = pow(i.aeff, 3.0);
						scaledVolume /= i.vFrac;
						scaledAeff = pow(scaledVolume, 1. / 3.);
					}
					else {
						double scaledSA = pow(i.aeff, 2.0);
						scaledSA /= i.vFrac;
						scaledAeff = pow(scaledSA, 0.5);
					}
				}

				std::complex<double> mRes = i.m;
				std::complex<double> mAir(1.0, 0);
				i.rmeth(i.m, mAir, i.vFrac, mRes);

				// Perform the calculation
				try {
					auto tp = mieParams::create(
						scaledAeff, s.wavelength, mRes.real(), mRes.imag());
					auto ori = mieCalc::calc(tp);

					auto ang = mieAngleRes::calc(ori, s.sTheta);
					for (size_t i = 0; i < 4; ++i)
						for (size_t j = 0; j < 4; ++j)
							p.mueller(i,j) = ang->getP(i, j);
					for (size_t i = 0; i < 2; ++i)
						for (size_t j = 0; j < 2; ++j)
							p.S(i,j) = ang->getS(i, j);
				} catch (const ::Ryan_Debug::error::xError& t) {
					std::cerr << "A mie error has occurred" << std::endl;
					std::cerr << t.what() << std::endl;
					RDthrow(Ryan_Debug::error::xOtherError());
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
	using namespace rtmath::plugins::mie;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-mie",
		"My independent implementation of Mie theory",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	rtmath::phaseFuncs::pf_class_registry pc;
	pc.name = "mie";
	pc.fCrossSections = rtmath::plugins::mie::doCrossSection;
	pc.fPfs = rtmath::plugins::mie::doPf;
	rtmath::phaseFuncs::pf_provider::registerHook(pc);

	return SUCCESS;
}
