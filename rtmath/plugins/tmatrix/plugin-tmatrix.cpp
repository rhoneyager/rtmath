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
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/plugin.h"

#include "../../rtmath/rtmath/error/debug.h"

#include <tmatrix/tmatrix.h>

#include "plugin-tmatrix.h"

namespace rtmath
{
	namespace plugins
	{
		namespace tmatrix
		{
			void doCrossSectionIso(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
			{
				using namespace ::rtmath::phaseFuncs;
				const double pi = boost::math::constants::pi<double>();

				// First, scale the effective radius and refractive index?
				auto cl = rtmath::units::converter::getConverter(i.lengthUnits, "m");
				auto cs = rtmath::units::converter::getConverter(s.lengthUnits, "m");
				double scaledAeff = cl->convert(i.aeff);
				if (i.aeff_rescale)
				{
					if (i.aeff_version ==
						pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					{
						double scaledVolume = pow(cl->convert(i.aeff), 3.0);
						scaledVolume /= i.vFrac;
						scaledAeff = pow(scaledVolume, 1. / 3.);
					} else {
						double scaledSA = pow(cl->convert(i.aeff), 2.0);
						scaledSA /= i.vFrac;
						scaledAeff = pow(scaledSA, 0.5);
					}
				}

				std::complex<double> mRes = i.m;
				std::complex<double> mAir(1.0, 0);
				i.rmeth(i.m, mAir, i.vFrac, mRes);

				// Perform the calculation

				using namespace ::tmatrix;
				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0;
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				double ar = 1. / i.eps; // Mish code wants oblate > 1
				if (std::abs(ar - 1.0) < 0.00001) ar = 1.000001;
				auto tp = ::tmatrix::tmatrixParams::create(
					scaledAeff, rat, cs->convert(s.wavelength),
					std::abs(mRes.real()), std::abs(mRes.imag()),
					ar, np, 0.001, 7, true);

				const double k = 2. * pi / cs->convert(s.wavelength);
				const double size_p = k * scaledAeff;

				try {
					auto ori = ::tmatrix::OriTmatrix::calcIso(tp);
					/// \todo Move these scalings into the T-matrix core code?
					c.Csca = ori->qsca; //* pow(scaledAeff / i.aeff, 2.);
					c.Cext = ori->qext; //* pow(scaledAeff / i.aeff, 2.);
					c.g = ori->g;

					auto isoAng = ::tmatrix::IsoAngleRes::calc(ori);

					c.Cbk = ::tmatrix::getDifferentialBackscatterCrossSectionUnpol(isoAng);

					// Cext (and thus Qext) can come from the optical theorem...
					// Cext = -4pi/k^2 * Re{S(\theta=0)}
					c.Cabs = c.Cext - c.Csca;

					c.Csca *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cext *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cabs *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cbk *= pi * pow(cl->convert(i.aeff),2.0);

				} catch (const ::std::exception& t) {
					std::cerr << "A tmatrix error has occurred." << std::endl;
					std::cerr << "\t" << t.what() << std::endl;
					throw(t);
				}
			}

			void doCrossSection(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
			{
				using namespace ::rtmath::phaseFuncs;
				const double pi = boost::math::constants::pi<double>();

				// First, scale the effective radius and refractive index?
				auto cl = rtmath::units::converter::getConverter(i.lengthUnits, "m");
				auto cs = rtmath::units::converter::getConverter(s.lengthUnits, "m");
				double scaledAeff = cl->convert(i.aeff);
				if (i.aeff_rescale)
				{
					if (i.aeff_version ==
						pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					{
						double scaledVolume = pow(cl->convert(i.aeff), 3.0);
						scaledVolume /= i.vFrac;
						scaledAeff = pow(scaledVolume, 1. / 3.);
					} else {
						double scaledSA = pow(cl->convert(i.aeff), 2.0);
						scaledSA /= i.vFrac;
						scaledAeff = pow(scaledSA, 0.5);
					}
				}

				std::complex<double> mRes = i.m;
				std::complex<double> mAir(1.0, 0);
				i.rmeth(i.m, mAir, i.vFrac, mRes);

				// Perform the calculation

				using namespace ::tmatrix;
				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0;
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				double ar = 1. / i.eps; // Mish code wants oblate ar > 1
				if (std::abs(ar - 1.0) < 0.00001) ar = 1.000001;
				auto tp = ::tmatrix::tmatrixParams::create(
					scaledAeff, rat, cs->convert(s.wavelength), std::abs(mRes.real()), std::abs(mRes.imag()), ar, np, 0.001, 7);

				const double k = 2. * pi / cs->convert(s.wavelength);
				const double size_p = k * scaledAeff;

				try {
					auto ori = ::tmatrix::OriTmatrix::calc(tp, 0, 0);
					/// \todo Move these scalings into the T-matrix core code?
					c.Csca = ori->qsca * pow(scaledAeff / cl->convert(i.aeff), 2.);
					c.Cext = ori->qext * pow(scaledAeff / cl->convert(i.aeff), 2.);

					double C_sphere = pi * pow(scaledAeff, 2.0);
					auto ang = ::tmatrix::OriAngleRes::calc(ori, 0, 0, 180., 0);
					// 4?
					c.Csca = -1; // 4 * 8. * pi / (3. * k * k) * ang->getP(0, 0) / C_sphere / C_sphere; // at theta = 0, phi = pi / 2.
					c.Cbk = ::tmatrix::getDifferentialBackscatterCrossSectionUnpol(ori);
					c.g = -1;

					c.Cbk *= pow(scaledAeff / cl->convert(i.aeff), 2.);
					// Cext (and thus Qext) can come from the optical theorem...
					// Cext = -4pi/k^2 * Re{S(\theta=0)}
					//c.Qext = -4. * pi * ang->getS(0, 0).real() / (k*k*C_sphere);
					//c.Qabs = c.Qext - c.Qsca;

					c.Csca *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cext *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cabs *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cbk *= pi * pow(cl->convert(i.aeff),2.0);
				} catch (const ::std::exception& t) {
					std::cerr << "A tmatrix error has occurred." << std::endl;
					std::cerr << "\t" << t.what() << std::endl;
					throw(t);
				}
			}

			void doPf(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::pfs& p)
			{
				using namespace ::rtmath::phaseFuncs;
				const double pi = boost::math::constants::pi<double>();

				// First, scale the effective radius and refractive index?
				auto cl = rtmath::units::converter::getConverter(i.lengthUnits, "m");
				auto cs = rtmath::units::converter::getConverter(s.lengthUnits, "m");
				double scaledAeff = cl->convert(i.aeff);
				if (i.aeff_rescale)
				{
					if (i.aeff_version ==
						pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					{
						double scaledVolume = pow(cl->convert(i.aeff), 3.0);
						scaledVolume /= i.vFrac;
						scaledAeff = pow(scaledVolume, 1. / 3.);
					}
					else {
						double scaledSA = pow(cl->convert(i.aeff), 2.0);
						scaledSA /= i.vFrac;
						scaledAeff = pow(scaledSA, 0.5);
					}
				}

				std::complex<double> mRes = i.m;
				std::complex<double> mAir(1.0, 0);
				i.rmeth(i.m, mAir, i.vFrac, mRes);

				using namespace ::tmatrix;
				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0;
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				double ar = i.eps;
				if (std::abs(ar - 1.0) < 0.00001) ar = 1.000001;

				// Perform the calculation
				try {
					auto tp = ::tmatrix::tmatrixParams::create(
						scaledAeff, rat, cs->convert(s.wavelength), std::abs(mRes.real()), std::abs(mRes.imag()), ar, np, 0.001, 7);
					auto ori = ::tmatrix::OriTmatrix::calc(tp, 0, 0);

					auto ang = ::tmatrix::OriAngleRes::calc(ori, s.sTheta, s.sTheta0, 180. - s.sPhi, s.sPhi0);
					for (size_t i = 0; i < 4; ++i)
						for (size_t j = 0; j < 4; ++j)
							p.mueller(i,j) = ang->getP(i, j);
					for (size_t i = 0; i < 2; ++i)
						for (size_t j = 0; j < 2; ++j)
							p.S(i,j) = ang->getS(i, j);
				} catch (const ::std::exception& t) {
					std::cerr << "A tmatrix error has occurred" << std::endl;
					std::cerr << t.what() << std::endl;
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
	using namespace rtmath::plugins::tmatrix;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-Tmatrix-ori",
		"Links to Mishchenko T-matrix code (oriented version)",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;


	rtmath::phaseFuncs::pf_class_registry pcb;
	pcb.name = "tmatrix-ori";
	pcb.fCrossSections = rtmath::plugins::tmatrix::doCrossSectionIso;
	//pcb.fPfs = rtmath::plugins::tmatrix::doPf;
	//rtmath::phaseFuncs::pf_provider::registerHook(pcb);
	return SUCCESS;
}
