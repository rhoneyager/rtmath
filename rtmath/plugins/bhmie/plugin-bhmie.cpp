/// \brief Provides Bohren and Huffman pf and cross-section calculations.
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/units.h"
#include "../../rtmath/rtmath/error/debug.h"

#include "../../related/bhmie/bhmie.h"

#include "plugin-bhmie.h"

namespace rtmath
{
	namespace plugins
	{
		namespace bhmie
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
				auto cl = rtmath::units::converter::getConverter(i.lengthUnits, "m");
				auto cs = rtmath::units::converter::getConverter(s.lengthUnits, "m");
				double scaledAeff = cl->convert(i.aeff);
				if (i.aeff_rescale) // Really to just get the radius. TODO: remove and simplify the code.
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

				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0;
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				double ar = i.eps;
				if (abs(ar - 1.0) < 0.00001) ar = 1.0001;

				const double size_p = 2. * pi * scaledAeff / cs->convert(s.wavelength);
				fcomplex cxref;
				cxref.r = (float) mRes.real();
				cxref.i = -1.f * (float) mRes.imag();
				const unsigned long nang = 2;
				const unsigned long ndirs = 2 * nang; // -1;
				fcomplex cxs1[ndirs], cxs2[ndirs];
				float qext, qsca, qback, gsca;

				try {
					::bhmie((float)size_p, cxref, nang, cxs1, cxs2, &qext, &qsca, &qback, &gsca);

					const double k = 2. * pi / cs->convert(s.wavelength);

					c.Csca = qsca * pow(scaledAeff / cl->convert(i.aeff), 2.);
					c.Cext = qext * pow(scaledAeff / cl->convert(i.aeff), 2.);
					c.Cabs = c.Cext - c.Csca;
					c.g = gsca;
					c.Cbk = qback * pow(scaledAeff / cl->convert(i.aeff), 2.);
					// Currently, these are normalized. Convert into actual cross sections.
					c.Csca *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cext *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cabs *= pi * pow(cl->convert(i.aeff),2.0);
					c.Cbk *= pi * pow(cl->convert(i.aeff),2.0);
					//double C_sphere = pi * pow(scaledAeff, 2.0);
				} catch (...) {
					std::cerr << "A bhmie error has occurred." << std::endl;
					//std::cerr << "\t" << t.what() << std::endl;
					RDthrow(Ryan_Debug::error::xOtherError());
				}
			}

			/*
			void doPf(
				const rtmath::phaseFuncs::pf_class_registry::setup &s,
				const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
				rtmath::phaseFuncs::pf_class_registry::pfs& p)
			{
				using namespace ::rtmath::phaseFuncs;
				const double pi = boost::math::constants::pi<double>();

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

				double rat = (i.aeff_version ==
					pf_class_registry::inputParamsPartial::aeff_version_type::EQUIV_V_SPHERE)
					? 1 : 0;
				int np = (i.shape == pf_class_registry::inputParamsPartial::shape_type::SPHEROID)
					? -1 : -2;
				double ar = i.eps;
				if (abs(ar - 1.0) < 0.00001) ar = 1.0001;

				// Perform the calculation
				try {
					auto tp = ::tmatrix::tmatrixParams::create(
						scaledAeff, rat, s.wavelength, abs(mRes.real()), abs(mRes.imag()), ar, np, 0.001, 7);
					auto ori = ::tmatrix::OriTmatrix::calc(tp, 0, 0);

					auto ang = ::tmatrix::OriAngleRes::calc(ori, s.sTheta, s.sTheta0, 180. - s.sPhi, s.sPhi0);
					for (size_t i = 0; i < 4; ++i)
						for (size_t j = 0; j < 4; ++j)
							p.mueller(i,j) = ang->getP(i, j);
					for (size_t i = 0; i < 2; ++i)
						for (size_t j = 0; j < 2; ++j)
							p.S(i,j) = ang->getS(i, j);
				} catch (...) {
					std::cerr << "A bhmie error has occurred" << std::endl;
					//std::cerr << t.what() << std::endl;
					RDthrow rtmath::debug::xOtherError();
				}
			}
			*/
		}
	}
}

D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	using namespace rtmath::plugins::bhmie;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-bhmie",
		"Links to Bohren and Huffman Mie code",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	//genAndRegisterIOregistry<::rtmath::ddscat::shapefile::shapefile, 
	//	rtmath::ddscat::shapefile::shapefile_IO_output_registry>("silo",PLUGINID);
	rtmath::phaseFuncs::pf_class_registry pc;
	pc.name = "bhmie";
	pc.fCrossSections = rtmath::plugins::bhmie::doCrossSection;
	//pc.fPfs = rtmath::plugins::bhmie::doPf;
	rtmath::phaseFuncs::pf_provider::registerHook(pc);

	return SUCCESS;
}
