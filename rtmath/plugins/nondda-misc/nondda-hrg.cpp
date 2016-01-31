/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>
#include <boost/math/constants/constants.hpp>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/refract.h"
#include "../../rtmath/rtmath/phaseFunc.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/conversions/convertLength.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-nondda-misc.h"

namespace rtmath
{
	namespace plugins
	{
		namespace nondda_misc
		{

			namespace HRG {
				void doCrossSection(
					const rtmath::phaseFuncs::pf_class_registry::setup &s,
					const rtmath::phaseFuncs::pf_class_registry::inputParamsPartial& i,
					rtmath::phaseFuncs::pf_class_registry::cross_sections& c)
				{
					using namespace ::rtmath::phaseFuncs;
					const double pi = boost::math::constants::pi<double>();

					// First, scale the effective radius and refractive index?
					double scaledAeff = i.aeff;

					// Force refractive index of ice
					std::complex<double> mRes = i.m;
					//std::complex<double> mAir(1.0, 0);
					//i.rmeth(i.m, mAir, i.vFrac, mRes);

					// Perform the calculation
					double ar = i.eps;
					bool prolate = false;
					if (ar > 1.0) {
						prolate = true;
						ar = 1./ar;
					}
					double Dlong = i.maxDiamFull;
					double Dshort = 0;

					// What is the angle of incidence?
					std::string sIncid = "vertical";
					if (i.other->hasVal("incid_angle"))
						sIncid = i.other->getVal<std::string>("incid_angle");

					if (sIncid == "vertical")
						Dshort = Dlong * ar;
					else if (sIncid == "horizontal")
						Dshort = Dlong;
					else if (sIncid == "random") {
						using namespace rtmath::units;
						using namespace rtmath::units::keywords;
						Dshort = convertLength(
							_in_length_value = i.maxDiamFull,
							_in_length_type = "Max_Diameter",
							_out_length_type = "Mean_Diameter",
							_ar = ar);
					}
					else
						RDthrow(Ryan_Debug::error::xBadInput()) 
							<< Ryan_Debug::error::key(sIncid)
							<< Ryan_Debug::error::otherErrorText("Unsupported value of incid_angle");

					const double k = 2. * pi / s.wavelength;
					const double size_p = k * Dshort;
					float qext = -1, qsca = -1, qback = -1, gsca = -1;
					const double f = i.vFrac;
					const double V = 4. / 3. * pi * std::pow(i.aeff, 3.); //i.aeff, scaledAeff
					//const double dmax = 2. * std::pow(V*3./(4.*pi*ar),1./3.); // oblate particles
					//const double d = ar * dmax; // d is the vertical dimension, d is the horizontal dimension.

					// These defaults are from Hogan and Westbrook (2014),
					// using Westbrook's aggregates as a base.
					// Default assumes aligned and vertical incidence.
					double gamma = 0, beta = 0, kappa = 0;
					std::string srcModel = "HW2014", srcParticles = "agg_rosettes";
					// Default is "HW2014" - Westbrook aggregates
					// Other option is "NLH2013" - our rounded aggregates
					if (i.other->hasVal("srcModel"))
						srcModel = i.other->getVal<std::string>("srcModel");
					if (i.other->hasVal("srcParticles"))
						srcParticles = i.other->getVal<std::string>("srcParticles");
					if (srcModel == "NLH2013") {
						// Only works for random orientation
						gamma = 5./3.;
						beta = 0.15;
						kappa = 0.0;
					} else if (srcModel == "HW2014") {
						// Default HW2014
						// Values depend on incidence angle
						gamma = 5./3.;
						if (srcParticles == "agg_rosettes") {
							if (sIncid == "vertical") {
								beta = 0.23;
								kappa = 0.19;
							}
							else if (sIncid == "horizontal") {
								beta = 0.56;
								kappa = -0.11;
							}
							else if (sIncid == "random") {
								beta = 0.45;
								kappa = 0.00;
							}
						} else if (srcParticles == "agg_plates") {
							if (sIncid == "vertical") {
								beta = 0.21;
								kappa = 0.18;
							}
							else if (sIncid == "horizontal") {
								beta = 0.61;
								kappa = -0.12;
							}
							else if (sIncid == "random") {
								beta = 0.51;
								kappa = -0.05;
							}
						}
					}

					// If they exist in i.other, then set to the specified
					// values.
					if (i.other->hasVal("gamma"))
						gamma = i.other->getVal<double>("gamma");
					if (i.other->hasVal("beta"))
						beta = i.other->getVal<double>("beta");
					if (i.other->hasVal("kappa"))
						kappa = i.other->getVal<double>("kappa");

					if (!gamma || !beta) { // kappa may equal zero
						RDthrow(Ryan_Debug::error::xBadInput())
							<< Ryan_Debug::error::key("SSRG")
							<< Ryan_Debug::error::otherErrorText("beta & gamma & kappa = 0. "
								"Check incid_angle, srcModel and srcParticles, "
								"or manually specify beta, gamma and kappa in i.other->");
					}
					try {
						std::complex<double> K = ((i.m*i.m) - 1.) / ((i.m*i.m) + 2.);
						std::complex<double> Km = ((mRes*mRes) - 1.) / ((mRes*mRes) + 2.);
						double K2 = pow(abs(Km),2.);
						qext = -1;
						qsca = -1;
						double prefactor = 9. * pi * pow(k,4.)* K2 * V * V / 16.;
						double term1 = cos(size_p) * ( (1.+kappa/3.)*(1./(2.*size_p+pi) - 1./(2.*size_p-pi))
								- kappa * (1./(2.*size_p+3*pi) - 1./(2.*size_p - 3.*pi)));
						term1 *= term1;
						int jmax = (int) ( 5.*size_p / pi) + 1;
						qback = 0;
						for (int j=1; j <= jmax; ++j) {
							qback += pow(2.*j,-gamma) * sin(size_p) * sin(size_p)
								* ( pow(1./(2.*(size_p+(pi*j))),2.)
								+ pow(1./(2.*(size_p - (pi*j))),2.) );
						}
						qback *= beta;
						qback += term1;
						qback *= prefactor;
						qback /= pi * pow(i.aeff,2.);

						c.Qsca = -1;
						c.Qbk = qback;
						c.g = -1;
						c.Qext = -1;
						c.Qabs = -1;

					}
					catch (std::exception &e)
					{
						std::cerr << "An SSRG error has occurred!" << std::endl;
						throw e;
					}
					catch (...) {
						//std::cerr << "\t" << t.what() << std::endl;
						RDthrow(Ryan_Debug::error::xOtherError()) 
							<< Ryan_Debug::error::otherErrorText("An SSRG error has occurred");
					}
				}
			}
		}
	}
}


