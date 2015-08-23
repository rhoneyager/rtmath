#pragma once
#include "../defs.h"
#include <complex>
#include <functional>
#include <string>
#include <sstream>
#include <vector>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/name.hpp>
#include <boost/parameter/preprocessor.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include "../units.h"
#include "../zeros.h"
#include "../conversions/convertLength.h"
#include "densityImpl.h"
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging_base.h>

#undef FL
#undef mylog
//#define FL __FILE__ << ", " << (int)__LINE__ << ": "
#define FL "density.h, line " << (int)__LINE__ << ": "

#define mylog(x) { std::ostringstream l; l << FL << x; rtmath::density::implementations::emit_density_log(l.str()); }

namespace rtmath
{
	/// \brief Functions to calculate the density of ice 1-h and water 
	/// (regular and supercooled) for a given temperature.
	///
	/// \note These are all at standard pressure (1013.25 hPa)!!!!!
	/// \todo Switch tags to CITE and use bibtex
	namespace density
	{
		/// Dielectric providers - these use f and T to automatically determine the correct
		/// base dielectric function to use.

#define standardTProvider(name) \
	BOOST_PARAMETER_FUNCTION( \
		(double), \
			name, \
			::rtmath::units::keywords::tag, \
			(required \
			(temperature, (double)) \
			) \
			(optional \
			(temp_units, *, std::string("K"))) \
			) \
		{ \
			double temp = rtmath::units::conv_temp(temp_units, "K").convert(temperature); \
			return implementations:: name(temp); \
		}

		standardTProvider(ice1h);
		standardTProvider(SuperWater);
		standardTProvider(water);


		/** \brief Provides effective densities for use in particle modeling.
		 *
		 * Takes input in terms of the particle dimension (max dimension, ice effective radius,
		 * etc.), temperature and substance (ice).
		 *
		 * \note This routine assumes that the aspect ratio is fixed at a user-provided
		 * or default value.
		 * \returns Effective density in g/cm^3.
		 **/
	BOOST_PARAMETER_FUNCTION(
		(double),
			effDen,
			::rtmath::units::keywords::tag,
			(required
				(in_length_value, (double))
				(in_length_type, (std::string))
				(provider, (std::string))
			)
			(optional
				(in_length_units, *, std::string("m"))
				(ar, *, 1.0)
				(temperature, *, 263)
				(temp_units, *, std::string("K"))
			)
			)
		{
			using namespace ::rtmath::units::keywords;
			using namespace ::rtmath::units;
			mylog( "effDen\n\tin_length: " << in_length_value << " " << in_length_units
				<< " as " << in_length_type
				<< "\n\tprovider: " << provider
				<< "\n\tar: " << ar
				<< "\n\ttemperature: " << temperature
				<< " " << temp_units);

			//const double pi = boost::math::constants::pi<double>();
			bool found = false;
			std::string needsDtype, needsUnits, needsSubstance, relnResult;
			std::function<double(double)> func;
			found = implementations::findProvider(provider, func,
				needsDtype, needsUnits, needsSubstance, relnResult);
			if (!found) RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::specializer_type(provider)
				<< Ryan_Debug::error::otherErrorText("Unknown density provider");

			double in_len_um = rtmath::units::conv_alt(in_length_units, "um").convert(in_length_value);
			mylog("Converting length to um gives " << in_len_um << " um");

			// Can the conversion go directly, or does it need an iteration with
			// successive approximations? Basically, see if the input quantity is a
			// 1) effective radius or diameter or total volume, or
			// 2) it is a Max or Min radius/diameter
			auto check = [](const std::string &name, const std::string &match, bool &out, bool &out2) {
				if (name.find(match) != std::string::npos) {
					//bool pout = out, pout2 = out2;
					out = true; out2 = true;
					//mylog("\n\tmatched " << match << " in " << name
					//	<< "\n\t prev out " << pout << ", prev out2 " << pout2);
				} };
			// These statements could be combined some more.
			bool inIsV = false, inIsMD = false, inIsMax = false, junk = false, inIsRad = false;
			check(in_length_type, "Max_Dimension", inIsMD, inIsMax);
			check(in_length_type, "Max_Diameter", inIsMD, inIsMax);
			check(in_length_type, "Max_Radius", inIsMD, inIsMax);
			check(in_length_type, "Min_Dimension", inIsMD, junk);
			check(in_length_type, "Min_Diameter", inIsMD, junk);
			check(in_length_type, "Min_Radius", inIsMD, junk);
			check(in_length_type, "Volume", inIsV, junk);
			check(in_length_type, "adius", inIsRad, junk);

			bool outIsV = false, outIsMD = false, outIsMax = false, outIsRad = false;
			check(needsDtype, "Max_Dimension", outIsMD, outIsMax);
			check(needsDtype, "Max_Diameter", outIsMD, outIsMax);
			check(needsDtype, "Max_Radius", outIsMD, outIsMax);
			check(needsDtype, "Min_Dimension", outIsMD, junk);
			check(needsDtype, "Min_Diameter", outIsMD, junk);
			check(needsDtype, "Min_Radius", outIsMD, junk);
			check(needsDtype, "Volume", outIsV, junk);
			check(needsDtype, "adius", outIsRad, junk);

			mylog("\n\tin_length_type = " << in_length_type <<
				"\n\tinIsV " << inIsV << "\n\tinIsMD " << inIsMD <<
				"\n\tinIsMax " << inIsMax << "\n\tinIsRad " << inIsRad <<
				"\n\toutIsV " << outIsV << "\n\toutIsMD " << outIsMD <<
				"\n\toutIsMax " << outIsMax << "\n\toutIsRad " << outIsRad
				<< "\n\tNow to perform the iterations");

			/// Can the conversion go directly, or does it need
			/// an iteration with successive approximations?
			//auto needsIteration = [&]() -> bool {
			//	if (inIsMD && outIsMD) return false;
			//	if (!outIsMD) return false;
			//	return true;
			//};

			auto innerGetDen = [&](double inlen, double ar, double invf,
				const std::string &in_type) -> double {
				double outlen = 0;
				// inlen is an effective radius. So, output conversion uses _out_volume_fraction.
				outlen = convertLength( _in_length_value = inlen,
					_in_length_type = in_type,
					_out_volume_fraction = invf,
					_ar = ar,
					_out_length_type = needsDtype);
				// convert to correct units here. inlen is always in um (see calling function)
				outlen = rtmath::units::conv_alt("um", needsUnits).convert(outlen);
				double arnorm = ar;
				if (ar > 1.) arnorm = 1./ar;
				double den = (func)(outlen); // either a mass (in g) or a density in g/cm^3
				mylog("innerGetDen\n"
					<< "\tinlen: " << inlen << " as " << in_type << ", with ar " << ar
					<< "\tinvf: " << invf
					<< "\n\toutlen: " << outlen << " as " << needsDtype << " units " << needsUnits
					<< "\n\tden before mass conversion (if needed): " << den);
				if (relnResult == "mass") {
					double in_len_needed = rtmath::units::conv_alt("um", "cm").convert(inlen);
					// This relation provides a result in mass. It needs to be divided by volume
					// to give a proper density. Has to be handled here.
					double V = convertLength( _in_length_value = in_len_needed,
						_in_length_type = in_type,
						_in_volume_fraction = invf,
						_ar = ar,
						_out_length_type = "Volume");
					V /= invf; // TODO: may need to tweak convertLength...
					//double Vcm = units::conv_vol(needsUnits, "cm^3").convert(V);
					double mass = den;
					den /= V;
					den *= arnorm; // SLightly different presentation.
					mylog("mass conversion needed\n"
						<< "\n\tmass " << mass << " g"
						<< "\n\tin_len " << in_len_needed << " cm as " << in_type
						<< "\n\tV: " << V << " cm^3"
						//<< "\n\tVcm: " << Vcm << " cm^3"
						<< "\n\tactual den: " << den << " g/cm^3");
				}
				//den *= invf; // TODO: may need to tweak convertLength
				return den / arnorm;
			};

			// Take a guessed vf, convert from ice term (aeff) into the ice+air term, stick into density
			// relation, and re-extract the resultant volume fraction.
			auto backConvert = [&](double guessvf) -> double {
				// The guess is a volume fraction. Other parameters are from context.
				// in_length_um is in ice units.
				double AeffUm = convertLength( _in_length_value = in_len_um, _ar = ar,
					_in_volume_fraction = guessvf,
					_in_length_type = in_length_type,
					_out_length_type = "Effective_Radius");
				//double VIceUm3 = (4./3.) * pi * pow(AeffIceUm,3.);
				//double VFullUm3 = VIceUm3 / guessvf;
				//double AeffFullUm = pow(3.*VFullUm3/(4.*pi),1./3.);
				//mylog("backConvert loop A:\n\tguessvf: " << guessvf
				//	<< "\n\tAeffIceUm: " << AeffIceUm
				//	<< "\n\tVIceUm3: " << VIceUm3
				//	<< "\n\tVFullUm3: " << VFullUm3
				//	<< "\n\tAeffFullUm: " << AeffFullUm);
				double den = innerGetDen( AeffUm, ar, guessvf, "Effective_Radius");

				double solidIceDen = 0;
				implementations::findDen(solidIceDen, "ice", temperature, temp_units);
				double resVf = den / solidIceDen;
				mylog("backConvert called:\n\tguessvf: " << guessvf
					<< "\n\tAeffUm: " << AeffUm << " ar " << ar
					<< "\n\t_in_length_value: " << in_len_um << " type " << in_length_type
					<< "\n\tden = innerGetDen: " << den << " g/cm^3"
					<< "\n\tsolidIceDen: " << solidIceDen << " g/cm^3"
					<< "\n\tresVf: " << resVf);
				return resVf;
			};

			/// Iterative conversion needed, usually from ice effective radius to max dimension. Guess a vf,
			/// then do the back conversion with the desired method, and successively re-approximate.
			auto convertIterate = [&]() -> double {
				mylog("convertIterate called. Will evaluate using backconvert for vfa 0.0001 and vfb 1.0");
				double vfa = backConvert(0.0001), vfb = backConvert(1.0);
				//mylog("convertIterate initial bounds a: vf 0.0001: " << vfa << ", b: vf 1.0: " << vfb);
				if (vfa > 1.0 || vfb > 1.0) {
					mylog("The particle size chosen is too small for this size-density relation! Cannot proceed.");
					return 0;
				}
				mylog("Loop to find zeros (in function convertIterate)"
					<< "\n\tvfa(0.0001) was " << vfa << " and vfb(1.0) was " << vfb)
				double vf = 0;
				if (abs((vfa-vfb)/vfa) < 0.000001) {
					mylog("Since vfa ~== vfb, the volume fraction doesn't need a loop. This "
						"happens when the transform already has all necessary info.");
					vf = vfa;
				}
				else vf = zeros::findzero(0.0001, 1.0, [&](double guess) {
					return guess - backConvert(guess); });
				//mylog("convertIterate returned a volume fraction of " << vf);
				// The proper volume fraction is now known. Now determine the effective density.
				// Slightly repetitive, but I prefer it this way.
				double solidIceDen = 0;
				implementations::findDen(solidIceDen, "ice", temperature, temp_units);
				mylog("The density of ice at " << temperature << " " << temp_units 
					<< " is " << solidIceDen << " g/cm^3");
				double effDen = solidIceDen * vf;
				return effDen;
			};

			double effden = 0;

			// Is a back-conversion necessary? If so, calculate the effective density this way.
			//if (needsIteration()) {
			//	mylog("This conversion needs iteration.");
				effden = convertIterate();
			//} else { // Can just calculate directly.
			//	mylog("No iteration needed. Can calculate directly");
			//	effden = innerGetDen( in_len_um, ar, invf, in_length_type); // TODO: invf
			//}

			mylog("\teffden is " << effden << " g/cm^3");
			return effden;
		}

#define standardDProvider(name) \
	template <class ArgumentPack> \
	double name (ArgumentPack const &args) { \
		ArgumentPack newArgs = (args, ::rtmath::units::keywords::_provider = name); \
		return effDen(newArgs); \
	};
		// Mass-density relationships for ice crystals
		// Note that some of these use ice diameter, while others use liquid water diameter. convertDensity
		// handles this.
		standardDProvider(BrownFrancis1995Hogan2012);
		standardDProvider(Brandes2007);
		standardDProvider(MagonoNakamura1965);
		standardDProvider(Holroyd1971);
		standardDProvider(Muramoto1995);
		standardDProvider(FabrySzyrmer1999);
		standardDProvider(Heymsfield2004);
	}
}


