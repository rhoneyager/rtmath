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
				(ar, *, 0.6)
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

			const double pi = boost::math::constants::pi<double>();
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
			// Separate into actual vs. sans air dimensions.
			auto check = [](const std::string &name, const std::string &match, bool &out) {
				if (name.find(match) != std::string::npos ) out = true; };
			bool in_is_ice_only = false, out_is_ice_only = false;
			check(in_length_type, std::string("Ice"), in_is_ice_only);
			check(needsDtype, std::string("Ice"), out_is_ice_only);
			mylog("is input for only ice: " << in_is_ice_only << ", is output for only ice: " << out_is_ice_only);

			/// Can the conversion go directly, or does it need an iteration with successive approximations?
			auto needsIteration = [&]() -> bool {
				if (in_is_ice_only == out_is_ice_only) return false;
				if (!out_is_ice_only) return true;
				return false;
			};

			auto innerGetDen = [&](double inlen, double ar,
				const std::string &in_type) -> double {
				mylog("innerGetDen called");
				double outlen = 0;
				outlen = convertLength( _in_length_value = inlen,
					_in_length_type = in_type,
					_in_length_units = "um",
					_ar = ar,
					_out_length_type = needsDtype,
					_out_length_units = needsUnits);
				double den = (func)(outlen); // either a mass or in g/cm^3
				mylog("innerGetDen\n"
					<< "\tinlen: " << inlen << " as " << in_type << ", with ar " << ar
					<< "\n\toutlen as " << needsDtype << " units " << needsUnits
					<< "\n\tden before mass conversion (if needed): " << den);
				if (relnResult == "mass") {
					// This relation provides a result in mass. It needs to be divided by volume
					// to give a proper density. Has to be handled here.
					double lenAeffUnits = convertLength( _in_length_value = inlen,
						_in_length_type = in_type,
						_in_length_units = "um",
						_out_length_units = needsUnits,
						_ar = ar,
						_out_length_type = "Effective_Radius_Full");
					double V = convertVolumeLength( _ar=ar,
							_in_length_value = lenAeffUnits,
							_in_length_type = "Effective_Radius_Full",
							_out_length_type = "Volume");
					V = units::conv_vol(needsUnits, "cm^3").convert(V);
					den /= V;
					mylog("mass conversion needed\n"
						<< "\tlenAeffUnits: " << lenAeffUnits
						<< "\n\tV: " << V << " cm^3"
						<< "\n\tactual den: " << den << " g/cm^3");
				}
				return den;
			};

			// Take a guessed vf, convert from ice term (aeff) into the ice+air term, stick into density
			// relation, and re-extract the resultant volume fraction.
			auto backConvert = [&](double guess) -> double {
				mylog("backConvert called");
				// The guess is a volume fraction. Other parameters are from context.
				// in_length_um is in ice units.
				double AeffIceUm = convertLength( _in_length_value = in_len_um, _ar = ar,
					_in_length_type = in_length_type, _out_length_type = "Effective_Radius_Ice");
				double VIceUm3 = (4./3.) * pi * pow(AeffIceUm,3.);
				double VFullUm3 = VIceUm3 / guess;
				double AeffFullUm = pow(3.*VFullUm3/(4.*pi),1./3.);
				mylog("backConvert loop A:\n\tguess: " << guess
					<< "\n\tAeffIceUm: " << AeffIceUm
					<< "\n\tVIceUm3: " << VIceUm3
					<< "\n\tVFullUm3: " << VFullUm3
					<< "\n\tAeffFullUm: " << AeffFullUm);
				double den = innerGetDen( AeffFullUm, ar, "Effective_Radius_Full");

				double solidIceDen = 0;
				implementations::findDen(solidIceDen, "ice", temperature, temp_units);
				double resVf = den / solidIceDen;
				mylog("backConvert loop B:\n\tguess: " << guess
					<< "\n\tAeffIceUm: " << AeffIceUm
					<< "\n\tVIceUm3: " << VIceUm3
					<< "\n\tVFullUm3: " << VFullUm3
					<< "\n\tAeffFullUm: " << AeffFullUm
					<< "\n\tden = innerGetDen: " << den << " g/cm^3"
					<< "\n\tsolidIceDen: " << solidIceDen << " g/cm^3"
					<< "\n\tresVf: " << resVf);
				return resVf;
			};

			/// Iterative conversion needed, usually from ice effective radius to max dimension. Guess a vf,
			/// then do the back conversion with the desired method, and successively re-approximate.
			auto convertIterate = [&]() -> double {
				//mylog("convertIterate called");
				double vfa = backConvert(0.0001), vfb = backConvert(1.0);
				//mylog("convertIterate initial bounds a: vf 0.0001: " << vfa << ", b: vf 1.0: " << vfb);
				if (vfa > 1.0 || vfb > 1.0) {
					mylog("The particle size chosen is too small for this size-density relation! Cannot proceed.");
					return 0;
				}
				double vf = zeros::findzero(0.0001, 1.0, [&](double guess) {
					return guess - backConvert(guess); });
				//mylog("convertIterate returned a volume fraction of " << vf);
				// The proper volume fraction is now known. Now determine the effective density.
				// Slightly repetitive, but I prefer it this way.
				double solidIceDen = 0;
				implementations::findDen(solidIceDen, "ice", temperature, temp_units);
				mylog("The density of ice at " << temperature << " " << temp_units << " is " << solidIceDen);
				double effDen = solidIceDen * vf;
				return effDen;
			};

			double effden = 0;

			// Is a back-conversion necessary? If so, calculate the effective density this way.
			if (needsIteration()) {
				mylog("This conversion needs iteration.");
				effden = convertIterate();
			} else { // Can just calculate directly.
				mylog("No iteration needed. Can calculate directly");
				effden = innerGetDen( in_len_um, ar, in_length_type);
			}

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


