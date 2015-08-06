#pragma once
#include "defs.h"
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
#include "units.h"
#include "zeros.h"
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging_base.h>

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
	/// \todo Add ice selector and other ice models
	/// \todo Add liquid water model
	namespace density
	{
		namespace implementations {
		/// Internal function used in templates that writes to the registry log
		void DLEXPORT_rtmath_core emit_density_log(const std::string&, ::Ryan_Debug::log::severity_level = ::Ryan_Debug::log::debug_2);

			/// \note All density outputs are in g/cm^3!!!

		/* For ice and supercooled water, 
		* taken from the 2011-2012 (92nd) ed. of the Handbook of Chemistry and Physics
		* http://www.hbcpnetbase.com/
		* Saved as 06_44_91.pdf
		*/

		/** \brief Density of ice 1h at standard pressure
		*
		* \note Feistel, R ., and Wagner, W ., J. Phys. Chem. Ref. Data 35, 1021, 2006 .
		* 3 .  International Association for the Properties of Water and Steam 
		* (IAPWS), Revised Release on the Equation of State 2006 for H2
		* O Ice Ih
		* (2009), available from http://www .iapws .org
		**/
			double DLEXPORT_rtmath_core ice1h(double Tk);
		/** \brief Density of supercooled water
		* \note Wagner, W ., and Pruß A ., J. Phys. Chem. Ref. Data 31, 387, 2002
		**/
			double DLEXPORT_rtmath_core SuperWater(double Tk);
		/// Selector function that uses the most appropriate model for water density 
		/// for a given temperature.
		/// \todo Implement this for density above freezing.
			double DLEXPORT_rtmath_core water(double Tk);

		/// Selector function to find density
			void DLEXPORT_rtmath_core findDen(double &den, const std::string &subst,
					double temperature, const std::string &temp_units);
		/**
		* \brief Brown and Francis (1995) mass-size relationship, but using Hogan et al.'s
		* (2012) conversion to be in terms of the longest particle dimension.
		*
		* the original BF95 expression was in terms of the "mean" dimension, which
		* was the mean of the maximum and minimum dimensions of particles as viewed
		* by their aircraft instruments. From Hogan and Westbrook (2014)
		**/
			double DLEXPORT_rtmath_core BrownFrancis1995Hogan2012(double Dlong);

			/// For all of these, D is the particle equivalent volume diameter. 
			/// Conversion to and from the longest particle dimension is performed by 
			/// the container function.
			/**
			 * \brief Brandes et al. (2007) size-density relationship.
			 **/
			double DLEXPORT_rtmath_core Brandes2007(double D);

			/// Magono and Nakamura (1965) snowflake density-particle size relation
			double DLEXPORT_rtmath_core MagonoNakamura1965(double D);

			/// Holroyd (1971) ...
			double DLEXPORT_rtmath_core Holroyd1971(double D);

			/// Muramoto et al. (1995) ...
			double DLEXPORT_rtmath_core Muramoto1995(double D);

			/// Fabry and Szyrmer (1999) ...
			double DLEXPORT_rtmath_core FabrySzyrmer1999(double D);

			/// Heymsfield et al. (2004) ...
			double DLEXPORT_rtmath_core Heymsfield2004(double D);

			/** \brief Determines which function shopuld be used to handle the density formulation.
			 *
			 * \param name is the name of the relation used.
			 * \param func is a returned pointer to the proper function.
			 * \param in_type indicates if effective ice or equivalent whole (ice+air) measurements are used.
			 * \param in_units is the length dimension passed to this function.
			 * \param in_subst is the substance used in this function (ice or water)
			 * \param out_quantity is the output quantity (mass (g) or density in g/cm^3).
			 * \returns bool Indicating success or failure to match the relation.
			 **/
			bool DLEXPORT_rtmath_core findProvider(const std::string& name, std::function<double(double)> &func,
					std::string& in_type, std::string& in_units,
					std::string& in_subst, std::string& out_quantity);

		}

		/// Dielectric providers - these use f and T to automatically determine the correct
		/// base dielectric function to use.
		BOOST_PARAMETER_NAME(frequency)
		BOOST_PARAMETER_NAME(temperature)
		BOOST_PARAMETER_NAME(salinity)
		BOOST_PARAMETER_NAME(temp_units)
		BOOST_PARAMETER_NAME(freq_units)
		BOOST_PARAMETER_NAME(salinity_units)
		BOOST_PARAMETER_NAME(m)
		BOOST_PARAMETER_NAME(volume_fraction)
		BOOST_PARAMETER_NAME(provider)
		BOOST_PARAMETER_NAME(in_length_value)
		BOOST_PARAMETER_NAME(max_dimension)
		BOOST_PARAMETER_NAME(mean_volume_diameter)
		BOOST_PARAMETER_NAME(mean_volume_radius)
		BOOST_PARAMETER_NAME(in_length_units)
		BOOST_PARAMETER_NAME(in_length_type)
		BOOST_PARAMETER_NAME(out_length_type)
		BOOST_PARAMETER_NAME(out_length_units)
		BOOST_PARAMETER_NAME(ar)
		BOOST_PARAMETER_NAME(in_density)
		BOOST_PARAMETER_NAME(out_density)
		BOOST_PARAMETER_NAME(in_volume)
		BOOST_PARAMETER_NAME(in_aeff)
		BOOST_PARAMETER_NAME(in_substance)
		BOOST_PARAMETER_NAME(out_substance)

#define standardTProvider(name) \
	BOOST_PARAMETER_FUNCTION( \
		(double), \
			name, \
			tag, \
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


		/// Convert between equivalent volumes for different substanses / phases
		/// Densities should be in g/cm^3.
		BOOST_PARAMETER_FUNCTION( (double),
			convertSubstanceDensity,
			tag,
			(optional
				(in_density, *, 0)
				(out_density, *, 0)
				(in_volume, *, 0)
				(in_aeff, *, 0)
				(temperature, *, 0)
				(temp_units, *, std::string("K"))
				(in_substance, *, std::string(""))
				(out_substance, *, std::string(""))
			) )
		{
				mylog( "convertSubstanceDensity\n"
					"\tin_density: " << in_density
				<< "\n\tout_density: " << out_density
				<< "\n\tin_volume: " << in_volume
				<< "\n\tin_aeff: " << in_aeff
				<< "\n\ttemp: " << temperature << " " << temp_units
				<< "\n\tin_subst: " << in_substance
				<< "\n\tout_subst: " << out_substance << std::endl);
			double inDen = in_density;
			double outDen = out_density;
			double inV = in_volume;
			double inAeff = in_aeff;
			bool doAeff = (inAeff > 0) ? true : false;
			if (!inV && !inAeff) RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::otherErrorText("Function requires either an input volume or "
						"effective radius to perform the conversion.");
			double outval = 0;
			if (in_substance == out_substance) {
				if (doAeff) return in_aeff;
				return in_volume;
			}
			// Suppressing the factors of pi, since this function always converts back to 
			// the same quantity specified in the input. As such, it only takes an effective 
			// radius of a volume. Max diameter or effective diameter won't work. Use convertLength.
			//const double pi = boost::math::constants::pi<double>();
			if (doAeff) inV = (4./3.) * pow(inAeff,3.);
			// Determine densities
			implementations::findDen(inDen, in_substance, temperature, temp_units);
			implementations::findDen(outDen, out_substance, temperature, temp_units);

			mylog("\n\tinDen " << inDen << "\toutDen: " << outDen << std::endl);
			// Once the densities are determined, then do the conversion of the volumes.
			// den = m / v.
			double mass = inDen * inV;
			outval = mass / outDen;

			if (doAeff) { // convert back to effective radius if requested
				outval = pow(3.*outval/(4.),1./3.);
			}
			mylog("\n\tresult is " << outval << " and doAeff is " << doAeff << std::endl);
			return outval;
		}

		/// Function to convert between different length measurements
		BOOST_PARAMETER_FUNCTION( (double),
			convertLength,
			tag,
			(required
				(in_length_value, (double))
				(in_length_type, (std::string))
				(out_length_type, (std::string))
			)
			(optional
				(ar, *, 0.6)
				(in_length_units, *, std::string("um"))
				(out_length_units, *, std::string("um"))
			) )
		{
			mylog( "Converting length.\n"
				<< "\tFrom: " << in_length_value << " " << in_length_units << " in " << in_length_type
				<< "\n\tto: " << out_length_units << " as " << out_length_type
				<< "\n\tar: " << ar);
			// First convert to the right dimensional units
			double inVal = rtmath::units::conv_alt(in_length_units, out_length_units).convert(in_length_value);
			// Next, pass the desired conversion formulas. The main conversions are between (radius,diameter)
			// effective volume units and max dimension units.

			bool inIsMD = false, outIsMD = false;
			// If input value is a radius, convert to a diameter by multiplying by 2.
			double inDiam = inVal;
			if (in_length_type.find("adius") != std::string::npos) inDiam *= 2.;
			auto check = [](const std::string &name, const std::string &match, bool &out) {
				if (name == match) out = true; };
			// Max_Dimension vs. Median_Diameter
			check(in_length_type, "Max_Dimension", inIsMD);
			check(in_length_type, "Max_Diameter", inIsMD);
			check(in_length_type, "Max_Radius", inIsMD);
			check(out_length_type, "Max_Dimension", outIsMD);
			check(out_length_type, "Max_Diameter", outIsMD);
			check(out_length_type, "Max_Radius", outIsMD);
			/// \todo add Min_Diameters and Mean_Diameters

			double outVal = 0;
			double inVnp = 0; // Volume without the factor of pi.
			if (inIsMD == outIsMD) outVal = inDiam;
			else if (inIsMD) {
				double meanDiam = 0;
				// Convert from max diameter to the mean diameter. AR is known, so this is easy.
				if (ar <= 1) { // oblate
					inVnp = pow(inDiam,3.) / (6. * ar);
				} else { // prolate
					inVnp = pow(inDiam,3.) * pow(ar,2.) / 6.;
				}
				meanDiam = 2. * pow(3. * inVnp / 4., 1./3.);
				outVal = meanDiam;
			} else {
				double maxDiam = 0;
				inVnp = (4./3.) * pow(inDiam/2.,3.);
				// Convert from mean diameter to max diameter.
				if (ar <= 1) { // oblate
					maxDiam = pow(6. * inVnp * ar, 1./3.);
				} else { // prolate
					maxDiam = pow(6. * inVnp / (ar * ar), 1./3.);
				}
				outVal = maxDiam;
			}
			if(out_length_type.find("adius") != std::string::npos) outVal /= 2.;
			mylog("\toutput is " << outVal << " " << out_length_units << " as " << out_length_type);
			return outVal;
		}

		/// Function that calculates volume based on different aspect ratios. Assumes
		/// that the volume has the units of length^3.
		BOOST_PARAMETER_FUNCTION( (double),
			convertVolumeLength,
			tag,
			(required
				(in_length_value, (double))
				(in_length_type, (std::string)) // can be Volume or a length that convertLength can handle
				(out_length_type, (std::string))
			)
			(optional
				(ar, *, 0.6)
				//(volume_fraction, *, 1)
			) )
		{
			mylog( "convertVolumeLength\n"
				<< "\tin_length_value: " << in_length_value
				<< "\n\tin_length_type: " << in_length_type
				<< "\n\tout_length_type: " << out_length_type
				<< "\n\tar: " << ar << std::endl);
			bool inIsV = false, outIsV = false;
			auto check = [](const std::string &name, const std::string &match, bool &out) {
				if (name == match) out = true; };
			check(in_length_type, "Volume", inIsV);
			check(out_length_type, "Volume", outIsV);
			// If input value is a volume, convert back to a length
			double inMD = 0;
			const double pi = boost::math::constants::pi<double>();
			if (inIsV) {
				if (ar <= 1) inMD = pow(6.*in_length_value*ar/pi,1./3.);
				else inMD = pow(6.*in_length_value/(pi*ar*ar),1./3.);
			} else {
				inMD = convertLength( _in_length_value = in_length_value,
				_in_length_type = in_length_type,
				_ar = ar,
				_out_length_type = "Max_Dimension"
				);
			}

			// inMD is the Max_Dimension
			double out = 0;
			if (outIsV) {
				if (ar <= 1) out = pow(inMD,3.) * pi / (6.*ar);
				else out = pow(inMD,3.) * pi * pow(ar,2.) / 6;
			} else {
				out = convertLength( _in_length_value = inMD,
					_in_length_type = "Max_Dimension",
					_ar = ar,
					_out_length_type = out_length_type
					);
			}
			mylog("\tresult is " << out);
			return out;
		}

		/** \brief Provides effective densities for use in particle modeling.
		 *
		 * Takes input in terms of the particle dimension (max dimension, ice effective radius,
		 * etc.), temperature and substance (ice).
		 *
		 * \returns Effective density in g/cm^3.
		 **/
//#define standardDProvider(name, needsDtype, needsUnits, needsSubstance)
	BOOST_PARAMETER_FUNCTION(
		(double),
			effDen,
			tag,
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
				mylog("convertIterate called");
				double vfa = backConvert(0.0001), vfb = backConvert(1.0);
				mylog("convertIterate initial bounds a: vf 0.0001: " << vfa << ", b: vf 1.0: " << vfb);
				if (vfa > 1.0 || vfb > 1.0) {
					mylog("The particle size chosen is too small for this size-density relation! Cannot proceed.");
					return 0;
				}
				double vf = zeros::findzero(0.0001, 1.0, [&](double guess) {
					return guess - backConvert(guess); });
				mylog("convertIterate returned a volume fraction of " << vf);
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
		ArgumentPack newArgs = (args, _provider = name); \
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


