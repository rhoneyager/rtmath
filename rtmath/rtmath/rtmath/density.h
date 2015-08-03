#pragma once
#include "defs.h"
#include <complex>
#include <functional>
#include <string>
#include <vector>
#include <boost/parameter/keyword.hpp>
#include <boost/parameter/name.hpp>
#include <boost/parameter/preprocessor.hpp>
#include <boost/lexical_cast.hpp>
//#include <boost/math/constants/constants.hpp>
#include "../rtmath/units.h"
#include <Ryan_Debug/error.h>

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
			double inDen = in_density;
			double outDen = out_density;
			double inV = in_volume;
			double inAeff = in_aeff;
			bool doAeff = (inAeff > 0) ? true : false;
			if (!inV && !inAeff) RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::otherErrorText("Function requires either an input volume or "
						"effective radius to perform the conversion.");
			double out = 0;
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
			auto findDen = [&temperature, &temp_units](double &den, const std::string &subst) {
				if (den) return;
				if (!temperature) RDthrow(Ryan_Debug::error::xBadInput())
					<< Ryan_Debug::error::otherErrorText("This density conversion requires a temperature.");
				if (subst == "ice1h") {
					den = ice1h( _temperature = temperature, _temp_units = temp_units );
				} else if (subst == "water") {
					den = water( _temperature = temperature, _temp_units = temp_units );
				} else if (subst == "SuperWater") {
					den = SuperWater( _temperature = temperature, _temp_units = temp_units );
				} else RDthrow(Ryan_Debug::error::xBadInput())
					<< Ryan_Debug::error::otherErrorText("Unknown substance for density conversion.");
				return;
			};
			findDen(inDen, in_substance);
			findDen(outDen, out_substance);

			// Once the densities are determined, then do the conversion of the volumes.
			// den = m / v.
			double mass = inDen * inV;
			out = mass / outDen;

			if (doAeff) { // convert back to effective radius if requested
				out = pow(3.*out/(4.),1./3.);
			}
			return out;
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
			return outVal;
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
			const size_t numProviders = 7;
			const char *providers[numProviders * 4] = { 
				"BrownFrancis1995Hogan2012", "Max_Diameter", "m", "ice",
				"Brandes2007", "Median_Volume_Diameter", "mm", "ice",
				"MagonoNakamura1965", "Equivalent_Diameter", "mm", "water",
				"Holroyd1971", "Equivalent_Diameter", "mm", "ice",
				"Muramoto1995", "Equivalent_Diameter", "mm", "ice",
				"FabrySzyrmer1999", "Equivalent_Diameter", "mm", "ice",
				"Heymsfield2004", "Equivalent_Diameter", "mm", "ice" };
			std::function<double(double)> funcs[] = {
				&(implementations::BrownFrancis1995Hogan2012),
				&(implementations::Brandes2007),
				&(implementations::MagonoNakamura1965),
				&(implementations::Holroyd1971),
				&(implementations::Muramoto1995),
				&(implementations::FabrySzyrmer1999),
				&(implementations::Heymsfield2004) };
			// Find provider and set values
			bool found = false;
			std::string needsDtype, needsUnits, needsSubstance;
			std::function<double(double)> func;
			for( size_t i = 0; i < numProviders; ++i) {
				if (std::string(providers[i*4]) != provider) continue;
				found = true;
				needsDtype = std::string(providers[(i*4)+1]);
				needsUnits = std::string(providers[(i*4)+2]);
				needsSubstance = std::string(providers[(i*4)+3]);
				func = funcs[i];
			}
			if (!found) RDthrow(Ryan_Debug::error::xBadInput())
				<< Ryan_Debug::error::otherErrorText("Unknown density provider");

			double len = 0;
			len = convertLength( _in_length_value = in_length_value,
				_in_length_type = in_length_type,
				_in_length_units = in_length_units,
				_ar = ar,
				_out_length_units = "um",
				_out_length_type = "EffectiveRadius"
				);
			len = convertSubstanceDensity( _in_aeff = len,
				_in_substance = "ice",
				_temperature = temperature,
				_temp_units = temp_units,
				_out_substance = needsSubstance
				);
			len = convertLength( _in_length_value = len,
				_in_length_type = "EffectiveRadius",
				_in_length_units = "um",
				_out_length_units = needsUnits,
				_ar = ar,
				_out_length_type = needsDtype
				);
			return (func)(len);
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


