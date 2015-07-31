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
		/// \todo Implement this.
		/// \todo Set a good pragma deprecated warning in msvc.
			double DLEXPORT_rtmath_core ERR_UNIMPLEMENTED water(double Tk);

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

#define standardDProvider(name, needsDtype, needsUnits) \
	BOOST_PARAMETER_FUNCTION( \
		(double), \
			name, \
			tag, \
			(required \
			(max_dimension, (double)) \
			) \
			(optional \
			(in_length_units, *, std::string("m")) \
			 (ar, *, 0.6) \
			 ) \
			) \
		{ \
			double len = 0; \
			len = convertLength( _in_length_value = 0, \
				_in_length_units = in_length_units, \
				_out_length_units = needsUnits, \
				_ar = ar \
				); \
			return implementations:: name(len); \
		}

		// Mass-density relstionships for ice crystals
		standardDProvider(BrownFrancis1995Hogan2012, "Max_Diameter", "m");
		standardDProvider(Brandes2007, "Median_Volume_Diameter", "mm"); // Code logic does not need to change.
		standardDProvider(MagonoNakamura1965, "Equivalent_Diameter", "mm");
		standardDProvider(Holroyd1971, "Equivalent_Diameter", "mm");
		standardDProvider(Muramoto1995, "Equivalent_Diameter", "mm");
		standardDProvider(FabrySzyrmer1999, "Equivalent_Diameter", "mm");
		standardDProvider(Heymsfield2004, "Equivalent_Diameter", "mm");
	}
}


