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
#include "unitOptions.h"
#include "../units.h"
#include "../zeros.h"
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging_base.h>

#undef FL
#undef mylog
#define FL "convertLength.h, line " << (int)__LINE__ << ": "

#define mylog(x) { std::ostringstream l; l << FL << x; rtmath::units::implementations::emit_units_log(l.str()); }

namespace rtmath {
	namespace units {
		/** \brief Function to convert between different length measurements
		 *
		 * You provide the function with a type of length and a value.
		 * Ex: 100 um effective radius.
		 * The length type can be an effective radius, effective diameter,
		 * equivalent radius, max_diameter, min_diameter, max_radius,
		 * min_radius, etc.
		 *
		 * An effective radius is the radius of an equal-volume solid sphere.
		 * An equivalent radius is the same.
		 *
		 * \note Aspect ratio is respected. Unit conversions between the input and
		 * output length units (um, mm, m, etc.) are performed at the start.
		 *
		 * \param ar is aspect ratio, where ar < 1 is oblate and ar > 1 is prolate.
		 * \todo Add another flag to change this ar convention?
		 **/
		BOOST_PARAMETER_FUNCTION( (double),
			convertLength,
			::rtmath::units::keywords::tag,
			(required
				(in_length_value, (double))
				(in_length_type, (std::string))
				(out_length_type, (std::string))
			)
			(optional
				(ar, *, 1)
				(in_volume_fraction, *, 1)
				(out_volume_fraction, *, 1)
				// unit conversion dropped since length and volume are both used
				//(in_length_units, *, std::string("um"))
				//(out_length_units, *, std::string("um"))
			) )
		{
			mylog( "Converting length.\n"
				<< "\tFrom: " << in_length_value << " in " << in_length_type
				<< "\n\tto: " << out_length_type
				<< "\n\tvf in: " << in_volume_fraction << " vf out: " << out_volume_fraction
				<< "\n\tar: " << ar);
			const double pi = boost::math::constants::pi<double>();
			// First convert to the output dimensional units
			double inVal = in_length_value; //rtmath::units::conv_alt(in_length_units, out_length_units).convert(in_length_value);

			// If input value is a radius, convert to a diameter by multiplying by 2.
			double inDiam = inVal;
			if (in_length_type.find("adius") != std::string::npos) inDiam *= 2.;

			// Next, pass the desired conversion formulas. The main conversions are between (radius,diameter)
			// effective volume units and max dimension units.

			bool inIsV = false, outIsV = false;
			bool inIsMD = false, outIsMD = false;
			bool inIsMax = false, outIsMax = false, junk = false;
			auto check = [](const std::string &name, const std::string &match, bool &out, bool &out2) {
				if (name.find(match) != std::string::npos) {
					//bool pout = out, pout2 = out2;
					out = true; out2 = true;
					//mylog("\n\tmatched " << match << " in " << name
					//	<< "\n\t prev out " << pout << ", prev out2 " << pout2);
				} };
			// Max_Dimension vs. Effective_Diameter vs. Min_Diameter
			check(in_length_type, "Max_Dimension", inIsMD, inIsMax);
			check(in_length_type, "Max_Diameter", inIsMD, inIsMax);
			check(in_length_type, "Max_Radius", inIsMD, inIsMax);
			check(in_length_type, "Min_Dimension", inIsMD, junk);
			check(in_length_type, "Min_Diameter", inIsMD, junk);
			check(in_length_type, "Min_Radius", inIsMD, junk);
			check(in_length_type, "Volume", inIsV, junk);
			check(out_length_type, "Max_Dimension", outIsMD, outIsMax);
			check(out_length_type, "Max_Diameter", outIsMD, outIsMax);
			check(out_length_type, "Max_Radius", outIsMD, outIsMax);
			check(out_length_type, "Min_Dimension", outIsMD, junk);
			check(out_length_type, "Min_Diameter", outIsMD, junk);
			check(out_length_type, "Min_Radius", outIsMD, junk);
			check(out_length_type, "Volume", outIsV, junk);

			double outVal = 0;
			double V = 0; // Volume
			double arnorm = ar; // arnorm from (0,1]. AR from (0,inf).
			bool prolate = false;
			if (arnorm > 1) { // Different conventions
				arnorm = 1./arnorm;
				prolate = true;
			}
			mylog("\n\tinIsMD " << inIsMD << "\n\tinIsMax " << inIsMax
				<< "\n\tinIsV " << inIsV << "\n\toutIsMD " << outIsMD
				<< "\n\toutIsMax " << outIsMax << "\n\toutIsV " << outIsV
				<< "\n\tprolate " << prolate);

			if(inIsV) {
				V = inVal;
			} else if (inIsMD) {
				// V = 4/3 pi a b c. Check if oblate or prolate.
				double inMaxDiam = inDiam;
				if (!inIsMax) {
					inMaxDiam = inDiam / arnorm;
				}
				double a = 0, b = 0, c = 0;
				if (!prolate) {
					a = inMaxDiam; b = inMaxDiam;
					c = inMaxDiam * arnorm;
				} else {
					a = inMaxDiam; b = inMaxDiam * arnorm; c = inMaxDiam * arnorm;
				}
				V = pi * a * b * c / (6.);
				mylog("\n\tinDiam " << inDiam << "\n\tinMaxDiam " << inMaxDiam);
			} else {
				// We have an effective radius. ar not needed.
				V = pi * pow(inDiam,3.) / (6.);
			}
			mylog("\n\tvolume before vf adjustment is " << V);
			if (inIsMD)
				V *= in_volume_fraction;
			if (outIsMD)
				V /= out_volume_fraction;
			// TODO: may need to still tweak V vf scaling.

			// Volume is known. Use this to get the output quantity
			// (as a diameter).
			if (outIsV) {
				outVal = V;
			} else if (outIsMD) {
				double outMax = 0;
				if (prolate)
					outMax = pow(6.*V/(pi*arnorm*arnorm),1./3.);
				else
					outMax = pow(6.*V/(pi*arnorm),1./3.);
				if (!outIsMax) {
					outVal = outMax * arnorm;
				} else outVal = outMax;
				mylog("\n\toutMax is " << outMax);
			} else { // An effective radius or diameter
				outVal = pow(6.*V/pi,1./3.);
			}
			if(!outIsV && out_length_type.find("adius") != std::string::npos) outVal /= 2.;
			mylog("\n\tvolume after adjustment is " << V << "\n\toutput is " << outVal
					<< " as " << out_length_type);
			return outVal;
		}

	}
}

