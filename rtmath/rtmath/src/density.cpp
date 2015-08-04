#include "Stdafx-core.h"
#include <cmath>
#include <map>
#include "../rtmath/density.h"
#include "../rtmath/units.h"
#include <Ryan_Debug/error.h>

/* This file contains the tables and functions for the density of ice-1h
 * and supercooled water */

/// Initialize the tables only once in a private namespace to improve function call times
namespace
{
	std::map<double, double> denIce, denSuperWater, denWater;

	void SHARED_PRIVATE initSuperWater()
	{
		static bool inited = false;
		if (inited) return;

		denSuperWater[0] = 0.9998;
		denSuperWater[-5] = 0.9993;
		denSuperWater[-10] = 0.9981;
		denSuperWater[-15] = 0.9963;
		denSuperWater[-20] = 0.9936;
		denSuperWater[-25] = 0.9896;
		denSuperWater[-30] = 0.9838;

		inited = true;
	}

	void SHARED_PRIVATE initIce()
	{
		static bool inited = false;
		if (inited) return;

		denIce[0] = 0.9167;
		denIce[-10] = 0.9182;
		denIce[-20] = 0.9196;
		denIce[-30] = 0.9209;
		denIce[-40] = 0.9222;
		denIce[-50] = 0.9235;
		denIce[-60] = 0.9247;
		denIce[-80] = 0.9269;
		denIce[-100] = 0.9288;
		denIce[-120] = 0.9304;
		denIce[-140] = 0.9317;
		denIce[-160] = 0.9326;
		denIce[-180] = 0.9332;
		denIce[-200] = 0.9336;
		denIce[-220] = 0.9337;
		denIce[-240] = 0.9338;
		denIce[-260] = 0.9338;

		inited = true;
	}

	double SHARED_PRIVATE interpMap(double target, const std::map<double,double> &m)
	{
		auto lower = m.lower_bound(target);
		if (lower->first == target) return lower->second;
		lower--;

		auto upper = m.upper_bound(target); // Will always point to element 'greater'

		// Linear interpolation
		double span = upper->first - lower->first;
		double xs = target - lower->first;
		double range = upper->second - lower->second;
		double res = lower->second + (xs*range/span);
		return res;

	}
}

namespace rtmath
{
	namespace density
	{
		namespace implementations {
			double ice1h(double T)
			{
				initIce();
				// temp is in K, but convert to Celsius for convenience
				units::conv_temp c("K","C");
				double Tc = c.convert(T);
				if (Tc > 0 || Tc < -260) RDthrow(Ryan_Debug::error::xModelOutOfRange())
					<< Ryan_Debug::error::temp_ref_range(std::pair<double, double>(-260, 0))
					<< Ryan_Debug::error::temp(Tc)
					<< Ryan_Debug::error::otherErrorText("Temp. out of range. Function "
						"needs input temp in Kelvin. Listed ranges "
						"in error are in Celsius.");

				return interpMap(Tc, denIce);
			}

			double SuperWater(double T)
			{
				initSuperWater();
				units::conv_temp c("K","C");
				double Tc = c.convert(T);
				if (Tc > 0 || Tc < -30) RDthrow(Ryan_Debug::error::xModelOutOfRange())
					<< Ryan_Debug::error::temp_ref_range(std::pair<double, double>(-30, 0))
					<< Ryan_Debug::error::temp(Tc)
					<< Ryan_Debug::error::otherErrorText("Temp. out of range. Function "
						"needs input temp in Kelvin. Listed ranges "
						"in error are in Celsius.");

				return interpMap(Tc, denSuperWater);
			}

			double water(double T)
			{
				if (T < 273.15) { 
					return SuperWater(T);
				}
				RDthrow(Ryan_Debug::error::xUnimplementedFunction());
				return 0;
			}

			double BrownFrancis1995Hogan2012(double D) {
				double mass = 0.012 * pow(D,1.9); // in kg
				//const double rho0 = 917.0; // kg m^-3
				//const double V = mass / rho0;
				return mass * 1000.; // output in g
			}

			double Brandes2007(double D) {
				return 0.178 * pow(D,-0.922); // Context of D is in the calling function.
			}

			double MagonoNakamura1965(double D) {
				return 2. * pow(D,-2.);
			}

			double Holroyd1971(double D) {
				return 0.17 * pow(D,-1.);
			}

			double Muramoto1995(double D) {
				return 0.048 * pow(D,-0.406);
			}

			double FabrySzyrmer1999(double D) {
				return 0.15 / D;
			}

			double Heymsfield2004(double D) {
				return 0.104 * pow(D,-0.95);
			}

			void findDen(double &den, const std::string &subst,
					double temperature, const std::string &temp_units)
			{
				if (den) return;
				if (!temperature) RDthrow(Ryan_Debug::error::xBadInput())
					<< Ryan_Debug::error::otherErrorText("This density conversion requires a temperature.");
				if (subst == "ice1h" || subst == "ice") {
					den = ::rtmath::density::ice1h( _temperature = temperature, _temp_units = temp_units );
				} else if (subst == "water") {
					den = ::rtmath::density::water( _temperature = temperature, _temp_units = temp_units );
				} else if (subst == "SuperWater") {
					den = ::rtmath::density::SuperWater( _temperature = temperature, _temp_units = temp_units );
				} else RDthrow(Ryan_Debug::error::xBadInput())
					<< Ryan_Debug::error::specializer_type(subst)
					<< Ryan_Debug::error::otherErrorText("Unknown substance for density conversion.");
				return;
			}
		}
	}
}

