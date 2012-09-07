#include "../rtmath/Stdafx.h"
#include <cmath>
#include <map>
#include "../rtmath/density.h"
#include "../rtmath/units.h"
#include "../rtmath/error/error.h"

/* This file contains the tables and functions for the density of ice-1h
 * and supercooled water */

// Initialize the tables only once in a private namespace to improve function call times
namespace
{
	std::map<double, double> denIce, denSuperWater, denWater;

	void initSuperWater()
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

	void initIce()
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

	double interpMap(double target, const std::map<double,double> &m)
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
		double ice1h(double T)
		{
			initIce();
			// temp is in K, but convert to Celsius for convenience
			units::conv_temp c("K","C");
			double Tc = c.convert(T);
			if (Tc > 0 || Tc < -260) throw rtmath::debug::xModelOutOfRange(T);

			return interpMap(Tc, denIce);
		}

		double SuperWater(double T)
		{
			initSuperWater();
			units::conv_temp c("K","C");
			double Tc = c.convert(T);
			if (Tc > 0 || Tc < -30) throw rtmath::debug::xModelOutOfRange(T);

			return interpMap(Tc, denSuperWater);
		}

		double water(double T)
		{
			throw rtmath::debug::xUnimplementedFunction();
			return 0;
		}
	}
}

