#pragma once

namespace rtmath
{
	namespace thermal
	{
		// For reference:
		// W = J/s = N.m/s = kg.m^2/s^3

		// Calculates the blackbody spectral radiance, as a function of 
		// temperature (K) and frequency (GHz).
		// Spectral radiance has units of W.s/m^2 = kg/s^2
		double radiancePlanck(double T, double f);

		// Calculate Rayleigh-Jeans approximation of spectral radiance, 
		// as a funciton of temperature (K) and frequency (GHz).
		// Spectral radiance has units of W.s/m^2 = kg/s^2
		double radianceRJ(double T, double f);
	}
}
