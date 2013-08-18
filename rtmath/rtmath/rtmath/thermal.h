#pragma once
#include "defs.h"

namespace rtmath
{
	/** 
	* \brief Provides blackbody stuff and thermal emission information
	*
	* For reference:
	* W = J/s = N.m/s = kg.m^2/s^3
	**/
	namespace thermal
	{
		/// Speed of light (m/s)
		extern DLEXPORT_rtmath_core const double c;
		/// Planck constant (J.s)
		extern DLEXPORT_rtmath_core const double h;
		/// Stefan-Boltzmann Constant (J/K)
		extern DLEXPORT_rtmath_core const double kb;

		/**
		* \brief Calculates the blackbody spectral radiance, as a function of 
		* temperature (K) and frequency (GHz).
		*
		* Spectral radiance has units of W.s/m^2 = kg/s^2 .
		**/
		double DLEXPORT_rtmath_core radiancePlanck(double T, double f);

		/** 
		* \brief Calculate Rayleigh-Jeans approximation of spectral radiance, 
		* as a funciton of temperature (K) and frequency (GHz).
		*
		* Spectral radiance has units of W.s/m^2 = kg/s^2 .
		**/
		double DLEXPORT_rtmath_core radianceRJ(double T, double f);
	}
}
