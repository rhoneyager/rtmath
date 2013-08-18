#pragma once
#include "defs.h"

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
	}
}


