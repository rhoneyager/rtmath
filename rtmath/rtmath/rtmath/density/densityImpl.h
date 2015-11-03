#pragma once
#include "../defs.h"
#include <functional>
#include <string>
#include <Ryan_Debug/error.h>
#include <Ryan_Debug/logging_base.h>

namespace rtmath
{
	namespace density
	{
		namespace implementations {
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

			/// Linear density relation. Different units than in other cases.
			double DLEXPORT_rtmath_core linearDensity(double lowAeff, double lowVf,
				double highAeff, double highVf, double aeff);

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
	}
}

