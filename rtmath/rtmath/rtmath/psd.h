#pragma once
/// Particle size distributions
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
	namespace psd
	{
		/// These functions are all PDFs, subject to further integration.
		namespace implementations {

#define implExp(x) namespace x { \
	double DLEXPORT_rtmath_core N0(double R); double DLEXPORT_rtmath_core lambda(double R); \
	double DLEXPORT_rtmath_core Nd(double R, double Dmelt); double DLEXPORT_rtmath_core NdI(double R); \
	double DLEXPORT_rtmath_core median(double R); }

			/// Sekhon and Srivastava (1970) ice crystal size distribution.
			implExp(SekhonSrivastava1970);
			/// Marshall and Palmer 1948. Size distribution for raindrops.
			implExp(MarshallPalmer1948);
			/// Gunn and Marshall 1958. Size distribution for aggregate snowflakes.
			implExp(GunnMarshall1958);
			/*
			/// \brief Framework for the exponential size distributions.
			template<typename ttag> struct expBase {
			private: expBase() {}
			public:
				/// N0 is in 
				inline static double N0(double R) { return tag:: ttag:: N0(R); }
				/// lambda is in 
				inline static double lambda(double R) { return tag:: ttag:: lambda(R); }
				/// Nd is in m^-3 mm^-1
				inline static double Nd(double R, double Dmelt) { return tag:: ttag:: Nd(R, Dmelt); }
				/// NdI is the integral, in m^-3
				inline static double NdI(double R) { return tag:: ttag:: NdI(R); }
				/// The size (in _) of the median particle in the distribution.
				inline static double median(double R) { return tag:: ttag:: median(R); }
			};

			typedef expBase<SekhonSrivastava1970> SekhonSrivastava1970;
			typedef expBase<MarshallPalmer1948> MarshallPalmer1948;
			typedef expBase<GunnMarshall1958> GunnMarshall1958;
*/

		}


	}
}

