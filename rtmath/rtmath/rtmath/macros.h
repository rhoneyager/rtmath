#pragma once
#include "defs.h"

#include <boost/lexical_cast.hpp>

namespace rtmath {
	/** \brief Macro definitions to speed things up
	* 
	* MSVC 2010 and 2012 runtimes of rtmath can be slow. File reads are hampered 
	* by slow atof calls, and slow parsing of input lines.
	*
	* Macros here redirect to the standard function or a custom inlined varient, 
	* depending on the compiler.
	**/
	namespace macros {
		/// Convert argument to float
		double DLEXPORT_rtmath_core m_atof(const char* x, size_t len = 0);

		/// Convert argument to int
		int DLEXPORT_rtmath_core m_atoi(const char *x, size_t len = 0);


#ifdef _MSC_FULL_VER
#define M_ATOF(x) rtmath::macros::m_atof(x)
#define M_ATOI(x) rtmath::macros::m_atoi(x)
#else
#define M_ATOF(x) atof(x)
#define M_ATOI(x) atoi(x)
#endif


		/**
		* \brief Find sqrt(a^2+b^2) without overflow or underflow
		* 
		* Borrowed from Ahlquist for use in zero-finding algorithm.
		* Based on slatec.
		**/
		//double DLEXPORT_rtmath_core pythag(double a, double b);

		template <class T> T fastCast(const std::string &ss)
		{
			return boost::lexical_cast<T>(ss);
		}
		template<> double fastCast(const std::string &ss)
		{
			return M_ATOF(ss.data());
		}
		template<> float fastCast(const std::string &ss)
		{
			return static_cast<float>(M_ATOF(ss.data()));
		}
		template<> size_t fastCast(const std::string &ss)
		{
			return static_cast<size_t>(M_ATOI(ss.data()));
		}
		template<> int fastCast(const std::string &ss)
		{
			return M_ATOI(ss.data());
		}

	}

}

