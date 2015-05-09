#pragma once
#include "defs.h"

#include <cstdint>
#include <boost/lexical_cast.hpp>

namespace Ryan_Debug {
	/** \brief Macro definitions to speed things up
	* 
	* MSVC 2010 and 2012 runtimes of Ryan_Debug can be slow. File reads are hampered 
	* by slow atof calls, and slow parsing of input lines.
	*
	* Macros here redirect to the standard function or a custom inlined varient, 
	* depending on the compiler.
	**/
	namespace macros {
		/// Convert argument to float
		template <class T>
		T RYAN_DEBUG_DLEXPORT m_atof(const char* x, size_t len = 0);


		/// Convert argument to int
		template <class T>
		T RYAN_DEBUG_DLEXPORT m_atoi(const char *x, size_t len = 0);


#ifdef _MSC_FULL_VER
#define M_ATOF(x) Ryan_Debug::macros::m_atof<double>(x)
#define M_ATOI(x) Ryan_Debug::macros::m_atoi<int>(x)
#else
#define M_ATOF(x) atof(x)
#define M_ATOI(x) atoi(x)
#endif

		template <class T> T fastCast(const std::string &ss)
		{
			return boost::lexical_cast<T>(ss);
		}
		template<> RYAN_DEBUG_DLEXPORT double fastCast(const std::string &ss);
		template<> RYAN_DEBUG_DLEXPORT float fastCast(const std::string &ss);
		template<> RYAN_DEBUG_DLEXPORT size_t fastCast(const std::string &ss);
		template<> RYAN_DEBUG_DLEXPORT int fastCast(const std::string &ss);
		template<> RYAN_DEBUG_DLEXPORT uint64_t fastCast(const std::string &ss);
	}

}

