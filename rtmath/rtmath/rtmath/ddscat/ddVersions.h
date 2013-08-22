#pragma once
#include "../defs.h"
#include <string>

namespace rtmath
{
	namespace ddscat
	{
		/// Provide ddscat versioning information and comparisons.
		namespace ddVersions
		{
			/// Parses a string to attempt to guess the DDSCAT version
			size_t DLEXPORT_rtmath_ddscat_base getVerId(const std::string&);

			// These functions each from a DDSCAT version string in 
			// slightly different ways

			/// Get ddscat version in form of x.x.x
			std::string DLEXPORT_rtmath_ddscat_base getVerString(size_t id); 
			/// Get header string in form of DDSCAT 7.0.7 [09.12.11]
			std::string DLEXPORT_rtmath_ddscat_base getVerAvgHeaderString(size_t id);


			/// Useful functions for checking whether a ddscat version falls 
			/// within the specified ranges
			bool DLEXPORT_rtmath_ddscat_base isVerWithin(size_t ver, const std::string &range);
			bool DLEXPORT_rtmath_ddscat_base isVerWithin(size_t ver, size_t min, size_t max);

			/// Get the default ddscat version (may be overridden by user in future)
			size_t DLEXPORT_rtmath_ddscat_base getDefaultVer();
		}
	}
}
