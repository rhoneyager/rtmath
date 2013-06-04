#pragma once
#include <string>

namespace rtmath
{
	namespace ddscat
	{
		namespace ddVersions
		{
			// Parses a string to attempt to guess the DDSCAT version
			size_t getVerId(const std::string&);

			// These functions each from a DDSCAT version string in 
			// slightly different ways
			std::string getVerString(size_t id); // Just x.x.x
			std::string getVerAvgHeaderString(size_t id); // DDSCAT 7.0.7 [09.12.11]


			// Useful functions for checking whether a ddscat version falls 
			// within the specified ranges
			bool isVerWithin(size_t ver, const std::string &range);
			bool isVerWithin(size_t ver, size_t min, size_t max);

			// Get the default ddscat version (may be overridden by user in future)
			size_t getDefaultVer();
		}
	}
}
