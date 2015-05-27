#pragma once
#define _CRT_SECURE_NO_WARNINGS
#pragma warning( disable : 4996 )
#include <iostream>
#include <sstream>
#include <cstdint>
#include <cstring>
#include <boost/version.hpp>
#include "defs.h"
#include "cmake-settings.h"

#define QUOTE(str) #str
#define EXPAND_AND_QUOTE(str) QUOTE(str)

namespace rtmath
{
	namespace versioning {
		struct versionInfo {
			enum nums {
				V_MAJOR, V_MINOR, V_REVISION, V_SVNREVISION, V_MAX_INTS
			};
			
			uint64_t vn[V_MAX_INTS];
			
			static const uint64_t charmax = 256;
			char vdate[charmax];
			char vtime[charmax];
			char vsdate[charmax];
			char vssource[charmax];
			char vsuuid[charmax];
			char vassembly[charmax];
		};

		enum ver_match {
			INCOMPATIBLE, COMPATIBLE_1, COMPATIBLE_2, COMPATIBLE_3, EXACT_MATCH
		};

		ver_match DLEXPORT_rtmath_core compareVersions(const versionInfo &a, const versionInfo &b);

		/// Internal Ryan_Debug version
		void DLEXPORT_rtmath_core getLibVersionInfo(versionInfo &out);

		/// Calculates Ryan_Debug version string based on compile-time version of external code.
		inline void genVersionInfo(versionInfo &out)
		{
			for (size_t i = 0; i < versionInfo::V_MAX_INTS; ++i) out.vn[i] = 0;
			out.vsdate[0] = '\0'; out.vssource[0] = '\0'; out.vsuuid[0] = '\0';
			out.vassembly[0] = '\0';
			std::strncpy(out.vdate, __DATE__, versionInfo::charmax);
			std::strncpy(out.vtime, __TIME__, versionInfo::charmax);

			out.vn[versionInfo::V_MAJOR] = RTMATH_MAJOR;
			out.vn[versionInfo::V_MINOR] = RTMATH_MINOR;
			out.vn[versionInfo::V_REVISION] = RTMATH_REVISION;


#ifdef RTMATH_ASSEMBLY_NAME
			std::strncpy(out.vassembly, RTMATH_ASSEMBLY_NAME, versionInfo::charmax);
#endif
#ifdef RTMATH_SVNREVISION
			out.vn[versionInfo::V_SVNREVISION] = RTMATH_SVNREVISION;
#endif
#ifdef SUB_DATE
			std::strncpy(out.vsdate, SUB_DATE, versionInfo::charmax);
#endif
#ifdef SUB_SOURCE
			std::strncpy(out.vssource, SUB_SOURCE, versionInfo::charmax);
#endif
#ifdef SUB_UUID
			std::strncpy(out.vsuuid, SUB_UUID, versionInfo::charmax);
#endif
		}

		/**
		* \brief Provides information about the build environment during compilation.
		*
		* This function is designed to provide information on a compiler's
		* build environment. It is a header function because it is designed
		* to reflect the compiler variables of an external project's code.
		*
		* @param out The output stream that receives the information.
		*/
		inline void debug_preamble(const versionInfo &v, std::ostream &out = std::cerr)
		{
			out << "Compiled on " << v.vdate << " at " << v.vtime << std::endl;
			out << "Version " << v.vn[versionInfo::V_MAJOR] << "." << v.vn[versionInfo::V_MINOR]
				<< "." << v.vn[versionInfo::V_REVISION] << std::endl;
			if (v.vn[versionInfo::V_SVNREVISION]) out << "SVN Revision " << v.vn[versionInfo::V_SVNREVISION] << std::endl;
			if (v.vsdate[0] != '\0') out << "SVN Revision Date: " << v.vsdate << std::endl;
			if (v.vssource[0] != '\0') out << "SVN Source: " << v.vssource << std::endl;
			if (v.vsuuid[0] != '\0') out << "SVN UUID: " << v.vsuuid << std::endl;
			if (v.vassembly[0] != '\0') out << "Assembly: " << v.vassembly << std::endl;
			out << std::endl;
			out << std::endl;
		}
		inline void debug_preamble(std::ostream &out = std::cerr)
		{
			versionInfo v;
			genVersionInfo(v);
			debug_preamble(v, out);
		}
	}
}

