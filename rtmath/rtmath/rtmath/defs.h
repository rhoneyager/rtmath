#pragma once
/* defs.h - Header containing typedefs and other defines for many things */

/// \todo CMakeLists needs to set the appropriate flags EXPORTING_RTMATH on all libraries and 
/// SHARED_RTMATH_* for the libraries that are shared.

/// \def EXPORTING_RTMATH is set in each rtmath library. Absence of flag indicates possible import of shared code.

/// \def SHARED_RTMATH is set when building using shared libraries. 
/// \todo Split flag per-library.

/// \def DLEXPORT_RTMATH takes three values. dllexport and dllimport are Windows-only, used with shared libraries.
/// Otherwise, this tag is removed by the preprocessor.

#ifdef EXPORTING_RTMATH
	#ifdef _MSC_FULL_VER
		#ifdef SHARED_RTMATH
			#define DLEXPORT_RTMATH __declspec(dllexport)
		#else
			#define DLEXPORT_RTMATH
		#endif
	#else
		#define DLEXPORT_RTMATH
	#endif
#else
	#ifdef _MSC_FULL_VER
		#ifdef SHARED_RTMATH
			#define DLEXPORT_RTMATH __declspec(dllimport)
		#else
			#define DLEXPORT_RTMATH
		#endif
	#else
		#define DLEXPORT_RTMATH
	#endif
#endif

