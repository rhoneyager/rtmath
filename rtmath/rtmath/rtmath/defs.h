#pragma once
/* defs.h - Header containing typedefs and other defines for many things */
#include <cstdint>
#include <cmath>

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

namespace rtmath {

	/// Used for hashing
	typedef struct _UINT128 {
		uint64_t lower;
		uint64_t upper;
	} UINT128, 
		*PUINT128;

//#define LARGEHASH

	/// Hash type definitions based on architecture
#if (defined(_M_X64) || defined(__amd64)) && defined(LARGEHASH)
	typedef UINT128 HASH_t;
#else
	typedef uint32_t HASH_t;
#endif


	/// Hash function definitions based on architecture
	/// Both MurmurHash3 functions take the same arguments in the same order
	/// void MurmurHash3_...  ( const void * key, int len, uint32_t seed, void * out );
#if (defined(_M_X64) || defined(__amd64)) && defined(LARGEHASH)
#define HASH MurmurHash3_x64_128
#else
#define HASH MurmurHash3_x86_32
#endif

#define HASHSEED 2387213

#if (defined(_M_X64) || defined(__amd64)) && defined(LARGEHASH)
	struct hashcomp
	{
		bool operator() (const HASH_t &lhs, const HASH_t &rhs) const
		{
			//if ( (lhs.upper + lhs.lower) > (rhs.upper + rhs.lower) ) return true; // another way of comparing
			if (lhs.upper > rhs.upper) return true;
			if (lhs.upper < rhs.upper) return false;
			// If the upper fields are equal, move on to the next comparison
			if (lhs.lower > rhs.lower) return true;
			return false;
		}
	};
#else
	struct hashcomp
	{
		bool operator() (const HASH_t &lhs, const HASH_t &rhs) const
		{
			if (lhs > rhs) return true;
			if (rhs < lhs) return false;
			return false;
		}
	};
#endif

}

