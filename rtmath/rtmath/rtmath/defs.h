#pragma once
/* defs.h - Header containing typedefs and other defines for many things */
#include <cstdint>
#include <cmath>

namespace rtmath {

	typedef struct _UINT128 {
		uint64_t lower;
		uint64_t upper;
	} UINT128, 
		*PUINT128;

	// Hash type definitions based on architecture
#if defined(_M_X64) || defined(__amd64)
	typedef UINT128 HASH_t;
#else
	typedef uint32_t HASH_t;
#endif


	// Hash function definitions based on architecture
	// Both MurmurHash3 functions take the same arguments in the same order
	//void MurmurHash3_...  ( const void * key, int len, uint32_t seed, void * out );
#if defined(_M_X64) || defined(__amd64)
#define HASH MurmurHash3_x64_128
#else
#define HASH MurmurHash3_x86_32
#endif

#define HASHSEED 2387213

#if defined(_M_X64) || defined(__amd64)
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

	// Replacement error function
	// TODO: replace with function provided by boost libraries
	inline double erf(double x)
	{
		const double p = 0.3275911;
		const double a[] = {
			0.254829592, -0.284496736, 1.421413741,
			-1.453152027, 1.061405429 };
		double t = 1.0 / (1.0 + (p * x) );
		size_t i=0;
		double tp = 1;
		double res = 0;
		for (i=0;i<5;i++)
		{
			tp *= t;
			res += a[i] * tp;
		}
		res *= -1.0 * exp(-1.0 * x * x);
		res += 1;
		return res;
	}

}; // end namespace rtmath

