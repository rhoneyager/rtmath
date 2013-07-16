#pragma once
/* defs.h - Header containing typedefs and other defines for many things */
#include <cstdint>
#include <cmath>
#include <boost/serialization/export.hpp>

namespace rtmath {

	/// Used for hashing
	struct UINT128 {
		uint64_t lower;
		uint64_t upper;
	};

//#define LARGEHASH

	/// Hash type definitions based on architecture
//#if (defined(_M_X64) || defined(__amd64)) && defined(LARGEHASH)
	typedef UINT128 HASH_t;
//#else
//	typedef uint32_t HASH_t;
//#endif

	/// Hash function definitions based on architecture
	/// Both MurmurHash3 functions take the same arguments in the same order
	/// void MurmurHash3_...  ( const void * key, int len, uint32_t seed, void * out );
//#if (defined(_M_X64) || defined(__amd64)) && defined(LARGEHASH)
//#define HASH MurmurHash3_x64_128
//#else
//#define HASH MurmurHash3_x86_32
//#endif

#define HASHSEED 2387213

HASH_t HASH(const void *key, int len);

//#if (defined(_M_X64) || defined(__amd64)) && defined(LARGEHASH)
	struct hashcomp
	{
		bool operator() (const HASH_t &lhs, const HASH_t &rhs) const
		{
			//if ( (lhs.upper + lhs.lower) > (rhs.upper + rhs.lower) ) return true; // another way of comparing
			if (lhs.upper != rhs.upper) return lhs.upper < rhs.upper;
			if (lhs.lower != rhs.lower) return lhs.lower < rhs.lower;
			return false;
		}
	};
/*
#else
	struct hashcomp
	{
		bool operator() (const HASH_t &lhs, const HASH_t &rhs) const
		{
			if (lhs != rhs) return lhs < rhs;
			return false;
		}
	};
#endif
*/
}

namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive&, rtmath::UINT128&, const unsigned int);
	}
}

BOOST_CLASS_EXPORT_KEY(rtmath::UINT128);
