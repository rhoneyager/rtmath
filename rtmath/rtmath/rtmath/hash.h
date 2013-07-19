#pragma once
/* defs.h - Header containing typedefs and other defines for many things */
#include <cstdint>
#include <cmath>
#include <iostream>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
//#include <boost/filesystem.hpp>

namespace boost {namespace filesystem { class path; } }

namespace rtmath {

	/// Used for hashing
	class UINT128 {
	public:
		uint64_t lower;
		uint64_t upper;
		inline bool operator<(const UINT128 &rhs) const
		{
			if (upper != rhs.upper) return upper < rhs.upper;
			if (lower != rhs.lower) return lower < rhs.lower;
			return false;
		}
	private:
		friend class ::boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);
	};

	typedef UINT128 HASH_t;
#define HASHSEED 2387213

	/// Wrapper function that calculates the hash of an object (key) with length (len).
	HASH_t HASH(const void *key, int len);

	/// \brief Function to find a hash in a directory hash structure.
	/// Does not modify hash tree structure.
	boost::filesystem::path findHash(const boost::filesystem::path &base, const HASH_t &hash);
	/// \brief Function to determine where a new hash should be stored in a hash path.
	/// May create new subhash folders.
	boost::filesystem::path storeHash(const boost::filesystem::path &base, const HASH_t &hash);

}

BOOST_CLASS_EXPORT_KEY(rtmath::UINT128);


