#pragma once
#include "defs.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
//#include <boost/filesystem.hpp>

namespace boost {namespace filesystem { class path; } }

namespace rtmath {

	/// Used for hashing
	class DLEXPORT_rtmath_core UINT128 {
	public:
		UINT128() : lower(0), upper(0) {}
		uint64_t lower;
		uint64_t upper;
		inline bool operator<(const UINT128 &rhs) const
		{
			if (upper != rhs.upper) return upper < rhs.upper;
			if (lower != rhs.lower) return lower < rhs.lower;
			return false;
		}
		std::string string() const
		{
			std::ostringstream o;
			o << lower;
			std::string res = o.str();
			return res;
		}
	private:
		friend class ::boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);
	};

	typedef UINT128 HASH_t;
#define HASHSEED 2387213

	/// Wrapper function that calculates the hash of an object (key) with length (len).
	HASH_t DLEXPORT_rtmath_core HASH(const void *key, int len);

	/// Wrapper function to read and hash a file (handles compression, too!)
	HASH_t DLEXPORT_rtmath_core HASHfile(const std::string& filename);


	struct hashStore;
	typedef boost::shared_ptr<const hashStore> pHashStore;
	typedef std::vector<pHashStore> hashStores;
	struct DLEXPORT_rtmath_core hashStore {
		bool writable;
		std::string objtype;
		boost::filesystem::path base;
		bool local;
		bool localCache;
		/// Extensions that get matched when searching for existing files
		std::vector<std::string> extensions;
		/// Add compressed files to search path
		bool compressible;
		enum class default_hashstores
		{
			SHAPEFILE,
			SHAPESTATS,
			VORONOI
		};
		static hashStores findHashStore(default_hashstores);
		static hashStores findHashStore(const std::string &objtype);
		static boost::filesystem::path findHash(const hashStores&, const HASH_t&);
		static boost::filesystem::path findHash(const hashStores&, const std::string&);
		static boost::filesystem::path storeHash(const hashStores&, const HASH_t&);
		static boost::filesystem::path storeHash(const hashStores&, const std::string&);

		boost::filesystem::path storeHash(const HASH_t&);
		boost::filesystem::path storeHash(const std::string&);

		/// \brief Function to find a hash in a directory hash structure.
		/// Does not modify hash tree structure.
		bool findHash(const HASH_t&, boost::filesystem::path&);
		bool findHash(const std::string&, boost::filesystem::path&);
		~hashStore();
	private:
		hashStore();
		hashStore(const boost::filesystem::path&, const std::string &objtype, bool writable, bool local);
	};

}

BOOST_CLASS_EXPORT_KEY(rtmath::UINT128);


