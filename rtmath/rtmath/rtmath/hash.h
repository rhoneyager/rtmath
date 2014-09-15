#pragma once
#include "defs.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#if USE_RYAN_SERIALIZATION
#include <boost/serialization/access.hpp>
#include <boost/serialization/export.hpp>
#endif

#include "macros.h"
#include "registry.h"
#include <boost/filesystem.hpp>

//namespace boost {
	//namespace filesystem { class path; } 
	//namespace program_options { class variables_map; } 
//}

namespace rtmath {

	/// Used for hashing
	class DLEXPORT_rtmath_core UINT128 {
	public:
		UINT128() : lower(0), upper(0) {}
		UINT128(uint64_t lower, uint64_t upper) : lower(lower), upper(upper) {}
		uint64_t lower;
		uint64_t upper;
		inline bool operator<(const UINT128 &rhs) const
		{
			if (upper != rhs.upper) return upper < rhs.upper;
			if (lower != rhs.lower) return lower < rhs.lower;
			return false;
		}
		inline bool operator==(const UINT128 &rhs) const
		{
			if (upper != rhs.upper) return false;
			if (lower != rhs.lower) return false;
			return true;
		}
		inline bool operator!=(const UINT128 &rhs) const
		{
			return !operator==(rhs);
		}
		inline UINT128 operator^(const UINT128 &rhs)
		{
			UINT128 res = *this;
			res.lower = res.lower ^ rhs.lower;
			res.upper = res.upper ^ rhs.upper;
			return res;
		}

		std::string string() const
		{
			std::ostringstream o;
			o << lower;
			std::string res = o.str();
			return res;
		}
#if USE_RYAN_SERIALIZATION
	private:
		friend class ::boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version);
#endif
	};

	typedef UINT128 HASH_t;
#define HASHSEED 2387213

	/// Wrapper function that calculates the hash of an object (key) with length (len).
	HASH_t DLEXPORT_rtmath_core HASH(const void *key, int len);

	/// Wrapper function to read and hash a file (handles compression, too!)
	HASH_t DLEXPORT_rtmath_core HASHfile(const std::string& filename);


	class hashStore;
	typedef std::shared_ptr<const hashStore> pHashStore;
	class DLEXPORT_rtmath_core hashStore {
	public:
		static bool findHashObj(const std::string &hash, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts);
		static bool storeHash(const std::string&, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts);
		static void addHashStore(pHashStore, size_t priority);
	public:
		virtual bool storeHashInStore(const std::string& hash, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts
			) const;
		virtual bool findHashInStore(const std::string &hash, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts) const;

	public:
		hashStore();
		/// Base location of the hash database
		boost::filesystem::path base;
		/// Indicates whether this store is read-only
		bool writable;
		
	public:
		~hashStore();
	
	};

}

#if USE_RYAN_SERIALIZATION
BOOST_CLASS_EXPORT_KEY(rtmath::UINT128);
#endif
std::ostream & operator<<(std::ostream &stream, const rtmath::UINT128 &ob);

