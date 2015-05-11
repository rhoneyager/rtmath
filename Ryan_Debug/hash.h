#pragma once
#include "defs.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "macros.h"
#include "registry.h"
#include <boost/filesystem.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

//namespace boost {
//namespace filesystem { class path; } 
//namespace program_options { class variables_map; } 
//}

namespace Ryan_Debug {
	namespace hash {
		BOOST_LOG_INLINE_GLOBAL_LOGGER_CTOR_ARGS(
			m_hash,
			blog::sources::severity_channel_logger_mt< >,
			(blog::keywords::severity = Ryan_Debug::log::error)(blog::keywords::channel = "hash"));

		class hashStore;
		typedef std::shared_ptr<const hashStore> pHashStore;
		class hash_provider_registry{};
		/// Designed to be a singleton
		class RYAN_DEBUG_DLEXPORT Hash_registry_provider
		{
		public:
			Hash_registry_provider();
			~Hash_registry_provider();
			//typedef std::function<boost::shared_ptr<VoronoiDiagram>
			//	(const Eigen::Array3f &, const Eigen::Array3f &,
			//	const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>&)> voronoiStdGeneratorType;
			//voronoiStdGeneratorType generator;
			const char* name;
		};
	}
	namespace registry {
		extern template class usesDLLregistry <
			::Ryan_Debug::hash::hash_provider_registry,
			::Ryan_Debug::hash::Hash_registry_provider > ;
	}
	namespace hash {

		/// Used for hashing
		class RYAN_DEBUG_DLEXPORT UINT128 {
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
		};

		typedef UINT128 HASH_t;
#define HASHSEED 2387213

		/// Wrapper function that calculates the hash of an object (key) with length (len).
		HASH_t RYAN_DEBUG_DLEXPORT HASH(const void *key, int len);

		/// Wrapper function to read and hash a file (handles compression, too!)
		HASH_t RYAN_DEBUG_DLEXPORT HASHfile(const std::string& filename);



		class RYAN_DEBUG_DLEXPORT hashStore :
			virtual public ::Ryan_Debug::registry::usesDLLregistry <
			hash_provider_registry, Hash_registry_provider >
		{
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
			/// The type of the hash store (dir, hdf5, ...)
			std::string type;
		public:
			virtual ~hashStore();

		};
	}
}
