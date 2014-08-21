#include "Stdafx-core.h"
#include "../rtmath/defs.h"
#include "../rtmath/hash.h"
#include "../rtmath/Public_Domain/MurmurHash3.h"
#if USE_RYAN_SERIALIZATION
#include "../rtmath/Serialization/serialization_macros.h"
#include <Ryan_Serialization/serialization.h>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#endif
#include <mutex>
#include <map>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <string>
#include <boost/lexical_cast.hpp>
#include "../rtmath/config.h"
#include "../rtmath/Serialization/Serialization.h"
#include "../rtmath/error/error.h"

/// Handles private hash functions
namespace {
	std::mutex mHashStore;
	std::multimap<size_t, rtmath::pHashStore> stores;

	void loadStores()
	{
		static bool inited = false;
		// Already locked by calling function
		if (inited) return;
		inited = true;

		/// \todo Implement most of this function

		// Stores can be folder trees, single hdf5 files, and websites.
		// The store database can be stored in sql. 
		/// \todo Add entries in registry for store registration

		// Each entry in the store has a priority, reflecting its use order.
		// The highest-priority writable store is used for writing, and the first store 
		// having a given key is used when reading.

		// For now, just default to loading the rtmath.conf-listed store, if any.
		{
			auto conf = rtmath::config::loadRtconfRoot();
			std::string hashDir;
			auto chash = conf->getChild("ddscat")->getChild("hash");
			chash->getVal<std::string>("hashDir", hashDir);

			std::shared_ptr<rtmath::hashStore> h(new rtmath::hashStore);
			h->writable = true;
			h->base = boost::filesystem::path(hashDir);
			rtmath::hashStore::addHashStore(h, 100);
		}
	}
}

namespace rtmath {

	/*
	boost::filesystem::path findHash(const boost::filesystem::path &base, const std::string &hash,
		const std::vector<std::string> &extensions)
	{
		using namespace boost::filesystem;
		using boost::lexical_cast;
		using Ryan_Serialization::detect_compressed;
		using std::string;

		string sHashName = hash;
		string sHashStart = sHashName.substr(0,2);
		path pHashName(sHashName);
		path pHashStart(sHashStart);

		std::vector<std::string> ext = extensions;
		if (!ext.size()) ext.push_back("");
		for (auto e : ext)
		{
			// First, check the base directory
			path pBaseCand = base / pHashName;
			pBaseCand += e;

			if( detect_compressed(pBaseCand) ) return pBaseCand;

			// Then, check in subfolders corresponding to the first two digits of the hash
			path pSubCand = base / pHashStart / pHashName;
			pSubCand += e;
			if (detect_compressed(pSubCand)) return pSubCand;
		}

		
		return path("");
	}

	boost::filesystem::path storeHash(const boost::filesystem::path &base, const std::string &hash)
	{
		using namespace boost::filesystem;
		using boost::lexical_cast;
		using Ryan_Serialization::detect_compressed;
		using std::string;

		string sHashName = hash;
		string sHashStart = sHashName.substr(0,2);
		path pHashName(sHashName);
		path pHashStart(sHashStart);

		if (!exists(base))
			return path(""); // Silently fail
			//RTthrow rtmath::debug::xMissingFile(base.string().c_str());

		if (!exists(base / pHashStart))
			create_directory(base / pHashStart);

		return base/pHashStart/pHashName;
	}
	*/

	hashStore::hashStore() {}
	hashStore::~hashStore() {}

	bool hashStore::storeHashInStore(const std::string& h, const std::string &key, 
		std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts) const
	{
		opts = registry::IO_options::generate(registry::IOhandler::IOtype::TRUNCATE);
		sh = nullptr; // IOhandler is not needed for the basic filesystem store
		if (!writable) return false;

		using namespace boost::filesystem;
		using boost::lexical_cast;
		using serialization::detect_compressed;
		using std::string;

		const string sHashName = h;
		const string sHashStart = sHashName.substr(0, 2);
		path pHashName(sHashName);
		path pHashStart(sHashStart);

		
		if (!exists(base))
			RTthrow debug::xMissingFolder(base.string().c_str());
		
		if (!exists(base / pHashStart))
			create_directory(base / pHashStart);
		
		path p = base / pHashStart / pHashName;

		if (!exists(p))
			create_directory(p);

		opts->setVal<std::string>("folder", p.string());
		if (!key.size()) return true;

		path pf = p / path(key);
		string meth;
		path target;
		if (detect_compressed(pf, meth, target))
		{
			opts->filename(target.string());
			opts->setVal<string>("compression_method", meth);
			opts->setVal<string>("base_filename", pf.string());
		}
		else {
			opts->filename(pf.string());
			opts->setVal<string>("base_filename", pf.string());
			string meth;
			serialization::select_compression(pf.string(), meth);
			opts->setVal<string>("compression_method", meth);
		}
		return true;
	}

	bool hashStore::findHashInStore(const std::string &hash, const std::string &key,
		std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts) const
	{
		opts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
		sh = nullptr; // IOhandler is not needed for the basic filesystem store

		using namespace boost::filesystem;
		using boost::lexical_cast;
		using serialization::detect_compressed;
		using std::string;

		const string sHashName = hash;
		const string sHashStart = sHashName.substr(0, 2);
		path pHashName(sHashName);
		path pHashStart(sHashStart);


		if (!exists(base))
			return false;
		if (!exists(base / pHashStart))
			return false;
		path p = base / pHashStart / pHashName;
		if (!exists(p))
			return false;

		opts->setVal<std::string>("folder", p.string());
		if (!key.size()) return true;
		path pf = p / path(key);
		string meth;
		path target;
		if (detect_compressed(pf, meth, target))
		{
			opts->filename(target.string());
			opts->setVal<string>("compression_method", meth);
			opts->setVal<string>("base_filename", pf.string());
			return true;
		}
		return false;
	}


	void hashStore::addHashStore(pHashStore p, size_t priority)
	{
		loadStores();
		stores.insert ( std::pair<size_t, pHashStore>(priority, p));
	}


	bool hashStore::storeHash(const std::string& h, const std::string &key,
		std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts)
	{
		// Make sure that the store backend is loaded
		std::lock_guard<std::mutex> lck(mHashStore);
		loadStores();

		for (auto &s : stores)
			if (s.second->storeHashInStore(h, key, sh, opts)) return true;
		return false;
	}

	bool hashStore::findHashObj(const std::string& h, const std::string &key,
		std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts)
	{
		// Make sure that the store backend is loaded
		std::lock_guard<std::mutex> lck(mHashStore);
		loadStores();

		for (const auto &s : stores)
			if (s.second->findHashInStore(h, key, sh, opts)) return true;
		return false;
	}


	HASH_t HASHfile(const std::string& filename)
	{
		using namespace boost::interprocess;
		using namespace boost::filesystem;

		using namespace serialization;
		std::string cmeth, fname;
		if (!serialization::detect_compressed(filename, cmeth, fname))
			throw rtmath::debug::xMissingFile(filename.c_str());

		// Do a direct map into memory. It's faster than stream i/o for reading a large file.
		// Plus, all other operations can be done solely in memory.
		size_t fsize = (size_t)file_size(path(fname)); // bytes

		file_mapping m_file(
			fname.c_str(),
			read_only
			);

		mapped_region region(
			m_file,
			read_only,
			0,
			fsize);

		void* start = region.get_address();
		const char* a = (char*)start;

		std::string s(a, fsize);
		std::istringstream ss(s);


		boost::iostreams::filtering_istream sin;
		// sin can contain either compressed or uncompressed input at this point.
		if (cmeth.size())
			prep_decompression(cmeth, sin);
		sin.push(ss);

		std::string suncompressed;
		suncompressed.reserve(1024 * 1024 * 10);
		std::ostringstream so;
		boost::iostreams::copy(sin, so);
		suncompressed = so.str();

		return HASH(suncompressed.c_str(), (int)suncompressed.size());
	}

	HASH_t HASH(const void *key, int len)
	{
		HASH_t res;
		MurmurHash3_x64_128(key, len, HASHSEED, &res);
		return res;
	}

#if USE_RYAN_SERIALIZATION
	template<class Archive>
	void UINT128::serialize(Archive &ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("upper", upper);
		ar & boost::serialization::make_nvp("lower", lower);
	}

	EXPORTINTERNAL(rtmath::UINT128::serialize);
#endif
}

#if USE_RYAN_SERIALIZATION
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::UINT128);
#endif


std::ostream & operator<<(std::ostream &stream, const rtmath::UINT128 &ob)
{
	stream << ob.string();
	return stream;
}


