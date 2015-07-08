#include "../Ryan_Debug/defs.h"
#include "../Ryan_Debug/hash.h"
#include "../Ryan_Debug/MurmurHash3.h"

#include <mutex>
#include <map>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <string>
#include <boost/lexical_cast.hpp>
#include "../Ryan_Debug/config.h"
#include "../Ryan_Debug/Serialization.h"
#include "../Ryan_Debug/error.h"

/// Handles private hash functions
namespace {
	std::mutex mHashStore, mLoadingStores, mLoadingStoresB;
	std::multimap<size_t, Ryan_Debug::hash::pHashStore> stores;
	std::set<std::string> known_ids;
	void loadStores() {
		std::lock_guard<std::mutex> lck(mLoadingStoresB);
		static bool inited = false;
		if (inited) return;
		inited = true;
		auto& lg = Ryan_Debug::hash::m_hash::get();
		auto conf = Ryan_Debug::config::loadRtconfRoot();
		if (!conf) return;
		auto chash = conf->getChild("hashes");
		if (!chash) {
			BOOST_LOG_SEV(lg, Ryan_Debug::log::warning) << "Ryan_Debug configuration file does "
				"not have a /hashes key. Unable to load hash stores for default setup.";
			//RDthrow(Ryan_Debug::error::xMissingKey())
			//	<< Ryan_Debug::error::otherErrorText("/ddscat/hashes in Ryan_Debug config is missing");
			return;
		}
		Ryan_Debug::hash::hashStore::loadStoresFromSource(chash, "");
	}
}

namespace Ryan_Debug {
	namespace registry {
		template class usesDLLregistry <
			::Ryan_Debug::hash::hash_provider_registry,
			::Ryan_Debug::hash::Hash_registry_provider > ;
	}
	namespace hash {
		Hash_registry_provider::~Hash_registry_provider() {}
		Hash_registry_provider::Hash_registry_provider() {}

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
		//RDthrow Ryan_Debug::debug::xMissingFile(base.string().c_str());

		if (!exists(base / pHashStart))
		create_directory(base / pHashStart);

		return base/pHashStart/pHashName;
		}
		*/

		hashStore::hashStore() {}
		hashStore::~hashStore() {}

		bool hashStore::storeHashInStore(const std::string& h, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts,
			const std::string &tag) const
		{
			if (tag != this->tag) return false;
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
				RDthrow(error::xMissingFolder())
				<< error::folder_name(base.string())
				<< error::hash(h)
				<< error::hashType(key);

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
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts,
			const std::string &tag) const
		{
			if (tag != this->tag) return false;
			opts = registry::IO_options::generate(registry::IOhandler::IOtype::READONLY);
			sh = nullptr; // IOhandler is not needed for the basic filesystem store
			auto& lg = Ryan_Debug::hash::m_hash::get();

			using namespace boost::filesystem;
			using boost::lexical_cast;
			using serialization::detect_compressed;
			using std::string;

			const string sHashName = hash;
			const string sHashStart = sHashName.substr(0, 2);
			path pHashName(sHashName);
			path pHashStart(sHashStart);

			try {

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
			}
			catch (::boost::exception &e) {
				std::ostringstream serr;
				serr << "When searching " << base << " - " << hash << " the following error was encountered:\n";
				serr << boost::diagnostic_information(e);
				BOOST_LOG_SEV(lg, Ryan_Debug::log::warning) << serr.str();
			}
			catch (boost::filesystem::filesystem_error &f) {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::warning)
					<< f.what() << " when searching " << base << " - " << hash;
			}
			catch (boost::system::system_error &f) {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::error)
					<< f.what() << " when searching " << base << " - " << hash;
				throw;
			}
			catch (...) {
				BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "An unhandled error occurred "
					" when searching " << base << " - " << hash;
				throw;
			}
			return false;
		}


		void hashStore::addHashStore(pHashStore p, size_t priority)
		{
			loadStores();
			stores.insert(std::pair<size_t, pHashStore>(priority, p));
		}


		bool hashStore::storeHash(const std::string& h, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts,
			const std::string &tag)
		{
			// Make sure that the store backend is loaded
			std::lock_guard<std::mutex> lck(mHashStore);
			loadStores();

			for (auto &s : stores)
				if (s.second->storeHashInStore(h, key, sh, opts, tag)) return true;
			// Log an error and throw.
			auto& lg = Ryan_Debug::hash::m_hash::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::error) << "No hash stores were available to store "
				"the object. Hash: " << h << ", Key: " << key << ", tag: " << tag;
			RDthrow(Ryan_Debug::error::xMissingKey())
				<< Ryan_Debug::error::otherErrorText("No hash stores accepted the given object. See log details. ");
			return false;
		}

		bool hashStore::findHashObj(const std::string& h, const std::string &key,
			std::shared_ptr<registry::IOhandler> &sh, std::shared_ptr<registry::IO_options> &opts,
			const std::string &tag)
		{
			// Make sure that the store backend is loaded
			std::lock_guard<std::mutex> lck(mHashStore);
			loadStores();

			for (const auto &s : stores)
				if (s.second->findHashInStore(h, key, sh, opts, tag)) return true;
			return false;
		}


		HASH_t HASHfile(const std::string& filename)
		{
			using namespace boost::interprocess;
			using namespace boost::filesystem;

			using namespace serialization;
			std::string cmeth, fname;
			if (!serialization::detect_compressed(filename, cmeth, fname))
				RDthrow(::Ryan_Debug::error::xMissingFile())
				<< ::Ryan_Debug::error::file_name(filename);

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

		/// \param conf is the configuration tree being used to load hashes
		/// \param id is a unique id for this tree to avoid loading the same hash sources twice
		void hashStore::loadStoresFromSource(boost::shared_ptr<const Ryan_Debug::config::configsegment> conf, const char* id)
		{
			std::lock_guard<std::mutex> lck(mLoadingStores);
			if (known_ids.count(std::string(id))) return;
			known_ids.insert(std::string(id));

			auto& lg = Ryan_Debug::hash::m_hash::get();
			BOOST_LOG_SEV(lg, Ryan_Debug::log::normal) << "Loading hash stores";
			/// \todo Implement most of this function

			// Stores can be folder trees, single hdf5 files, and websites.
			// The store database can be stored in sql. 
			/// \todo Add entries in registry for store registration

			// Each entry in the store has a priority, reflecting its use order.
			// The highest-priority writable store is used for writing, and the first store 
			// having a given key is used when reading.

			{
				//auto conf = Ryan_Debug::config::loadRtconfRoot();
				//if (!conf) return;
				//auto cdd = conf->getChild("ddscat");
				//if (!cdd) cdd = conf;
				//auto chash = cdd->getChild("hashes");
				auto chash = conf;
				if (!chash) { // Should never happen. Code beanch is superseded and moved elsewhere.
					BOOST_LOG_SEV(lg, Ryan_Debug::log::critical) << "Ryan_Debug configuration file does "
						"not have a /ddscat/hashes or /hashes key. Unable to load hash stores.";
					RDthrow(Ryan_Debug::error::xMissingKey())
						<< Ryan_Debug::error::otherErrorText("/ddscat/hashes in Ryan_Debug config is missing");
					return;
				}
				// Iterate over all hash store entries
				std::multiset<boost::shared_ptr<Ryan_Debug::config::configsegment> > children;
				chash->listChildren(children);
				for (const auto &c : children)
				{
					if (c->name() != "store") continue;

					bool enabled = true;
					if (c->hasVal("enabled"))
						c->getVal<bool>("enabled", enabled);
					if (!enabled) continue;

					size_t priority = 999;
					if (c->hasVal("priority"))
						c->getVal<size_t>("priority", priority);

					std::string location;
					if (c->hasVal("path"))
						c->getVal<std::string>("path", location);

					bool writable = false;
					if (c->hasVal("writable"))
						c->getVal<bool>("writable", writable);

					std::string type = "dir";
					if (c->hasVal("type"))
						c->getVal<std::string>("type", type);

					std::string tag = "";
					if (c->hasVal("tag"))
						c->getVal<std::string>("tag", tag);

					BOOST_LOG_SEV(lg, Ryan_Debug::log::debug_1) << "Parsed hash store:\n"
						<< "enabled: " << enabled
						<< "\npriority: " << priority
						<< "\npath: " << location
						<< "\ntype: " << type
						<< "\nwritable: " << writable
						<< "\ntag: " << tag;

					std::shared_ptr < Ryan_Debug::hash::hashStore > h;

					if (type == "dir") {
						h = std::shared_ptr<Ryan_Debug::hash::hashStore>(new Ryan_Debug::hash::hashStore);
					} /// \todo Add hash store plugin search code here, and fill in the store generator in the header file.
					else RDthrow(::Ryan_Debug::error::xUnknownFileFormat())
						<< ::Ryan_Debug::error::otherErrorText("Hash store code currently "
						"only supports \"dir\"-type stores. TODO: Add "
						"hash store plugin search code in hash.cpp and "
						"the header file.")
						<< ::Ryan_Debug::error::hashType(type);

					h->writable = writable;
					h->base = boost::filesystem::path(location);
					h->type = type;
					h->tag = tag;

					Ryan_Debug::hash::hashStore::addHashStore(h, priority);
				}
			}
		}
	}

}

std::ostream & operator<<(std::ostream &stream, const Ryan_Debug::hash::UINT128 &ob)
{
	stream << ob.string();
	return stream;
}


