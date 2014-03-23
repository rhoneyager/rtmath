#include "Stdafx-core.h"
#include "../rtmath/defs.h"
#include "../rtmath/hash.h"
#include "../rtmath/Public_Domain/MurmurHash3.h"
#include "../rtmath/Serialization/serialization_macros.h"

#include <Ryan_Serialization/serialization.h>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/filesystem.hpp>
#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <string>
#include <boost/lexical_cast.hpp>
#include "../rtmath/error/error.h"

namespace rtmath {

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

	boost::filesystem::path findHash(const boost::filesystem::path &base, const HASH_t &hash,
		const std::vector<std::string> &extensions)
	{
		return findHash(base, boost::lexical_cast<std::string>(hash.lower), extensions);
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

	boost::filesystem::path storeHash(const boost::filesystem::path &base, const HASH_t &hash)
	{
		return storeHash(base, boost::lexical_cast<std::string>(hash.lower));
	}

	HASH_t HASHfile(const std::string& filename)
	{
		using namespace boost::interprocess;
		using namespace boost::filesystem;

		using namespace Ryan_Serialization;
		std::string cmeth, fname;
		if (!detect_compressed(filename, cmeth, fname))
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

	template<class Archive>
	void UINT128::serialize(Archive &ar, const unsigned int version)
	{
		ar & boost::serialization::make_nvp("upper", upper);
		ar & boost::serialization::make_nvp("lower", lower);
	}

	EXPORTINTERNAL(rtmath::UINT128::serialize);
}


BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::UINT128);
