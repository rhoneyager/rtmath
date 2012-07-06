#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <map>

#pragma warning( push )
#pragma warning( disable : 4244 )
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/filesystem.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>


#include "../rtmath/serialization.h"


#ifdef WITH_CMAKE
#include "cmake-settings.h"
#else
#define COMPRESS_ZLIB 1
#define COMPRESS_GZIP 1
#define COMPRESS_BZIP2 1
#endif


#if COMPRESS_BZIP2
#include <boost/iostreams/filter/bzip2.hpp>
#endif
#if COMPRESS_GZIP
#include <boost/iostreams/filter/gzip.hpp>
#endif
#if COMPRESS_ZLIB
#include <boost/iostreams/filter/zlib.hpp>
#endif

#pragma warning( pop ) 

namespace
{
	// Compression methods
	std::set<std::string> cmeths;
	std::multimap<size_t, std::string> cpriorities;

	void initCmeths()
	{
		static bool inited = false;
		if (inited) return;
		// TODO: allow priority override in rtmath.conf
#if COMPRESS_BZIP2
		cmeths.insert("bz2");
		cpriorities.insert(std::pair<size_t,std::string>(0,"bz2"));
#endif
#if COMPRESS_GZIP
		cmeths.insert("gz");
		cpriorities.insert(std::pair<size_t,std::string>(1,"gz"));
#endif
#if COMPRESS_ZLIB
		cmeths.insert("zlib");
		cpriorities.insert(std::pair<size_t,std::string>(2,"zlib"));
#endif
		inited = true;
	}

}

namespace rtmath
{
	namespace serialization
	{
		bool detect_compressed(const std::string &name, std::string &meth, std::string &target)
		{
			initCmeths();
			using namespace boost::filesystem;
			meth = "";
			target = "";

			path p(name);
			// First, try trivial case
			if (exists(p))
			{
				path ext = p.extension(); // Note: has leading dot
				std::string sext = ext.string().substr(1);
				if (cmeths.count(sext))
				{
					meth = sext;
				}
				target = name;
				return true;
			}

			// File does not exist. Try appending different extensions.
			for (auto it = cmeths.begin(); it != cmeths.end(); ++it)
			{
				std::ostringstream spe(p.string());
				spe << "." << *it;
				path pe(spe.str());
				if (exists(pe))
				{
					meth = *it;
					target = spe.str();
					return true;
				}
			}

			return false;
		}

		bool detect_compression(const std::string &name, std::string &meth)
		{
			meth = "";
			initCmeths();
			using namespace boost::filesystem;
			path p(name);
			path ext = p.extension(); // Note: has leading dot
			std::string sext = ext.string().substr(1);

			if (cmeths.count(sext))
			{
				// TODO: once more algorithms supported, change to a bimap
				meth = sext;
				return true;
			} else {
				return false;
			}
		}

		bool select_compression(const std::string &name, std::string &meth)
		{
			meth = "";
			initCmeths();
			using namespace boost::filesystem;
			
			path p(name);
			path ext = p.extension(); // Note: has leading dot
			std::string sext = ext.string().substr(1);

			// First, look at extension to see if method is already set
			if (cmeths.count(sext))
			{
				meth = sext;
				return true;
			}

			// If not set, go through the list of supported methods and select the most preferred
			if (!cmeths.size()) return false;
			auto it = cpriorities.begin();
			meth = it->second;
			return true;
		}

		void prep_compression(const std::string &meth, boost::iostreams::filtering_streambuf<boost::iostreams::input> &sbuf)
		{
			initCmeths();
			using namespace boost::iostreams;
#if COMPRESS_BZIP2
			if (meth == "bz2")
			{
				sbuf.push(bzip2_decompressor());
			}
#endif
#if COMPRESS_GZIP
			if (meth == "gz")
			{
				sbuf.push(gzip_decompressor());
			}
#endif
#if COMPRESS_ZLIB
			if (meth == "zlib")
			{
				sbuf.push(zlib_decompressor());
			}
#endif
		}

		void prep_compression(const std::string &meth, boost::iostreams::filtering_streambuf<boost::iostreams::output> &sbuf)
		{
			initCmeths();
			using namespace boost::iostreams;
#if COMPRESS_BZIP2
			if (meth == "bz2")
			{
				sbuf.push(bzip2_compressor());
			}
#endif
#if COMPRESS_GZIP
			if (meth == "gz")
			{
				sbuf.push(gzip_compressor());
			}
#endif
#if COMPRESS_ZLIB
			if (meth == "zlib")
			{
				sbuf.push(zlib_compressor());
			}
#endif
		}


	}
}

