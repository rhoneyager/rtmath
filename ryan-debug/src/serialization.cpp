#pragma warning( disable : 4996 )
#pragma warning( disable : 4251 )
#pragma warning( disable : 4275 )
#pragma warning( disable : 4244 )
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <set>
#include <map>

#pragma warning( push )
#include <boost/filesystem.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/copy.hpp>


#include "../Ryan_Debug/Serialization.h"


#include "cmake-settings.h"



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


typedef std::set<std::string>::const_iterator cmit;
typedef std::multimap<size_t, std::string>::const_iterator cpit;

// Oddly, I can't use static sets directly, and attempts at initializing at 
// dlopen proved futile on gcc 4.7. May be a compiler bug. This formulation 
// fixes it.

/// This structure holds the per-instance serialization library settings
struct sdata
{
	bool disableAuto;
	/// Compression methods
	std::set<std::string> cmeths;
	/// Compression method ordering
	std::multimap<size_t, std::string> cpriorities;
	/// Default serialization extension
	Ryan_Debug::serialization::serialization_method defaultType;
	/// Recognize these extensions as XML files
	std::set<std::string> extensionsXML;
	/// Recognize these extensions as text files
	std::set<std::string> extensionsText;

	sdata() : disableAuto(false), defaultType(Ryan_Debug::serialization::serialization_method::XML) 
	{ initCmeths(); initSmeths(); }
	/// Initialize list of serialization types to use when reading and writing
	void initSmeths()
	{
		extensionsXML.clear();
		extensionsText.clear();
		extensionsXML.insert(".xml");
		extensionsText.insert(".st");
		//extensionsText.insert(".txt");
	}
	/**
	* \brief Initialize list of compression types to use when writing.
	*
	* Lowest priority number is selected for use. If a selection is
	* specified, all writes will occur using this method. If a method
	* is unsupported at compile time, it is left blank.
	*
	* Does not affect reads at all.
	* \item selection is the string matching a forced compression type (bz2, gz, zlib)
	**/
	void initCmeths(const std::string &selection = "")
	{
		cmeths.clear();
		cpriorities.clear();

		//if (selection.size() == 0)
		{
#if COMPRESS_BZIP2
			cmeths.insert("bz2");
			cpriorities.insert(std::pair<size_t, std::string>(1, "bz2"));
#endif
#if COMPRESS_GZIP
			cmeths.insert("gz");
			cpriorities.insert(std::pair<size_t, std::string>(2, "gz"));
#endif
#if COMPRESS_ZLIB
			cmeths.insert("zlib");
			cpriorities.insert(std::pair<size_t, std::string>(3, "zlib"));
#endif
			//} else {
			if (selection == "bz2")
			{
#if COMPRESS_BZIP2
				cmeths.insert("bz2");
				cpriorities.insert(std::pair<size_t, std::string>(0, "bz2"));
#endif
			}
			if (selection == "gz")
			{
#if COMPRESS_GZIP
				cmeths.insert("gz");
				cpriorities.insert(std::pair<size_t, std::string>(0, "gz"));
#endif
			}
			if (selection == "zlib")
			{
#if COMPRESS_ZLIB
				cmeths.insert("zlib");
				cpriorities.insert(std::pair<size_t, std::string>(0, "zlib"));
#endif
			}
		}
	}

};

static sdata d;

/*
// DLL binding and unbinding code
#ifndef _MSC_FULL_VER //__GNUC__, __llvm__, __clang__, __INTEL_COMPILER, ...
void __attribute__ ((constructor)) serialization_gcc_init()
{
//	sdata d();
}

void __attribute__ ((destructor)) serialization_gcc_fini()
{
}
#endif
#ifdef _MSC_FULL_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
if (dwReason == DLL_PROCESS_ATTACH)
{
//		initCmeths();
// DisableThreadLibraryCalls(hInstance);
}
return true;
}
#endif
*/


namespace Ryan_Debug
{
	namespace serialization
	{

		template <typename T>
		bool detect_compressed(const T &name)
		{
			std::string a;
			T b;
			return detect_compressed<T>(name, a, b);
		}

		template bool RYAN_DEBUG_DLEXPORT detect_compressed
			(const std::string&);
		template bool RYAN_DEBUG_DLEXPORT detect_compressed
			(const boost::filesystem::path&);

		template <typename T>
		bool detect_compressed(const T &name, std::string &meth, T &target)
		{
			using boost::filesystem::path;
			using boost::filesystem::exists;
			meth = "";
			target = "";

			path p(name);
			// First, try trivial case
			if (exists(p))
			{
				path ext = p.extension(); // Note: has leading dot
				std::string sext;
				if (ext.string().size())
					sext = ext.string().substr(1);
				if (d.cmeths.count(sext))
				{
					meth = sext;
				}
				target = name;
				return true;
			}

			// File does not exist. Try appending different extensions.
			for (cmit it = d.cmeths.begin(); it != d.cmeths.end(); ++it)
			{
				std::ostringstream spe;
				spe << p.string() << "." << *it;
				std::string s = spe.str(); // Odd stuff happens with ostringstream::str
				path pe(s);
				if (exists(pe))
				{
					meth = *it;
					target = T(spe.str());
					return true;
				}
			}

			return false;
		}

		template bool RYAN_DEBUG_DLEXPORT detect_compressed
			(const std::string&, std::string&, std::string&);
		template bool RYAN_DEBUG_DLEXPORT detect_compressed
			(const boost::filesystem::path&, std::string&, boost::filesystem::path&);

		template <typename T>
		bool detect_compression(const T &name, std::string &meth)
		{
			meth = "";
			using boost::filesystem::path;
			path p(name);
			path ext = p.extension(); // Note: has leading dot
			std::string sext;
			if (ext.string().size())
				sext = ext.string().substr(1);

			if (d.cmeths.count(sext))
			{
				// TODO: once more algorithms supported, change to a bimap
				meth = sext;
				return true;
			}
			else {
				return false;
			}
		}

		template bool RYAN_DEBUG_DLEXPORT detect_compression
			(const std::string&, std::string&);
		template bool RYAN_DEBUG_DLEXPORT detect_compression
			(const boost::filesystem::path&, std::string&);

		void known_compressions(std::string &output, const std::string &base)
		{
			//output.clear();

			auto makeString = [](const std::set<std::string>& s) -> std::string
			{
				std::string sout;
				std::ostringstream out;
				for (auto it = s.cbegin(); it != s.cend(); ++it)
				{
					if (sout.size()) out << ",";
					else if (it != s.cbegin()) out << ",";
					out << *it;
				}
				sout.append(out.str());
				return sout;
			};

			// extBase is the base serialization extensions
			std::set<std::string> extBase;
			if (base.size())
				if (!extBase.count(base))
					extBase.insert(base);

			// cBase holds the different compression method extensions.
			std::set<std::string> cBase = d.cmeths;

			{
				std::set<std::string> res = extBase;

				for (const auto &c : cBase)
				{
					std::string sc = base;
					sc.append(".");
					sc.append(c);
					res.insert(sc);
				}

				output = makeString(res);
			}
		}

		void known_formats(std::string &output, bool includeCompressions)
		{
			output.clear();

			auto makeString = [](const std::set<std::string>& s) -> std::string
			{
				std::string sout;
				std::ostringstream out;
				for (auto it = s.cbegin(); it != s.cend(); ++it)
				{
					if (it != s.cbegin()) out << ",";
					out << *it;
				}
				sout = out.str();
				return sout;
			};

			// extBase is the base serialization extensions
			std::set<std::string> extBase = d.extensionsText;
			for (const auto& i : d.extensionsXML)
				if (!extBase.count(i))
					extBase.insert(i);

			// cBase holds the different compression method extensions.
			std::set<std::string> cBase = d.cmeths;

			if (includeCompressions)
			{
				std::set<std::string> res = extBase;

				for (const auto &s : extBase)
					for (const auto &c : cBase)
					{
					std::string sc = s;
					sc.append(".");
					sc.append(c);
					res.insert(sc);
					}

				output = makeString(res);
			}
			else {
				output = makeString(extBase);
			}
		}

		template <typename T>
		bool uncompressed_name(const T &name, T &uncompressed, std::string &meth)
		{
			bool isCompressed = detect_compression(name, meth);
			if (isCompressed)
			{
				using boost::filesystem::path;
				path p(name);
				path pu = p.replace_extension();
				uncompressed = pu.string();
			}
			else {
				uncompressed = name;
			}
			return isCompressed;
		}

		template bool RYAN_DEBUG_DLEXPORT uncompressed_name
			(const std::string&, std::string&, std::string&);
		template bool RYAN_DEBUG_DLEXPORT uncompressed_name
			(const boost::filesystem::path&, boost::filesystem::path&, std::string&);

		template <typename T>
		bool select_compression(const T &name, std::string &meth)
		{
			//		std::cerr << "Selecting from " << d.cmeths.size() << " methods\n";
			meth = "";
			using boost::filesystem::path;

			path p(name);
			path ext = p.extension(); // Note: has leading dot
			if (ext.string() == "") ext = p; // Avoids bug when path has no dot
			std::string sext;
			if (ext.string().size())
				sext = ext.string().substr(1);

			// First, look at extension to see if method is already set
			if (d.cmeths.count(sext))
			{
				meth = sext;
				return true;
			}

			// If not set, go through the list of supported methods and select the most preferred
			if (!d.cmeths.size()) return false;
			if (d.disableAuto) return false;
			cpit it = d.cpriorities.begin();
			meth = it->second;
			return true;
		}

		template bool RYAN_DEBUG_DLEXPORT select_compression
			(const std::string&, std::string&);
		template bool RYAN_DEBUG_DLEXPORT select_compression
			(const boost::filesystem::path&, std::string&);

		template <typename T>
		Ryan_Debug::serialization::serialization_method select_format(const T &name, T &outname)
		{
			using std::string;
			using boost::filesystem::path;
			path p(name), uncompressed;
			string meth;
			uncompressed_name(p, uncompressed, meth);
			outname = name;

			// Ensure that the serialized object has an appropriate extension if none has been specified.
			if (!uncompressed.has_extension())
			{
				path cOutname = uncompressed;
				auto addExt = [&](const serialization_method &m, const std::set<std::string> &exts)
				{
					if (d.defaultType != m) return;
					if (!exts.size()) return;
					cOutname += *exts.begin();
				};
				addExt(serialization_method::XML, d.extensionsXML);
				addExt(serialization_method::TEXT, d.extensionsText);
				cOutname += path(meth);
				outname = T(cOutname.string());
				return d.defaultType;
			}
			path pext = uncompressed.extension();
			if (d.extensionsXML.count(pext.string())) return serialization_method::XML;
			if (d.extensionsText.count(pext.string())) return serialization_method::TEXT;
			return d.defaultType;
		}

		template Ryan_Debug::serialization::serialization_method RYAN_DEBUG_DLEXPORT select_format
			(const std::string&, std::string&);
		template Ryan_Debug::serialization::serialization_method RYAN_DEBUG_DLEXPORT select_format
			(const boost::filesystem::path&, boost::filesystem::path&);

		template<typename T>
		bool known_format(const T &filename, const serialization_method &meth)
		{
			using namespace boost::filesystem;
			using std::string;
			path pf(filename);
			// First assume it's just an extension
			auto checkExt = [&](const boost::filesystem::path &p, std::set<std::string> &exts)
			{
				for (auto &e : exts)
					if (path(e) == p) return true;
				return false;
			};
			if (meth == serialization_method::XML)
				if (checkExt(pf, d.extensionsXML)) return true;
			if (meth == serialization_method::TEXT)
				if (checkExt(pf, d.extensionsText)) return true;

			// Then, assume that the path needs to be broken down, and redo.
			path uncompressed, uext;
			string cmeth;
			uncompressed_name(pf, uncompressed, cmeth);
			if (!uncompressed.has_extension()) return false;
			uext = uncompressed.extension();

			if (meth == serialization_method::XML)
				if (checkExt(uext, d.extensionsXML)) return true;
			if (meth == serialization_method::TEXT)
				if (checkExt(uext, d.extensionsText)) return true;
			return false;
		}

		template bool RYAN_DEBUG_DLEXPORT known_format
			(const boost::filesystem::path &filename, const Ryan_Debug::serialization::serialization_method &meth);
		template bool RYAN_DEBUG_DLEXPORT known_format
			(const std::string &filename, const Ryan_Debug::serialization::serialization_method &meth);

		template<typename T>
		bool known_format(const T &filename)
		{
			using namespace boost::filesystem;
			path pf(filename);
			// Nonoptimal, but fast to write, and the function call should complete swiftly.
			if (known_format(pf, serialization_method::XML)) return true;
			if (known_format(pf, serialization_method::TEXT)) return true;
			return false;
		}

		template bool RYAN_DEBUG_DLEXPORT known_format
			(const boost::filesystem::path &filename);
		template bool RYAN_DEBUG_DLEXPORT known_format
			(const std::string &filename);

		void disable_auto_compression(bool val)
		{
			d.disableAuto = val;
		}

		bool auto_compression_enabled()
		{
			return !d.disableAuto;
		}

		void prep_decompression(const std::string &meth, boost::iostreams::filtering_istream &sbuf)
		{
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

		void prep_compression(const std::string &meth, boost::iostreams::filtering_ostream &sbuf)
		{
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

		template<> RYAN_DEBUG_DLEXPORT void read<std::string>(std::string &obj, std::stringstream &in)
		{
			obj = in.str();
		}


	}
}

