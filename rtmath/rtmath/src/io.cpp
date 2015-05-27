#include "Stdafx-core.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <memory>
#include <mutex>
#if USE_RYAN_SERIALIZATION
#include <Ryan_Serialization/serialization.h>
#endif
#include <boost/algorithm/string.hpp>
#include <boost/iostreams/filter/newline.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/filesystem.hpp>
#include "../rtmath/splitSet.h"
#include "../rtmath/registry.h"
#include "../rtmath/Serialization/Serialization.h"
#include "../rtmath/io.h"

namespace {
	const char* hid = "io_implementsSerialization";
	std::set<std::string> mtypes;
	std::mutex mlock_serialization_handle;
	std::mutex mlock_implementsIO;
}

namespace rtmath
{
	namespace io
	{
		namespace TextFiles
		{
			class hSerialization
			{
				friend struct serialization_handle;
			public:
				~hSerialization() {}
			private:
				hSerialization() {}
				/// For loading file
				std::unique_ptr<std::ifstream> fi;
				/// For writing file
				std::unique_ptr<std::ofstream> fo;
				/// Boost stream to load file, applying compression filters
				std::shared_ptr<boost::iostreams::filtering_istream> bfi;
				/// Boost stream to write file, applying compression filters
				std::shared_ptr<boost::iostreams::filtering_ostream> bfo;
			};


			bool serialization_handle::compressionEnabled()
			{
				/// \note Disabled because it is not working on Windows AND Linux. Very odd.
				return false;
			}

			const char* serialization_handle::getSHid()
			{
				return hid;
			}

			serialization_handle::serialization_handle(const char* filename,
				::rtmath::registry::IOhandler::IOtype t) :
				IOhandler(hid), h(new hSerialization)
			{
				open(filename, t);
			}

			serialization_handle::~serialization_handle()
			{
			}

			void serialization_handle::open(const char* filename, ::rtmath::registry::IOhandler::IOtype t)
			{
				using namespace boost::filesystem;
				switch (t)
				{
				case IOtype::READONLY:
				{
					if (!exists(path(filename))) RDthrow(debug::xMissingFile())
						<< debug::file_name(filename);
					load(filename);
				}
					break;
				case IOtype::CREATE:
					if (exists(path(filename))) RDthrow(debug::xFileExists())
						<< debug::file_name(filename);
				case IOtype::TRUNCATE:
					create(filename);
					break;
				case IOtype::EXCLUSIVE:
				case IOtype::DEBUG:
				case IOtype::READWRITE:
					RDthrow(debug::xUnsupportedIOaction())
						<< debug::otherErrorText("IO mode READWRITE "
						"is currently unsupported in serialization code.");
					break;
				}
			}

			void serialization_handle::load(const char* fname)
			{
				// Check file existence
				using namespace std;
				using namespace boost::filesystem;
				using namespace serialization;
				std::string cmeth, target, uncompressed, filename(fname);
				// Combination of detection of compressed file, file type and existence.
				if (!detect_compressed(filename, cmeth, target))
					RDthrow(rtmath::debug::xMissingFile())
					<< rtmath::debug::file_name(filename);
				uncompressed_name(target, uncompressed, cmeth);

				boost::filesystem::path p(uncompressed);
				boost::filesystem::path pext = p.extension(); // Uncompressed extension

				h->fi = std::unique_ptr<std::ifstream>
					(new std::ifstream(fname, std::ios_base::binary | std::ios_base::in));
				// Consutuct an filtering_iostream that matches the type of compression used.
				using namespace boost::iostreams;

				h->bfi = (std::shared_ptr<filtering_istream>
					(new filtering_istream));
				if (cmeth.size())
					prep_decompression(cmeth, *(h->bfi.get()));
				h->bfi->push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
				h->bfi->push(*(h->fi.get()));

				reader = h->bfi;
			}

			void serialization_handle::create(const char* fname)
			{
				using namespace serialization;
				std::string cmeth, uncompressed, filename(fname);
				uncompressed_name(filename, uncompressed, cmeth);
				boost::filesystem::path p(uncompressed);
				boost::filesystem::path pext = p.extension(); // Uncompressed extension


				h->fo = (std::unique_ptr<std::ofstream>
					(new std::ofstream(fname, std::ios_base::trunc | std::ios_base::binary | std::ios_base::out)));
				// Consutuct an filtering_iostream that matches the type of compression used.
				using namespace boost::iostreams;

				h->bfo = (std::shared_ptr<filtering_ostream>
					(new filtering_ostream));
				if (cmeth.size())
					prep_compression(cmeth, *(h->bfo.get()));
				h->bfo->push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
				h->bfo->push(*(h->fo.get()));

				writer = h->bfo;
			}

			const std::set<std::string>& serialization_handle::known_formats()
			{
				// Moved to hidden file scope to avoid race condition
				//static std::set<std::string> mtypes;
				//static std::mutex mlock;
				// Prevent threading clashes
				{
					std::lock_guard<std::mutex> lck(mlock_serialization_handle);
					if (!mtypes.size())
					{
						std::string formats;
						serialization::known_formats(formats, compressionEnabled());
						rtmath::config::splitSet(formats, mtypes);
					}
				}
				return mtypes;
			}

			bool serialization_handle::match_file_type(const char* filename,
				const char* type,
				const std::set<std::string> &mtypes, 
				const char *op)
			{
				using namespace boost::filesystem;
				using std::string;
				using std::ofstream;
				auto& lg = rtmath::io::m_io::get();
				BOOST_LOG_SEV(lg, rtmath::debug::normal) << "Matching file type for file: " << filename 
					<< ", with type: " << type << ", with op (exportType): " << op<< "\n";
				for (const auto & mt : mtypes)
					BOOST_LOG_SEV(lg, rtmath::debug::normal) << "Matched type enumeration: " << mt << "\n";

				//const std::set<string> &mtypes = known_formats();

				string sop(op);
				if (sop.size()) return false;

				// Actually comparing the type and file extension.
				string stype(type);
				string sfilename(filename);

				for (const auto &m : mtypes)
				{
					auto match = [](const std::string &a, const std::string &b) -> bool
					{
						auto res = a.find(b);
						if (res == string::npos) return false;
						// Verify that the extension is at the end of the filename
						string sa = a.substr(a.size() - b.size());
						if (sa == b) return true;
						return false;
					};
					auto res = sfilename.find(m);
					auto resb = stype.find(m);

					if (match(sfilename, m)) return true;
					if (match(stype, m)) return true;
				}

				return false;
			}

			bool serialization_handle::match_file_type_multi(
				std::shared_ptr<rtmath::registry::IOhandler> h,
				const char* pluginid,
				std::shared_ptr<rtmath::registry::IO_options> opts, 
					const std::set<std::string> &mtypes)
			{
				std::string spluginid(pluginid);
				if (h)
				{
					if (h->getId() != spluginid) return false;
					return true;
				} else {
					std::string filename = opts->filename();
					std::string type = opts->filetype();
					return match_file_type(filename.c_str(), type.c_str(), mtypes,
						opts->exportType().c_str());
				}
			}

		}

		std::mutex& getLock()
		{
			return mlock_implementsIO;
		}
	}
}

