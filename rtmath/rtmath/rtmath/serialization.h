#pragma once
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>

namespace rtmath {
	namespace serialization {
		// TODO: add gzip/bzip/lzma functionality with boost::iostreams
		template<class T>
			void write(const T &obj, const std::string &outfile, const std::string &dirsuffix = "", bool autoCompress = false)
			{
				// TODO: check if outfile can be Compressed, then figure out an appropriate algorithm
				using namespace boost::filesystem;
				path pBase(outfile), pXML;
				if (is_directory(pBase))
				{
					if (dirsuffix.size() == 0)
						throw rtmath::debug::xPathExistsWrongType(pBase.string().c_str());
					pXML = pBase / path(dirsuffix);
					if (exists(pXML))
					{
						if (is_directory(pXML))
							throw rtmath::debug::xPathExistsWrongType(pXML.string().c_str());
						boost::filesystem::remove(pXML);
					}
					//throw rtmath::debug::xPathExistsWrongType
				} else {
					pXML = pBase;
					if (exists(pXML))
					{
						boost::filesystem::remove(pXML);
					}
				}

				// Okay, now to serialize and output...
				std::ofstream out(pXML.string().c_str());
				//boost::archive::text_oarchive oa(out);
				// oa << *this;
				boost::archive::xml_oarchive oa(out);
				oa << BOOST_SERIALIZATION_NVP(obj);
		
			}
		template<class T>
			void read(T &obj, const std::string &infile, const std::string &dirsuffix = "")
			{
				// TODO: if infile does not exist, search for compressed versions (by adding extensions)
				// This routine can accept either a base directory or an actual filename
				// If a base directory is given, search for runSet.xml.
				using namespace boost::filesystem;
				path pBase(infile), pXML;
				if (!exists(pBase)) throw rtmath::debug::xMissingFile(basename.c_str());
				if (is_directory(pBase))
				{
					if (dirsuffix.size() == 0)
						throw rtmath::debug::xPathExistsWrongType(pBase.string().c_str());
					// Search for runSet.xml
					path pCand = pBase / path(dirsuffix);
					if (exists(pCand))
					{
						if (!is_directory(pCand))
						{
							pXML = pCand;
						} else {
							throw rtmath::debug::xPathExistsWrongType(pCand.string().c_str());
						}
					} else {
						throw rtmath::debug::xMissingFile(pCand.string().c_str());
					}
				} else 
				{
					pXML = pBase;
				}

				// Okay, now to serialize and input...
				std::ifstream in(pXML.string().c_str());
				//boost::archive::text_oarchive oa(out);
				// oa << *this;
				boost::archive::xml_iarchive ia(in);
				ia >> BOOST_SERIALIZATION_NVP(obj);
		
			}
	}
}


