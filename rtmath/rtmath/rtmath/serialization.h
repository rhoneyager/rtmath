#pragma once
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
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#pragma warning( pop ) 

#include <ios>
#include <fstream>
#include <string>
#include <sstream>

#include "../rtmath/error/error.h"

namespace rtmath {
	namespace serialization {

		bool detect_compressed(const std::string &base, std::string &meth, std::string &target);

		bool detect_compression(const std::string &name, std::string &meth);

		bool select_compression(const std::string &name, std::string &meth);

		// output and input streambufs supported
		void prep_decompression(const std::string &meth, 
			boost::iostreams::filtering_istream &sbuf);
		void prep_compression(const std::string &meth, 
			boost::iostreams::filtering_ostream &sbuf);

		template<class T>
		void write(const T &obj, std::ostream &out)
		{
			boost::archive::xml_oarchive oa(out);
			oa << BOOST_SERIALIZATION_NVP(obj);
		}

		template<class T>
		void write(const T &obj, boost::iostreams::filtering_ostream &sout)
		{
			std::ostringstream out;
			boost::archive::xml_oarchive oa(out);
			oa << BOOST_SERIALIZATION_NVP(obj);
			//boost::iostreams::copy(out,sout);
			sout << out.str();
		}

		template<class T>
		void write(const T &obj, const std::string &outfile, const std::string &dirsuffix = "", bool autoCompress = true)
		{
			using namespace std;
			using namespace boost::filesystem;
			path pBase(outfile), pXML;
			string cmeth;

			// If passed a directory, attach desired suffix
			if (boost::filesystem::is_directory(pBase))
			{
				if (dirsuffix.size())
				{
					pBase = pBase / dirsuffix;
				} else {
					throw rtmath::debug::xPathExistsWrongType(outfile.c_str());
				}
			}

			// Check for compression
			if (autoCompress)
			{
				if (select_compression(pBase.string(), cmeth))
				{
					ostringstream sCompressed;
					// Note: boost 1.49 or 1.50 implements << operator.  1.48 and below do not.
//					if (pBase.string().find(cmeth) != std::string::npos)
					{
						sCompressed << pBase.string() << "." << cmeth;
						pXML = path(sCompressed.str());
//					} else {
//						pXML = pBase;
					}
				} else {
					pXML = pBase;
				}
			} else {
				pXML = pBase;
			}
			if (exists(pXML))
			{
				boost::filesystem::remove(pXML);
			}

			// Open file for writing with appropriate other params
			std::ofstream out;
			if (cmeth.size())
			{
				out.open(pXML.string().c_str(), ios_base::out | ios_base::binary);
			} else {
				out.open(pXML.string().c_str());
			}

			// Prepare compression
			using namespace boost::iostreams;
			filtering_ostream sout;
			prep_compression(cmeth, sout);

			// Serialize and output
			sout.push(out);
			rtmath::serialization::write<T>(obj,sout);
		}

		template<class T>
		void read(T &obj, std::istream &in)
		{
			boost::archive::xml_iarchive ia(in);
			ia >> BOOST_SERIALIZATION_NVP(obj);
		}

		template<class T>
		void read(T &obj, boost::iostreams::filtering_istream &sin)
		{
			std::stringstream in;
			//sin >> in;
			boost::iostreams::copy(sin,in);
			boost::archive::xml_iarchive ia(in);
			ia >> BOOST_SERIALIZATION_NVP(obj);
		}

		template<class T>
		void read(T &obj, const std::string &infile, const std::string &dirsuffix = "")
		{
			// This routine can accept either a base directory or an actual filename
			// If a base directory is given, search for dirSuffix within the directory
			using namespace std;
			using namespace boost::filesystem;
			bool found = false;
			bool isDir = false;

			path pBase(infile), pTargetBase, pTarget;
			string sTarget, cmeth;
			if (is_directory(pBase))
			{
				if (dirsuffix.size() == 0)
					throw rtmath::debug::xPathExistsWrongType(infile.c_str());
				pTargetBase = pBase / dirsuffix;
			} else {
				pTargetBase = pBase;
			}

			found = detect_compressed(pTargetBase.string(), cmeth, sTarget);
			if (!found) throw rtmath::debug::xMissingFile(pTargetBase.string().c_str());

			pTarget = path(sTarget);

			// Open file for writing with appropriate other params
			ifstream *iin;
			if (cmeth.size())
			{
				iin = new ifstream(pTarget.string().c_str(), ios_base::binary);
			} else {
				iin = new ifstream(pTarget.string().c_str());
			}
			ifstream &in = *iin;

			// Prepare compression
			using namespace boost::iostreams;
			filtering_istream sin;
			prep_decompression(cmeth, sin);
			sin.push(in);

			// Serialize and output
			rtmath::serialization::read<T>(obj,sin);
			delete iin;
		}
	}
}


