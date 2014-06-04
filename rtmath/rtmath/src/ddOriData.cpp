#include "Stdafx-ddscat.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/iostreams/filter/newline.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include "../rtmath/ddscat/ddOutput.h"
#include "../rtmath/ddscat/ddOriData.h"
#include "../rtmath/ddscat/ddOutputSingleKeys.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/refract.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"


namespace rtmath {

	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::ddscat::ddOriData>;

		template struct IO_class_registry_reader
			<::rtmath::ddscat::ddOriData>;

		template class usesDLLregistry<
			::rtmath::ddscat::ddOriData_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddOriData> >;

		template class usesDLLregistry<
			::rtmath::ddscat::ddOriData_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddOriData> >;
	}


	namespace ddscat {

		implementsDDRES::implementsDDRES() :
			rtmath::io::implementsIObasic<ddOriData, ddOriData_IO_output_registry,
			ddOriData_IO_input_registry, ddOutputSingle_Standard>(ddOriData::writeDDSCAT, ddOriData::readDDSCAT, known_formats())
		{}

		const std::set<std::string>& implementsDDRES::known_formats()
		{
			static std::set<std::string> mtypes;
			static std::mutex mlock;
			// Prevent threading clashes
			{
				std::lock_guard<std::mutex> lck(mlock);
				if (!mtypes.size())
				{
					mtypes.insert(".avg");
					mtypes.insert(".fml");
					mtypes.insert(".sca");
					mtypes.insert("avg_");
				}
				if (io::TextFiles::serialization_handle::compressionEnabled())
				{
					std::string sctypes;
					std::set<std::string> ctypes;
					Ryan_Serialization::known_compressions(sctypes, ".avg");
					Ryan_Serialization::known_compressions(sctypes, ".fml");
					Ryan_Serialization::known_compressions(sctypes, ".sca");
					Ryan_Serialization::known_compressions(sctypes, "avg_");
					rtmath::config::splitSet(sctypes, ctypes);
					for (const auto & t : ctypes)
						mtypes.emplace(t);
				}
			}
			return mtypes;
		}

		void ddOriData::readDDSCAT(ddOriData* obj, std::istream&in, std::shared_ptr<registry::IO_options> opts)
		{
			std::string filename = opts->filename();
			std::string filetype = opts->filetype();
			std::string ext = opts->extension();
			std::string utype = filetype;
			if (!utype.size()) utype = filetype;
			if (!utype.size()) utype = boost::filesystem::path(filename).extension().string().c_str();
			if (utype == ".sca") {
				obj->readSCA(in);
			}
			else if (utype == ".fml") {
				obj->readFML(in);
			}
			else if (utype == ".avg") {
				obj->readAVG(in);
			}
			else if (filename.find("avg_") != std::string::npos) {
				obj->readAVG(in);
			}
			else {
				throw rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		void ddOriData::writeDDSCAT(const ddOriData* obj, std::ostream &out, std::shared_ptr<registry::IO_options> opts)
		{
			std::string filename = opts->filename();
			std::string filetype = opts->filetype();
			std::string ext = opts->extension();
			std::string utype = filetype;
			if (!utype.size()) utype = filetype;
			if (!utype.size()) utype = boost::filesystem::path(filename).extension().string().c_str();
			if (utype == ".sca") {
				obj->writeSCA(out);
			}
			else if (utype == ".fml") {
				obj->writeFML(out);
			}
			else if (utype == ".avg") {
				obj->writeAVG(out);
			}
			else if (filename.find("avg_") != std::string::npos) {
				obj->writeAVG(out);
			}
			else {
				throw rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		ddOriData::ddOriData(ddOutput &parent, size_t row, const std::string &infile, const std::string &type) :
			_parent(parent), _row(row)
		{
			_init();
			if (infile.size()) readFile(infile, type);
		}
		
		void ddOriData::_init()
		{
			//_statTable_Size_ts.at(stat_entries_size_ts::VERSION)
			//	= rtmath::ddscat::ddVersions::getDefaultVer();
		}

		ddOriData::~ddOriData() {}

		/// Input in avg format
		void ddOriData::readAVG(std::istream &in)
		{
			readHeader(in);
			readStatTable(in);
			readMueller(in);
		}

		/// Input in sca format
		void ddOriData::readSCA(std::istream &in)
		{
			readHeader(in);
			readStatTable(in);
			readMueller(in);
		}

		/// Input in fml format
		void ddOriData::readFML(std::istream &in)
		{
			readHeader(in, "Re(f_11)");
			// Get e1 and e2 in lab frame from the header data
			auto cn = getConnector();
			readF(in, cn);
		}

		namespace ddOriDataParsers
		{
			struct version
			{
				/// \note Version is specified internally, and defaults to the latest version.
				static void write(std::ostream &out, size_t v){
					out << " DDSCAT --- ";
					out << rtmath::ddscat::ddVersions::getVerAvgHeaderString(v);
					out << std::endl;
				}
				static size_t read(std::istream &in, size_t)
				{
					std::string lin;
					std::getline(in, lin);
					return rtmath::ddscat::ddVersions::getVerId(lin);
				}
			};
			struct simpleString
			{
				static void write(std::ostream &out, size_t, const std::string &s, const std::string &p)
				{
					out << " " << p << " --- ";
					out << s << std::endl;
				}
				static void read(std::istream &in, std::string &s)
				{
					std::string lin;
					std::getline(in, lin);
					size_t p = lin.find("---");
					s = lin.substr(p + 3);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(s);
				}
			};
			struct simpleStringRev
			{
				static void write(std::ostream &out, size_t, const std::string &s, const std::string &p)
				{
					out << " " << s << " --- ";
					out << p << std::endl;
				}
				static void read(std::istream &in, std::string &s)
				{
					std::string lin;
					std::getline(in, lin);
					size_t p = lin.find("---");
					s = lin.substr(0, p - 1);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(s);
				}
			};
			
			template <class T>
			struct simpleNumRev
			{
				static void write(std::ostream &out, size_t, const T &s, const std::string &p, size_t pwd = 2, size_t wd = 10)
				{
					std::string sp(' ', pwd);
					out << sp;
					out.width(wd);
					out << std::left << s << " = ";
					out << p << std::endl;
				}
				static void read(std::istream &in, T &s)
				{
					std::string lin;
					std::getline(in, lin);
					size_t p = lin.find("=");
					std::string ss;
					ss = lin.substr(0, p - 1);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(ss);
					using namespace rtmath::macros;
					s = fastCast<T>(ss);
				}
			};

			template <class T>
			struct simpleNumCompound
			{
				static void write(std::ostream &out, size_t, const T &val, size_t wd, 
					const std::string &pre, const std::string &post)
				{
					out << pre;
					out.width(wd);
					out << std::right << val << " = ";
					out << p << std::endl;
				}
				static void read(std::istream &in, T &s)
				{
					std::string lin;
					std::getline(in, lin);
					size_t p = lin.find("=");
					size_t pend = lin.find("=", p + 1);
					std::string ss;
					ss = lin.substr(p+1, pend-p);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(ss);
					using namespace rtmath::macros;
					s = fastCast<T>(ss);
				}
			};
		}


		void ddOriData::writeAVG(std::ostream &out) const
		{
			// Write the file in the appropriate order
			using namespace std;
			using namespace ddOriDataParsers;
			using namespace ddOutput::stat_entries;
			const auto &od = _parent.oridata_d.at(_row);
			const auto &os = _parent.oridata_s.at(_row);
			const auto &oi = _parent.oridata_i.at(_row);
			version::write(out, this->version());
			simpleString::write(out, this->version(), os.at(TARGET), "TARGET");
			simpleStringRev::write(out, this->version(), os.at(CCGMETH), "DDA method");
			simpleStringRev::write(out, this->version(), os.at(DDAMETH), "CCG method");
			simpleStringRev::write(out, this->version(), os.at(SHAPE), "shape");
			simpleNumRev<size_t>::write(out, this->version(), oi.at(NUM_DIPOLES), "NAT0 = number of dipoles");
			double daeff = od.at(D) / od.at(AEFF);
			simpleNumRev<double>::write(out, this->version(), daeff, "d/aeff for this target [d=dipole spacing]");
			simpleNumRev<double>::write(out, this->version(), od.at(D), "d (physical units)");

			simpleNumCompound<double>::write(out, this->version(), od.at(AEFF), 12, "  AEFF=  ", "effective radius (physical units)");
			simpleNumCompound<double>::write(out, this->version(), od.at(WAVE), 12, "  WAVE=  ", "wavelength (in vacuo, physical units)");
			double kaeff = 2. * boost::math::constants::pi<double>() * od.at(AEFF) / od.at(WAVE);
			simpleNumCompound<double>::write(out, this->version(), kaeff, 12, "K*AEFF=  ", "2*pi*aeff/lambda");
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				simpleNumCompound<double>::write(out, this->version(), od.at(NAMBIENT), 8, "NAMBIENT=    ", "refractive index of ambient medium");

			// Write refractive indices (plural)
			WRITE(neps);

			simpleNumCompound<double>::write(out, this->version(), od.at(TOL), 9, "   TOL= ", " error tolerance for CCG method");

			WRITE(a1tgt);
			WRITE(a2tgt);

			simpleNumCompound<size_t>::write(out, this->version(), oi.at(NAVG), 5, "  NAVG= ", "(theta,phi) values used in comp. of Qsca,g");

			WRITE(kveclf);
			WRITE(incpol1lf);
			WRITE(incpol2lf);

			WRITE(betarange);
			WRITE(thetarange);
			WRITE(phirange);

			out << endl;

			simpleNumRev<double>::write(out, this->version(), od.at(ETASCA), 
				"ETASCA = param. controlling # of scatt. dirs used to calculate <cos> etc.", 1, 6);
			
			WRITE(avgnumori);
			WRITE(avgnumpol);

			// Write the odd table of Qsca and the others
			writeStatTable(out);

			// Write the P matrix
			writeMueller(out);
		}

		void ddOriData::writeSCA(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			WRITE("version");
			WRITE("target");
			WRITE("solnmeth");
			WRITE("polarizability");
			WRITE("shape");
			WRITE("numdipoles");

			WRITE("d/aeff");
			WRITE("d");
			WRITE("physextent");
			WRITE("xtf");
			WRITE("ytf");
			WRITE("ztf");

			WRITE("aeff");
			WRITE("wave");
			WRITE("k.aeff");
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				WRITE("nambient");
			WRITE("neps");
			WRITE("tol");

			WRITE("a1tgt");
			WRITE("a2tgt");
			WRITE("navg");

			WRITE("kvectf");
			WRITE("incpol1tf");
			WRITE("incpol2tf");
			WRITE("kveclf");
			WRITE("incpol1lf");
			WRITE("incpol2lf");

			WRITE("beta");
			WRITE("theta");
			WRITE("phi");

			WRITE("etasca");

			// Write the odd table of Qsca and the others
			writeStatTable(out);

			// Write the P matrix
			writeMueller(out);
		}

		void ddOriData::writeFML(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			WRITE("version");
			WRITE("target");
			WRITE("solnmeth");
			WRITE("polarizability");
			WRITE("shape");
			WRITE("numdipoles");

			WRITE("aeff");
			WRITE("wave");
			WRITE("k.aeff");
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				WRITE("nambient");
			WRITE("neps");
			WRITE("tol");
			WRITE("navg");
			WRITE("a1tgt");
			WRITE("a2tgt");
			WRITE("kvectf");
			WRITE("incpol1tf");
			WRITE("incpol2tf");
			WRITE("kveclf");
			WRITE("incpol1lf");
			WRITE("incpol2lf");
			WRITE("beta");
			WRITE("theta");
			WRITE("phi");
			WRITE("targetperiodicity");
			WRITE("fmldotline");
			WRITE("mdef");

			out << endl;

			// Write the f matrix
			writeF(out);
		}

	}
}

