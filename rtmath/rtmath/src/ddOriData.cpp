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
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/refract.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"

#include "ddOriDataParsers.h"

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
			ddOriData_IO_input_registry, ddOriData_Standard>(ddOriData::writeDDSCAT, ddOriData::readDDSCAT, known_formats())
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


		void ddOriData::writeAVG(std::ostream &out) const
		{
			// Write the file in the appropriate order
			using namespace std;
			using namespace ddOriDataParsers;
			//using namespace ddOutput::stat_entries;
			const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row,0);
			const auto &os = _parent.oridata_s.at(_row);
			const auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);
			version::write(out, this->version());
			simpleString::write(out, this->version(), os.at(ddOutput::stat_entries::TARGET), "TARGET");
			simpleStringRev::write(out, this->version(), os.at(ddOutput::stat_entries::CCGMETH), "DDA method");
			simpleStringRev::write(out, this->version(), os.at(ddOutput::stat_entries::DDAMETH), "CCG method");
			simpleStringRev::write(out, this->version(), os.at(ddOutput::stat_entries::SHAPE), "shape");
			simpleNumRev<size_t>::write(out, this->version(), oi(ddOutput::stat_entries::NUM_DIPOLES), "NAT0 = number of dipoles");
			double daeff = od(ddOutput::stat_entries::D) / od(ddOutput::stat_entries::AEFF);
			simpleNumRev<double>::write(out, this->version(), daeff, "d/aeff for this target [d=dipole spacing]");
			simpleNumRev<double>::write(out, this->version(), od(ddOutput::stat_entries::D), "d (physical units)");

			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::AEFF), 12, "  AEFF=  ", "effective radius (physical units)");
			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::WAVE), 12, "  WAVE=  ", "wavelength (in vacuo, physical units)");
			double k = 2. * boost::math::constants::pi<double>() / od(ddOutput::stat_entries::WAVE);
			double kaeff = 2. * boost::math::constants::pi<double>() * od(ddOutput::stat_entries::AEFF) / od(ddOutput::stat_entries::WAVE);
			simpleNumCompound<double>::write(out, this->version(), kaeff, 12, "K*AEFF=  ", "2*pi*aeff/lambda");
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::NAMBIENT), 8, "NAMBIENT=    ", "refractive index of ambient medium");

			// Write refractive indices (plural)
			for (size_t i = 0; i < ms.size(); ++i)
				refractive::write(out, this->version(), i + 1, ms[i], k, od(ddOutput::stat_entries::D));

			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::TOL), 9, "   TOL= ", " error tolerance for CCG method");
			
			std::vector<double> a(3);
			a[0] = od(ddOutput::stat_entries::TA1TFX); a[1] = od(ddOutput::stat_entries::TA1TFY); a[2] = od(ddOutput::stat_entries::TA1TFZ);
			ddAxisVec::write(out, this->version(), a, 1, frameType::TF);
			a[0] = od(ddOutput::stat_entries::TA2TFX); a[1] = od(ddOutput::stat_entries::TA2TFY); a[2] = od(ddOutput::stat_entries::TA2TFZ);
			ddAxisVec::write(out, this->version(), a, 2, frameType::TF);
			

			simpleNumCompound<size_t>::write(out, this->version(), oi(ddOutput::stat_entries::NAVG), 5, "  NAVG= ", "(theta,phi) values used in comp. of Qsca,g");

			a[0] = od(ddOutput::stat_entries::LFKX); a[1] = od(ddOutput::stat_entries::LFKY); a[2] = od(ddOutput::stat_entries::LFKZ);
			ddAxisVec::write(out, this->version(), a, 0, frameType::TF);

			std::vector<std::complex<double> > iv(3);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFXR), od(ddOutput::stat_entries::IPV1LFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFYR), od(ddOutput::stat_entries::IPV1LFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFZR), od(ddOutput::stat_entries::IPV1LFZI));
			ddPolVec::write(out, this->version(), iv, 1, frameType::LF);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFXR), od(ddOutput::stat_entries::IPV2LFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFYR), od(ddOutput::stat_entries::IPV2LFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFZR), od(ddOutput::stat_entries::IPV2LFZI));
			ddPolVec::write(out, this->version(), iv, 2, frameType::LF);

			ddRot1d::write(out, this->version(), "beta", 0, 360, 0, "BETA");
			ddRot1d::write(out, this->version(), "theta", 0, 180, 0, "THETA");
			ddRot1d::write(out, this->version(), "phi", 0, 360, 0, "PHI");

			out << endl;

			simpleNumRev<double>::write(out, this->version(), od(ddOutput::stat_entries::ETASCA),
				"ETASCA = param. controlling # of scatt. dirs used to calculate <cos> etc.", 1, 6);
			
			out << " Results averaged over " << 0 << " target orientations\n"
				<< "                   and    2 incident polarizations\n";
			

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

