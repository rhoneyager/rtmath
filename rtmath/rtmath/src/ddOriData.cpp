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

		ddOriData::ddOriData(ddOutput &parent, size_t row, 
			const std::string &filenameSCA, 
			const std::string &filenameFML) :
			_parent(parent), _row(row)
		{
			_init();
			// Double read to capture fml data and cross-sections in one container.
			if (filenameSCA.size()) readFile(filenameSCA);
			if (filenameFML.size()) readFile(filenameFML);
		}
		
		void ddOriData::_init()
		{
			//_statTable_Size_ts.at(stat_entries_size_ts::VERSION)
			//	= rtmath::ddscat::ddVersions::getDefaultVer();
		}



		void ddOriData::doExportFMLs(size_t startIndex) const
		{
			auto o = _parent.fmldata->block(startIndex, 0, numMat(), ddOutput::fmlColDefs::NUM_FMLCOLDEFS);
			size_t i = 0;
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it).id() != scattMatrixType::F) continue;
				//auto s = boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it);

				auto f = it->getF();

				o(i, ddOutput::fmlColDefs::ORIINDEX) = static_cast<float>(_row);
				o(i, ddOutput::fmlColDefs::THETAB) = static_cast<float>((*it).theta());
				o(i, ddOutput::fmlColDefs::PHIB) = static_cast<float>((*it).phi());
				o(i, ddOutput::fmlColDefs::F00R) = static_cast<float>(f(0, 0).real());
				o(i, ddOutput::fmlColDefs::F00I) = static_cast<float>(f(0, 0).imag());
				o(i, ddOutput::fmlColDefs::F01R) = static_cast<float>(f(0, 1).real());
				o(i, ddOutput::fmlColDefs::F01I) = static_cast<float>(f(0, 1).imag());
				o(i, ddOutput::fmlColDefs::F10R) = static_cast<float>(f(1, 0).real());
				o(i, ddOutput::fmlColDefs::F10I) = static_cast<float>(f(1, 0).imag());
				o(i, ddOutput::fmlColDefs::F11R) = static_cast<float>(f(1, 1).real());
				o(i, ddOutput::fmlColDefs::F11I) = static_cast<float>(f(1, 1).imag());

				i++;
			}
		}

		void ddOriData::doImportFMLs()
		{
			size_t startRow = 0;
			size_t stopRow = 0;
			bool startFound = false;
			bool endFound = false;
			for (size_t i = 0; i < _parent.fmldata->rows(), ++i)
			{
				size_t oriindex = static_cast<size_t>((*(parent.fmldata))(i, fmlColDefs::ORIINDEX));
				if (!startFound && oriindex == _row)
				{
					startFound = true;
					startRow = oriindex;
				}
				if (startFound && oriindex != row)
				{
					endFound = true;
					stopRow = oriindex;
					break;
				}
			}
			if (!startFound) return;
			if (!endFound) stopRow = _parent.fmldata->rows();
			if (stopRow <= startRow) RTthrow debug::xArrayOutOfBounds();

			doImportFMLs(startRow, stopRow - startRow);
		}

		void ddOriData::doImportFMLs(size_t startIndex, size_t n)
		{
			auto o = _parent.fmldata->block(startIndex, 0, n, ddOutput::fmlColDefs::NUM_FMLCOLDEFS);
			size_t i = 0;
			_scattMatricesRaw.clear();
			_scattMatricesRaw.reserve(n);
			using std::complex;
			for (i = 0; i < n; ++i)
			{
				ddScattMatrixF mat(freq(), o(i, ddOutput::fmlColDefs::THETAB), o(i, ddOutput::fmlColDefs::PHIB), 
					0, 0, getConnector());
				ddScattMatrix::FType fs;
				fs(0, 0) = complex<double>(o(i, ddOutput::fmlColDefs::F00R), o(i, ddOutput::fmlColDefs::F00I));
				fs(1, 0) = complex<double>(o(i, ddOutput::fmlColDefs::F01R), o(i, ddOutput::fmlColDefs::F01I));
				fs(0, 1) = complex<double>(o(i, ddOutput::fmlColDefs::F10R), o(i, ddOutput::fmlColDefs::F10I));
				fs(1, 1) = complex<double>(o(i, ddOutput::fmlColDefs::F11R), o(i, ddOutput::fmlColDefs::F11I));
				mat.setF(fs);

				//boost::shared_ptr<const ddScattMatrix> matC =
				//	boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

				//_scattMatricesRaw.push_back(matC);
				_scattMatricesRaw.push_back(mat);
			}
		}



		ddOriData::~ddOriData() {}

		/// Input in avg format
		void ddOriData::readAVG(std::istream &in)
		{
			using namespace std;
			using namespace ddOriDataParsers;

			auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			auto &os = _parent.oridata_s.at(_row);
			auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);
			std::string junk;

			oi(ddOutput::stat_entries::VERSION) = version::read(in, this->version());
			simpleString::read(in, os[ddOutput::stat_entries::TARGET]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::DDAMETH]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::CCGMETH]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::SHAPE]);
			simpleNumRev<size_t>::read(in, oi(ddOutput::stat_entries::NUM_DIPOLES));
			std::getline(in, junk); // d/aeff
			simpleNumRev<double>::read(in, od(ddOutput::stat_entries::D));
			
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::AEFF));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::WAVE));

			od(ddOutput::stat_entries::FREQ) = units::conv_spec("um", "GHz").convert(od(ddOutput::stat_entries::WAVE));

			std::getline(in, junk); // k*aeff
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::NAMBIENT));

			// Read refractive indices (plural)
			std::string lin; // Used for peeking ahead
			for (size_t i = 0;; ++i)
			{
				std::getline(in, lin);
				if (lin.at(0) != 'n') break; // No more refractive indices
				size_t subst = 0;
				std::complex<double> m;
				refractive::read(lin, subst, m);
				ms.push_back(m);
			}

			simpleNumCompound<double>::read(lin, od(ddOutput::stat_entries::TOL)); // lin from refractive index read

			std::vector<double> a(3);
			size_t axisnum = 0;
			frameType frm = frameType::TF;
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TA1TFX) = a[0]; od(ddOutput::stat_entries::TA1TFY) = a[1]; od(ddOutput::stat_entries::TA1TFZ) = a[2];
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TA2TFX) = a[0]; od(ddOutput::stat_entries::TA2TFY) = a[1]; od(ddOutput::stat_entries::TA2TFZ) = a[2];

			simpleNumCompound<size_t>::read(in, oi(ddOutput::stat_entries::NAVG));

			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TFKX) = a[0]; od(ddOutput::stat_entries::TFKY) = a[1]; od(ddOutput::stat_entries::TFKZ) = a[2];


			std::vector<std::complex<double> > iv(3);
			size_t vecnum = 0;
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV1TFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV1TFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV1TFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV1TFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV1TFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV1TFZI) = iv[2].imag();
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV2TFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV2TFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV2TFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV2TFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV2TFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV2TFZI) = iv[2].imag();

			std::getline(in, junk); // beta extent
			std::getline(in, junk); // theta extent
			std::getline(in, junk); // phi extent
			std::getline(in, junk); // empty line

			simpleNumRev<double>::read(in, od(ddOutput::stat_entries::ETASCA));
			std::getline(in, junk); // num orientations
			std::getline(in, junk); // num polarizations

			std::getline(in, junk); //"          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;

			readStatTable(in);
			//readMueller(in);
		}

		/// Input in sca format
		void ddOriData::readSCA(std::istream &in)
		{
			using namespace std;
			using namespace ddOriDataParsers;

			auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			auto &os = _parent.oridata_s.at(_row);
			auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);
			std::string junk;

			oi(ddOutput::stat_entries::VERSION) = version::read(in, this->version());
			simpleString::read(in, os[ddOutput::stat_entries::TARGET]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::DDAMETH]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::CCGMETH]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::SHAPE]);
			simpleNumRev<size_t>::read(in, oi(ddOutput::stat_entries::NUM_DIPOLES));
			std::getline(in, junk); // d/aeff
			simpleNumRev<double>::read(in, od(ddOutput::stat_entries::D));
			std::getline(in, junk); // physical extent
			ddPhysExtent::read(in, od(ddOutput::stat_entries::XMIN), od(ddOutput::stat_entries::XMAX), junk[0]);
			ddPhysExtent::read(in, od(ddOutput::stat_entries::YMIN), od(ddOutput::stat_entries::YMAX), junk[0]);
			ddPhysExtent::read(in, od(ddOutput::stat_entries::ZMIN), od(ddOutput::stat_entries::ZMAX), junk[0]);

			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::AEFF));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::WAVE));
			od(ddOutput::stat_entries::FREQ) = units::conv_spec("um", "GHz").convert(od(ddOutput::stat_entries::WAVE));

			std::getline(in, junk); // k*aeff
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::NAMBIENT));

			// Read refractive indices (plural)
			std::string lin; // Used for peeking ahead
			for (size_t i = 0;; ++i)
			{
				std::getline(in, lin);
				if (lin.at(0) != 'n') break; // No more refractive indices
				size_t subst = 0;
				std::complex<double> m;
				refractive::read(lin, subst, m);
				ms.push_back(m);
			}

			simpleNumCompound<double>::read(lin, od(ddOutput::stat_entries::TOL)); // lin from refractive index read

			std::vector<double> a(3);
			size_t axisnum = 0;
			frameType frm = frameType::TF;
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TA1TFX) = a[0]; od(ddOutput::stat_entries::TA1TFY) = a[1]; od(ddOutput::stat_entries::TA1TFZ) = a[2];
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TA2TFX) = a[0]; od(ddOutput::stat_entries::TA2TFY) = a[1]; od(ddOutput::stat_entries::TA2TFZ) = a[2];

			simpleNumCompound<size_t>::read(in, oi(ddOutput::stat_entries::NAVG));

			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TFKX) = a[0]; od(ddOutput::stat_entries::TFKY) = a[1]; od(ddOutput::stat_entries::TFKZ) = a[2];


			std::vector<std::complex<double> > iv(3);
			size_t vecnum = 0;
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV1TFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV1TFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV1TFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV1TFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV1TFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV1TFZI) = iv[2].imag();
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV2TFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV2TFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV2TFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV2TFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV2TFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV2TFZI) = iv[2].imag();

			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::LFKX) = a[0]; od(ddOutput::stat_entries::LFKY) = a[1]; od(ddOutput::stat_entries::LFKZ) = a[2];

			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV1LFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV1LFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV1LFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV1LFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV1LFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV1LFZI) = iv[2].imag();
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV2LFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV2LFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV2LFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV2LFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV2LFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV2LFZI) = iv[2].imag();

			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::BETA));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::THETA));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::PHI));

			simpleNumRev<double>::read(in, od(ddOutput::stat_entries::ETASCA));

			std::getline(in, junk); //"          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;

			readStatTable(in);
			//readMueller(in);
		}

		/// Input in fml format
		void ddOriData::readFML(std::istream &in)
		{
			using namespace std;
			using namespace ddOriDataParsers;

			auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			auto &os = _parent.oridata_s.at(_row);
			auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);
			std::string junk;

			oi(ddOutput::stat_entries::VERSION) = version::read(in, this->version());
			simpleString::read(in, os[ddOutput::stat_entries::TARGET]);
			simpleStringRev::read(in,os[ddOutput::stat_entries::DDAMETH]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::CCGMETH]);
			simpleStringRev::read(in, os[ddOutput::stat_entries::SHAPE]);
			simpleNumRev<size_t>::read(in, oi(ddOutput::stat_entries::NUM_DIPOLES));
			std::getline(in, junk); // d/aeff
			simpleNumRev<double>::read(in, od(ddOutput::stat_entries::D));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::AEFF));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::WAVE));
			od(ddOutput::stat_entries::FREQ) = units::conv_spec("um", "GHz").convert(od(ddOutput::stat_entries::WAVE));

			std::getline(in, junk); // k*aeff
			if (rtmath::ddscat::ddVersions::isVerWithin(version(), 72, 0))
				simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::NAMBIENT));

			// Read refractive indices (plural)
			std::string lin; // Used for peeking ahead
			for (size_t i=0;;++i)
			{
				std::getline(in, lin);
				if (lin.at(0) != 'n') break; // No more refractive indices
				size_t subst = 0;
				std::complex<double> m;
				refractive::read(lin, subst, m);
				ms.push_back(m);
			}

			simpleNumCompound<double>::read(lin, od(ddOutput::stat_entries::TOL)); // lin from refractive index read
			simpleNumCompound<size_t>::read(in, oi(ddOutput::stat_entries::NAVG));

			std::vector<double> a(3);
			size_t axisnum = 0;
			frameType frm = frameType::TF;
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TA1TFX) = a[0]; od(ddOutput::stat_entries::TA1TFY) = a[1]; od(ddOutput::stat_entries::TA1TFZ) = a[2];
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TA2TFX) = a[0]; od(ddOutput::stat_entries::TA2TFY) = a[1]; od(ddOutput::stat_entries::TA2TFZ) = a[2];
			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::TFKX) = a[0]; od(ddOutput::stat_entries::TFKY) = a[1]; od(ddOutput::stat_entries::TFKZ) = a[2];


			std::vector<std::complex<double> > iv(3);
			size_t vecnum = 0;
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV1TFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV1TFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV1TFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV1TFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV1TFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV1TFZI) = iv[2].imag();
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV2TFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV2TFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV2TFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV2TFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV2TFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV2TFZI) = iv[2].imag();

			ddAxisVec::read(in, a, axisnum, frm);
			od(ddOutput::stat_entries::LFKX) = a[0]; od(ddOutput::stat_entries::LFKY) = a[1]; od(ddOutput::stat_entries::LFKZ) = a[2];

			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV1LFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV1LFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV1LFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV1LFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV1LFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV1LFZI) = iv[2].imag();
			ddPolVec::read(in, iv, vecnum, frm);
			od(ddOutput::stat_entries::IPV2LFXR) = iv[0].real(); od(ddOutput::stat_entries::IPV2LFXI) = iv[0].imag();
			od(ddOutput::stat_entries::IPV2LFYR) = iv[1].real(); od(ddOutput::stat_entries::IPV2LFYI) = iv[1].imag();
			od(ddOutput::stat_entries::IPV2LFZR) = iv[2].real(); od(ddOutput::stat_entries::IPV2LFZI) = iv[2].imag();

			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::BETA));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::THETA));
			simpleNumCompound<double>::read(in, od(ddOutput::stat_entries::PHI));


			std::getline(in, junk); // "     Finite target:\n"
			std::getline(in, junk); // "     e_m dot E(r) = i*exp(ikr)*f_ml*E_inc(0)/(kr)\n"
			std::getline(in, junk); // "     m=1 in scatt. plane, m=2 perp to scatt. plane\n";
			std::getline(in, junk); //out << endl;

			std::getline(in, junk); //readHeader(in, "Re(f_11)");
			// Get e1 and e2 in lab frame from the header data
			auto cn = getConnector();
			_scattMatricesRaw.clear();
			_scattMatricesRaw.reserve(40);
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
			ddAxisVec::write(out, this->version(), a, 0, frameType::LF);

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
			// Write the file in the appropriate order
			using namespace std;
			using namespace ddOriDataParsers;
			//using namespace ddOutput::stat_entries;
			const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
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

			out << "----- physical extent of target volume in Target Frame ------\n";
			ddPhysExtent::write(out, this->version(), od(ddOutput::stat_entries::XMIN), od(ddOutput::stat_entries::XMAX), 'x');
			ddPhysExtent::write(out, this->version(), od(ddOutput::stat_entries::YMIN), od(ddOutput::stat_entries::YMAX), 'y');
			ddPhysExtent::write(out, this->version(), od(ddOutput::stat_entries::ZMIN), od(ddOutput::stat_entries::ZMAX), 'z');

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


			a[0] = od(ddOutput::stat_entries::TFKX); a[1] = od(ddOutput::stat_entries::TFKY); a[2] = od(ddOutput::stat_entries::TFKZ);
			ddAxisVec::write(out, this->version(), a, 0, frameType::TF);

			std::vector<std::complex<double> > iv(3);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV1TFXR), od(ddOutput::stat_entries::IPV1TFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV1TFYR), od(ddOutput::stat_entries::IPV1TFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV1TFZR), od(ddOutput::stat_entries::IPV1TFZI));
			ddPolVec::write(out, this->version(), iv, 1, frameType::TF);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV2TFXR), od(ddOutput::stat_entries::IPV2TFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV2TFYR), od(ddOutput::stat_entries::IPV2TFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV2TFZR), od(ddOutput::stat_entries::IPV2TFZI));
			ddPolVec::write(out, this->version(), iv, 2, frameType::TF);


			a[0] = od(ddOutput::stat_entries::LFKX); a[1] = od(ddOutput::stat_entries::LFKY); a[2] = od(ddOutput::stat_entries::LFKZ);
			ddAxisVec::write(out, this->version(), a, 0, frameType::LF);

			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFXR), od(ddOutput::stat_entries::IPV1LFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFYR), od(ddOutput::stat_entries::IPV1LFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFZR), od(ddOutput::stat_entries::IPV1LFZI));
			ddPolVec::write(out, this->version(), iv, 1, frameType::LF);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFXR), od(ddOutput::stat_entries::IPV2LFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFYR), od(ddOutput::stat_entries::IPV2LFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFZR), od(ddOutput::stat_entries::IPV2LFZI));
			ddPolVec::write(out, this->version(), iv, 2, frameType::LF);

			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::BETA), 7, " BETA =", "rotation of target around A1");
			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::THETA), 7, " THETA=", "angle between A1 and k");
			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::PHI), 7, "  PHI =", "rotation of A1 around k");

			out << endl;

			simpleNumRev<double>::write(out, this->version(), od(ddOutput::stat_entries::ETASCA),
				"ETASCA = param. controlling # of scatt. dirs used to calculate <cos> etc.", 1, 6);

			//out << " Results averaged over " << 0 << " target orientations\n"
			//	<< "                   and    2 incident polarizations\n";


			// Write the odd table of Qsca and the others
			writeStatTable(out);

			// Write the P matrix
			writeMueller(out);
		}

		void ddOriData::writeFML(std::ostream &out) const
		{
			// Write the file in the appropriate order
			using namespace std;
			using namespace ddOriDataParsers;
			//using namespace ddOutput::stat_entries;
			const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			const auto &os = _parent.oridata_s.at(_row);
			const auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);
			version::write(out, this->version());
			simpleString::write(out, this->version(), os.at(ddOutput::stat_entries::TARGET), "TARGET");
			simpleStringRev::write(out, this->version(), os.at(ddOutput::stat_entries::DDAMETH), "CCG method");
			simpleStringRev::write(out, this->version(), os.at(ddOutput::stat_entries::CCGMETH), "DDA method");
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


			simpleNumCompound<size_t>::write(out, this->version(), oi(ddOutput::stat_entries::NAVG), 5, "  NAVG= ", "(theta,phi) values used in comp. of Qsca,g");


			std::vector<double> a(3);
			a[0] = od(ddOutput::stat_entries::TA1TFX); a[1] = od(ddOutput::stat_entries::TA1TFY); a[2] = od(ddOutput::stat_entries::TA1TFZ);
			ddAxisVec::write(out, this->version(), a, 1, frameType::TF);
			a[0] = od(ddOutput::stat_entries::TA2TFX); a[1] = od(ddOutput::stat_entries::TA2TFY); a[2] = od(ddOutput::stat_entries::TA2TFZ);
			ddAxisVec::write(out, this->version(), a, 2, frameType::TF);

			a[0] = od(ddOutput::stat_entries::TFKX); a[1] = od(ddOutput::stat_entries::TFKY); a[2] = od(ddOutput::stat_entries::TFKZ);
			ddAxisVec::write(out, this->version(), a, 0, frameType::TF);

			std::vector<std::complex<double> > iv(3);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV1TFXR), od(ddOutput::stat_entries::IPV1TFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV1TFYR), od(ddOutput::stat_entries::IPV1TFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV1TFZR), od(ddOutput::stat_entries::IPV1TFZI));
			ddPolVec::write(out, this->version(), iv, 1, frameType::TF);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV2TFXR), od(ddOutput::stat_entries::IPV2TFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV2TFYR), od(ddOutput::stat_entries::IPV2TFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV2TFZR), od(ddOutput::stat_entries::IPV2TFZI));
			ddPolVec::write(out, this->version(), iv, 2, frameType::TF);


			a[0] = od(ddOutput::stat_entries::LFKX); a[1] = od(ddOutput::stat_entries::LFKY); a[2] = od(ddOutput::stat_entries::LFKZ);
			ddAxisVec::write(out, this->version(), a, 0, frameType::LF);

			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFXR), od(ddOutput::stat_entries::IPV1LFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFYR), od(ddOutput::stat_entries::IPV1LFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV1LFZR), od(ddOutput::stat_entries::IPV1LFZI));
			ddPolVec::write(out, this->version(), iv, 1, frameType::LF);
			iv[0] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFXR), od(ddOutput::stat_entries::IPV2LFXI));
			iv[1] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFYR), od(ddOutput::stat_entries::IPV2LFYI));
			iv[2] = std::complex<double>(od(ddOutput::stat_entries::IPV2LFZR), od(ddOutput::stat_entries::IPV2LFZI));
			ddPolVec::write(out, this->version(), iv, 2, frameType::LF);


			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::BETA), 7, " BETA =", "rotation of target around A1");
			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::THETA), 7, " THETA=", "angle between A1 and k");
			simpleNumCompound<double>::write(out, this->version(), od(ddOutput::stat_entries::PHI), 7, "  PHI =", "rotation of A1 around k");


			out << "     Finite target:\n"
				"     e_m dot E(r) = i*exp(ikr)*f_ml*E_inc(0)/(kr)\n"
				"     m=1 in scatt. plane, m=2 perp to scatt. plane\n";
			out << endl;

			// Write the f matrix
			writeF(out);
		}

		boost::shared_ptr<const ddScattMatrixConnector> ddOriData::getConnector() const
		{
			if (!_connector)
			{
				const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
				std::vector<std::complex<double> > v;
				v.push_back(std::complex<double>(od(ddOutput::stat_entries::IPV1LFXR), od(ddOutput::stat_entries::IPV1LFXI)));
				v.push_back(std::complex<double>(od(ddOutput::stat_entries::IPV1LFYR), od(ddOutput::stat_entries::IPV1LFYI)));
				v.push_back(std::complex<double>(od(ddOutput::stat_entries::IPV1LFZR), od(ddOutput::stat_entries::IPV1LFZI)));
				v.push_back(std::complex<double>(od(ddOutput::stat_entries::IPV2LFXR), od(ddOutput::stat_entries::IPV2LFXI)));
				v.push_back(std::complex<double>(od(ddOutput::stat_entries::IPV2LFYR), od(ddOutput::stat_entries::IPV2LFYI)));
				v.push_back(std::complex<double>(od(ddOutput::stat_entries::IPV2LFZR), od(ddOutput::stat_entries::IPV2LFZI)));
				_connector = ddScattMatrixConnector::fromVector(v);
			}
			return _connector;
		}

		void ddOriData::setConnector(boost::shared_ptr<const ddScattMatrixConnector> cn)
		{
			_connector = cn;
		}


		void ddOriData::writeStatTable(std::ostream &out) const
		{
			const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			const auto &os = _parent.oridata_s.at(_row);
			const auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);
			using namespace std;
			out << "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			out << " JO=1: ";
			out.width(11);
			for (size_t i = (size_t)ddOutput::stat_entries::QEXT1; i< (size_t)ddOutput::stat_entries::QEXT2; i++)
				out << "\t" << od(i);
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i = (size_t)ddOutput::stat_entries::QEXT2; i< (size_t)ddOutput::stat_entries::QEXTM; i++)
				out << "\t" << od(i);
			out << endl;
			out.width(0);
			out << " mean: ";
			out.width(11);
			for (size_t i = (size_t)ddOutput::stat_entries::QEXTM; i< (size_t)ddOutput::stat_entries::QPOL; i++)
				out << "\t" << od(i);
			out << endl;
			out.width(0);
			out << " Qpol= " << od(ddOutput::stat_entries::QPOL) <<
				"                                                  " <<
				"dQpha= ";
			out.width(11);
			out << od(ddOutput::stat_entries::DQPHA) << endl;

			out << "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca\n";
			out << " JO=1: ";
			out.width(11);
			for (size_t i = (size_t)ddOutput::stat_entries::QSCAG11; i< (size_t)ddOutput::stat_entries::QSCAG12; i++)
				out << "\t" << od(i);
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i = (size_t)ddOutput::stat_entries::QSCAG12; i< (size_t)ddOutput::stat_entries::QSCAG1M; i++)
				out << "\t" << od(i);
			out << endl;
			out.width(0);
			out << " mean: ";
			for (size_t i = (size_t)ddOutput::stat_entries::QSCAG1M; i< (size_t)ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES; i++)
				out << "\t" << od(i);
			out << endl;
			out.width(0);
		}


		void ddOriData::readStatTable(std::istream &in)
		{
			auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			auto &os = _parent.oridata_s.at(_row);
			auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);

			using namespace std;
			string line;
			// First line was detected by the calling function, so it is outside of the istream now.
			//std::getline(in,line); // "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			in >> line; // " JO=1: ";
			for (size_t i = ddOutput::stat_entries::QEXT1; i < (size_t)ddOutput::stat_entries::QEXT2; i++)
				in >> od(i);
			in >> line; // " JO=2: ";
			for (size_t i = (size_t)ddOutput::stat_entries::QEXT2; i< (size_t)ddOutput::stat_entries::QEXTM; i++)
				in >> od(i);
			in >> line; // " mean: ";
			for (size_t i = (size_t)ddOutput::stat_entries::QEXTM; i< (size_t)ddOutput::stat_entries::QPOL; i++)
				in >> od(i);
			in >> line // " Qpol= " 
				>> od(ddOutput::stat_entries::QPOL) >> line; // "dQpha=";
			in >> od(ddOutput::stat_entries::DQPHA);

			std::getline(in, line); // "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca";
			std::getline(in, line);
			in >> line; // " JO=1: ";
			for (size_t i = (size_t)ddOutput::stat_entries::QSCAG11; i< (size_t)ddOutput::stat_entries::QSCAG12; i++)
				in >> od(i);
			in >> line; // " JO=2: ";
			for (size_t i = (size_t)ddOutput::stat_entries::QSCAG12; i< (size_t)ddOutput::stat_entries::QSCAG1M; i++)
				in >> od(i);
			in >> line; // " mean: ";
			for (size_t i = (size_t)ddOutput::stat_entries::QSCAG1M; i< (size_t)ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES; i++)
				in >> od(i);
			std::getline(in, line);
		}

		double ddOriData::guessTemp(size_t dielIndex) const
		{
			const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			return rtmath::refract::guessTemp(od(ddOutput::stat_entries::FREQ), M(dielIndex));
		}

		void ddOriData::writeF(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(f_11)   Im(f_11)   Re(f_21)   Im(f_21)   Re(f_12)   Im(f_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it).id() != scattMatrixType::F) continue;
				//boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
				//	boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it));
				out << endl;
				out.width(6);
				// it->first crds ordering is freq, phi, theta
				out << (*it).theta() << "\t";
				out << (*it).phi();
				out.width(11);
				ddScattMatrix::FType f = it->getF();

				for (size_t j = 0; j < 2; j++)
					for (size_t i = 0; i < 2; i++)
					{
					// Note the reversed coordinates. This matches ddscat.
					out << "\t" << f(i, j).real();
					out << "\t" << f(i, j).imag();
					}
			}
		}

		void ddOriData::writeS(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(S_11)   Im(S_11)   Re(S_21)   Im(S_21)   Re(S_12)   Im(S_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it).id() != scattMatrixType::F) continue;
				//boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
				//	boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it));
				out << endl;
				out.width(6);
				// it->first crds ordering is freq, phi, theta
				out << (*it).theta() << "\t";
				out << (*it).phi();
				out.width(11);
				ddScattMatrix::FType s = it->getS();
				for (size_t j = 0; j < 2; j++)
					for (size_t i = 0; i < 2; i++)
					{
					// Note the reversed coordinates. This matches ddscat.
					out << "\t" << s(i, j).real();
					out << "\t" << s(i, j).imag();
					}
			}
		}

		/// Provides default Mueller matrix entries to write.
		const ddOriData::mMuellerIndices& ddOriData::mMuellerIndicesDefault()
		{
			static mMuellerIndices mi;
			/// \todo Make threadable
			if (!mi.size())
			{
				mi.push_back(std::pair<size_t, size_t>(0, 0));
				mi.push_back(std::pair<size_t, size_t>(0, 1));
				mi.push_back(std::pair<size_t, size_t>(1, 0));
				mi.push_back(std::pair<size_t, size_t>(1, 1));
				mi.push_back(std::pair<size_t, size_t>(2, 0));
				mi.push_back(std::pair<size_t, size_t>(3, 0));
			}
			return mi;
		}

		void ddOriData::writeMueller(std::ostream &out, const mMuellerIndices &mi) const
		{
			using namespace std;
			out << "            Mueller matrix elements for selected scattering directions in Lab Frame" << endl;
			out << " theta    phi    Pol.    "; // "S_11        S_12        S_21       S_22       S_31       S_41\n";
			for (auto it = mi.begin(); it != mi.end(); ++it)
			{
				out << "S_" << (it->first + 1) << (it->second + 1);
				auto ot = it;
				ot++;
				if (ot != mi.end()) out << "        ";
			}
			out << "\n";

			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				//boost::shared_ptr<const ddscat::ddScattMatrix> sf(*it);
				out << endl;
				//out.width(6);
				using namespace std;
				//out.width(6);
				out << fixed << right << showpoint << setprecision(2) << setw(6) << (*it).theta() << " ";
				out << setw(6) << (*it).phi() << " ";
				//out.width(8);
				out << setprecision(5) << setw(8) << it->polLin() << " ";
				//out.width(10);
				ddScattMatrix::PnnType p = it->mueller();
				for (auto ot = mi.begin(); ot != mi.end(); ++ot)
				{
					out << " " << scientific << setprecision(4) << setw(10) << p(ot->first, ot->second);
				}
			}
		}

		size_t ddOriData::numMat() const { return _scattMatricesRaw.size(); }

		bool ddOriData::operator<(const ddOriData &rhs) const
		{
			const auto &od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			const auto &os = _parent.oridata_s.at(_row);
			const auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);

			const auto &rod = rhs._parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(rhs._row, 0);
			const auto &ros = rhs._parent.oridata_s.at(rhs._row);
			const auto &roi = rhs._parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(rhs._row, 0);

			
#define CHECKD(x) if( od(x) != rod(x)) return od(x) < rod(x);
#define CHECKI(x) if( oi(x) != roi(x)) return oi(x) < roi(x);
			CHECKI(ddOutput::stat_entries::DOWEIGHT);
			CHECKD(ddOutput::stat_entries::FREQ);
			CHECKD(ddOutput::stat_entries::AEFF);
			CHECKI(ddOutput::stat_entries::NUM_DIPOLES);
			CHECKD(ddOutput::stat_entries::BETA);
			CHECKD(ddOutput::stat_entries::THETA);
			CHECKD(ddOutput::stat_entries::PHI);
#undef CHECKD
#undef CHECKI

			return false;
		}

		std::complex<double> ddOriData::M(size_t dielIndex) const
		{
			if (ms.size() > dielIndex) return ms[dielIndex];
			RTthrow debug::xArrayOutOfBounds();
			return std::complex<double>(0, 0); // needed to suppress msvc warning
		}
		void ddOriData::M(const std::complex<double>& m, size_t dielIndex)
		{
			if (ms.size() < dielIndex) ms.resize(dielIndex + 1);
			ms[dielIndex] = m;
		}

		size_t ddOriData::numM() const
		{
			return ms.size();
		}



	}
}

