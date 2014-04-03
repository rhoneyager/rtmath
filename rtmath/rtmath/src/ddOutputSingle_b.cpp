#include "Stdafx-ddscat.h"

#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/units.h"

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer
			<::rtmath::ddscat::ddOutputSingle>;

		template struct IO_class_registry_reader
			<::rtmath::ddscat::ddOutputSingle>;

		template class usesDLLregistry<
			::rtmath::ddscat::ddOutputSingle_IO_output_registry,
			IO_class_registry_writer<::rtmath::ddscat::ddOutputSingle> >;

		template class usesDLLregistry<
			::rtmath::ddscat::ddOutputSingle_IO_input_registry,
			IO_class_registry_reader<::rtmath::ddscat::ddOutputSingle> >;
	}

	namespace ddscat {

		//using namespace rtmath::ddscat::ddOutputSingleKeys;

		std::string getStatNameFromId(stat_entries id)
		{
#define str(s) #s
#define CHECK(x) if(id==x) return str(x)
			CHECK(BETA); CHECK(THETA); CHECK(PHI);
			CHECK(WAVE); CHECK(FREQ); CHECK(AEFF); CHECK(DIPOLESPACING);

			CHECK(QEXT1); CHECK(QABS1); CHECK(QSCA1);
			CHECK(G11); CHECK(G21); CHECK(QBK1); CHECK(QPHA1);
			CHECK(QEXT2); CHECK(QABS2); CHECK(QSCA2);
			CHECK(G12); CHECK(G22); CHECK(QBK2); CHECK(QPHA2);
			CHECK(QEXTM); CHECK(QABSM); CHECK(QSCAM);
			CHECK(G1M); CHECK(G2M); CHECK(QBKM); CHECK(QPHAM);
			CHECK(QPOL); CHECK(DQPHA);
			CHECK(QSCAG11); CHECK(QSCAG21); CHECK(QSCAG31);
			CHECK(ITER1); CHECK(MXITER1); CHECK(NSCA1);
			CHECK(QSCAG12); CHECK(QSCAG22); CHECK(QSCAG32);
			CHECK(ITER2); CHECK(MXITER2); CHECK(NSCA2);
			CHECK(QSCAG1M); CHECK(QSCAG2M); CHECK(QSCAG3M);

			throw rtmath::debug::xBadInput(str(id));
#undef CHECK
#undef str
		}

		std::string getStatNameFromId(stat_entries_size_ts id)
		{
#define str(s) #s
#define CHECK(x) if(id==x) return str(x)
			CHECK(VERSION); CHECK(NUM_DIPOLES);
			CHECK(NUMP); CHECK(NUMF);
			throw rtmath::debug::xBadInput(str(id));
#undef CHECK
#undef str
		}


		implementsDDRES::implementsDDRES() :
			rtmath::io::implementsIObasic<ddOutputSingle, ddOutputSingle_IO_output_registry,
			ddOutputSingle_IO_input_registry, ddOutputSingle_Standard>(ddOutputSingle::writeDDSCAT, ddOutputSingle::readDDSCAT, known_formats())
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

		void ddOutputSingle::readDDSCAT(ddOutputSingle* obj, std::istream&in, std::shared_ptr<registry::IO_options> opts)
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

		void ddOutputSingle::writeDDSCAT(const ddOutputSingle* obj, std::ostream &out, std::shared_ptr<registry::IO_options> opts)
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


		void ddOutputSingle::writeStatTable(std::ostream &out) const
		{
			using namespace std;
			out << "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			out << " JO=1: ";
			out.width(11);
			for (size_t i = (size_t)QEXT1; i< (size_t)QEXT2; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i = (size_t)QEXT2; i< (size_t)QEXTM; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " mean: ";
			out.width(11);
			for (size_t i = (size_t)QEXTM; i< (size_t)QPOL; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " Qpol= " << _statTable[(size_t)QPOL] <<
				"                                                  " <<
				"dQpha= ";
			out.width(11);
			out << _statTable[(size_t)DQPHA] << endl;

			out << "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca\n";
			out << " JO=1: ";
			out.width(11);
			for (size_t i = (size_t)QSCAG11; i< (size_t)QSCAG12; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i = (size_t)QSCAG12; i< (size_t)QSCAG1M; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " mean: ";
			for (size_t i = (size_t)QSCAG1M; i< (size_t)NUM_STAT_ENTRIES; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
		}


		void ddOutputSingle::readAVG(std::istream &in)
		{
			readHeader(in);
			readStatTable(in);
			readMueller(in);
		}


		void ddOutputSingle::readSCA(std::istream &in)
		{
			readHeader(in);
			readStatTable(in);
			readMueller(in);
		}


		void ddOutputSingle::writeAVG(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
#define WRITE(x) _objMap.at(x)->write(out,version())
			WRITE("version");
			WRITE("target");
			WRITE("solnmeth");
			WRITE("polarizability");
			WRITE("shape");
			WRITE("numdipoles");

			WRITE("d/aeff");
			WRITE("d");

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

			WRITE("kveclf");
			WRITE("incpol1lf");
			WRITE("incpol2lf");

			WRITE("betarange");
			WRITE("thetarange");
			WRITE("phirange");

			out << endl;

			WRITE("etasca");

			WRITE("avgnumori");
			WRITE("avgnumpol");

			// Write the odd table of Qsca and the others
			writeStatTable(out);

			// Write the P matrix
			writeMueller(out);
		}

		void ddOutputSingle::writeMueller(std::ostream &out) const
		{
			using namespace std;
			out << "            Mueller matrix elements for selected scattering directions in Lab Frame" << endl;
			out << " theta    phi    Pol.    "; // "S_11        S_12        S_21       S_22       S_31       S_41\n";
			for (auto it = _muellerMap.begin(); it != _muellerMap.end(); ++it)
			{
				out << "S_" << (it->second.first + 1) << (it->second.second + 1);
				auto ot = it;
				ot++;
				if (ot != _muellerMap.end()) out << "        ";
			}
			out << "\n";

			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				boost::shared_ptr<const ddscat::ddScattMatrix> sf(*it);
				out << endl;
				//out.width(6);
				using namespace std;
				//out.width(6);
				out << fixed << right << showpoint << setprecision(2) << setw(6) << (*it)->theta() << " ";
				out << setw(6) << (*it)->phi() << " ";
				//out.width(8);
				out << setprecision(5) << setw(8) << sf->polLin() << " ";
				//out.width(10);
				ddScattMatrix::PnnType p = sf->mueller();
				for (auto ot = _muellerMap.begin(); ot != _muellerMap.end(); ++ot)
				{
					out << " " << scientific << setprecision(4) << setw(10) << p(ot->second.first, ot->second.second);
				}
			}
		}

		void ddOutputSingle::writeSCA(std::ostream &out) const
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

		void ddOutputSingle::writeFML(std::ostream &out) const
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


		size_t ddOutputSingle::numP() const
		{
			//if (_statTable_Size_ts.at(stat_entries_size_ts::NUMP))
			return _statTable_Size_ts.at(stat_entries_size_ts::NUMP);
			//size_t i = 0;
			//for (const auto &m : _scattMatricesRaw)
			//	if (m->id() == rtmath::ddscat::scattMatrixType::P) ++i;
			//return i;
		}

		size_t ddOutputSingle::numF() const
		{
			//if (_statTable_Size_ts.at(stat_entries_size_ts::NUMF))
			return _statTable_Size_ts.at(stat_entries_size_ts::NUMF);
			//size_t i = 0;
			//for (const auto &m : _scattMatricesRaw)
			//	if (m->id() == rtmath::ddscat::scattMatrixType::F) ++i;
			//return i;
		}

		void ddOutputSingle::getScattMatrices(scattMatricesContainer& c) const
		{
			c = _scattMatricesRaw;
		}

		ddOutputSingle::scattMatricesContainer& ddOutputSingle::getScattMatrices()
		{
			return _scattMatricesRaw;
		}

		void ddOutputSingle::getStatTable(statTableType &res) const
		{
			res = _statTable;
		}

		void ddOutputSingle::getStatTable(statTableTypeSize_ts &res) const
		{
			res = _statTable_Size_ts;
		}


		double ddOutputSingle::getStatEntry(stat_entries e) const
		{
			return _statTable[e];
		}

		size_t ddOutputSingle::getStatEntry(stat_entries_size_ts e) const
		{
			return _statTable_Size_ts[e];
		}


		void ddOutputSingle::getHeaderMaps(headerMap &res) const
		{
			res = _objMap;
		}


		ddOutputSingle::ddOutputSingle(const std::string &infile, const std::string &type)
		{
			_init();
			if (infile.size()) readFile(infile, type);
		}

		ddOutputSingle::ddOutputSingle(const ddOutputSingle &base) :
			_muellerMap(base._muellerMap),
			_statTable(base._statTable),
			_statTable_Size_ts(base._statTable_Size_ts)
		{
			// Perform a deep copy of the header map
			for (auto it : base._objMap)
			{
				_objMap.insert(std::pair<std::string,
					boost::shared_ptr<ddOutputSingleObj> >
					(it.first, it.second->clone()));
			}
			// Perform a deep copy of the scattering matrices
			for (auto it : base._scattMatricesRaw)
			{
				_scattMatricesRaw.insert(
					boost::shared_ptr<ddScattMatrix>(it->clone()));
			}
		}

		void ddOutputSingle::_init()
		{
			_statTable_Size_ts.resize(stat_entries_size_ts::NUM_STAT_ENTRIES_INTS, 0);
			_statTable_Size_ts.at(stat_entries_size_ts::VERSION) 
				= rtmath::ddscat::ddVersions::getDefaultVer();

			_statTable.resize(NUM_STAT_ENTRIES, 0);

			// theta phi Pol. S_11 S_12 S_21 S_22 S_31 S_41
			_muellerMap[3] = std::pair<size_t, size_t>(0, 0);
			_muellerMap[4] = std::pair<size_t, size_t>(0, 1);
			_muellerMap[5] = std::pair<size_t, size_t>(1, 0);
			_muellerMap[6] = std::pair<size_t, size_t>(1, 1);
			_muellerMap[7] = std::pair<size_t, size_t>(2, 0);
			_muellerMap[8] = std::pair<size_t, size_t>(3, 0);

			::rtmath::io::Serialization::implementsSerialization<
				ddOutputSingle, ddOutputSingle_IO_output_registry,
				ddOutputSingle_IO_input_registry, ddOutputSingle_serialization>::set_sname("rtmath::ddscat::ddOutputSingle");
		}

		ddOutputSingle::~ddOutputSingle() {}


		size_t ddOutputSingle::version() const
		{
			return _statTable_Size_ts.at(stat_entries_size_ts::VERSION);
		}

		double ddOutputSingle::beta() const
		{
			return _statTable.at(stat_entries::BETA); 
		}

		double ddOutputSingle::theta() const
		{
			return _statTable.at(stat_entries::THETA); 
		}

		double ddOutputSingle::phi() const
		{
			return _statTable.at(stat_entries::PHI); 
		}


		double ddOutputSingle::wave() const
		{
			return _statTable.at(stat_entries::WAVE); 
		}

		double ddOutputSingle::freq() const
		{
			return _statTable.at(stat_entries::FREQ); 
		}

		double ddOutputSingle::aeff() const
		{
			return _statTable.at(stat_entries::AEFF); 
		}


		bool ddOutputSingle::operator<(const ddOutputSingle &rhs) const
		{
#define comp(x) if (_statTable.at(x) != rhs._statTable.at(x)) \
	return _statTable.at(x) < rhs._statTable.at(x);
			comp(stat_entries::WAVE);
			comp(stat_entries::AEFF);
			comp(stat_entries::BETA);
			comp(stat_entries::THETA);
			comp(stat_entries::PHI);
			return false;
#undef comp
		}


		bool ddOutputSingleObj::operator==(const ddOutputSingleObj &rhs) const
		{
			if (key != rhs.key) return false;
			// In absence of an id field (for introspection), simply write
			// the values ob both objects and do a comparison.

			std::ostringstream oa, ob;
			write(oa, 0);
			rhs.write(ob, 0);
			std::string sa = oa.str(), sb = ob.str();
			//std::cerr << sa << sb << std::endl;
			//sa = value();
			//sb = rhs.value();
			//std::cerr << "\t" << sa << std::endl << "\t" << sb << std::endl;
			return sa == sb;
		}

		bool ddOutputSingleObj::operator!=(const ddOutputSingleObj &rhs) const
		{
			return !operator==(rhs);
		}


		double ddOutputSingle::dipoleSpacing() const
		{
			return _statTable.at(stat_entries::DIPOLESPACING);
		}

		size_t ddOutputSingle::numDipoles() const
		{
			return _statTable_Size_ts.at(stat_entries_size_ts::NUM_DIPOLES); 
		}


		boost::shared_ptr<ddOutputSingleObj> ddOutputSingle::getObj(const std::string &id) const
		{
			if (_objMap.count(id) == 0) RTthrow debug::xBadInput(id.c_str());
			return _objMap.at(id);
		}


		void ddOutputSingle::readStatTable(std::istream &in)
		{
			using namespace std;
			string line;
			// First line was detected by the calling function, so it is outside of the istream now.
			//std::getline(in,line); // "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			in >> line; // " JO=1: ";
			for (size_t i = QEXT1; i< (size_t)QEXT2; i++)
				in >> _statTable[i];
			in >> line; // " JO=2: ";
			for (size_t i = (size_t)QEXT2; i< (size_t)QEXTM; i++)
				in >> _statTable[i];
			in >> line; // " mean: ";
			for (size_t i = (size_t)QEXTM; i< (size_t)QPOL; i++)
				in >> _statTable[i];
			in >> line // " Qpol= " 
				>> _statTable[(size_t)QPOL] >> line; // "dQpha=";
			in >> _statTable[(size_t)DQPHA];

			std::getline(in, line); // "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca";
			std::getline(in, line);
			in >> line; // " JO=1: ";
			for (size_t i = (size_t)QSCAG11; i< (size_t)QSCAG12; i++)
				in >> _statTable[i];
			in >> line; // " JO=2: ";
			for (size_t i = (size_t)QSCAG12; i< (size_t)QSCAG1M; i++)
				in >> _statTable[i];
			in >> line; // " mean: ";
			for (size_t i = (size_t)QSCAG1M; i< (size_t)NUM_STAT_ENTRIES; i++)
				in >> _statTable[i];
			std::getline(in, line);
		}

		ddOutputSingleObj::ddOutputSingleObj() { }

		ddOutputSingleObj::~ddOutputSingleObj() { }

		void ddOutputSingleObj::setKey(const std::string &k)
		{
			key = k;
		}



	}

}


std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddOutputSingleObj &ob)
{
	ob.write(stream);
	return stream;
}
