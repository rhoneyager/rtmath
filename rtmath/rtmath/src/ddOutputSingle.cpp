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

#include <Ryan_Serialization/serialization.h>

#include "../rtmath/Serialization/serialization_macros.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/assume_abstract.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/complex.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>

#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddOutputSingleKeys.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace registry {
		template struct IO_class_registry
			<::rtmath::ddscat::ddOutputSingle>;

		template class usesDLLregistry<
			::rtmath::ddscat::ddOutputSingle_IO_output_registry,
			IO_class_registry<::rtmath::ddscat::ddOutputSingle> >;
	}

	namespace ddscat {
		using namespace rtmath::ddscat::ddOutputSingleKeys;

		std::string getStatNameFromId(stat_entries id)
		{
#define str(s) #s
#define CHECK(x) if(id==x) return str(x)
			/*
			QEXT1,QABS1,QSCA1,G11,G21,QBK1,QPHA1,
			QEXT2,QABS2,QSCA2,G12,G22,QBK2,QPHA2,
			QEXTM,QABSM,QSCAM,G1M,G2M,QBKM,QPHAM,
			QPOL,DQPHA,
			QSCAG11,QSCAG21,GSCAG31,ITER1,MXITER1,NSCA1,
			QSCAG12,QSCAG22,GSCAG32,ITER2,MXITER2,NSCA2,
			QSCAG1M,QSCAG2M,QSCAG3M,
			*/
			CHECK(QEXT1); CHECK(QABS1); CHECK(QSCA1);
			CHECK(G11); CHECK(G21); CHECK(QBK1); CHECK(QPHA1);
			CHECK(QEXT2); CHECK(QABS2); CHECK(QSCA2);
			CHECK(G12); CHECK(G22); CHECK(QBK2); CHECK(QPHA2);
			CHECK(QEXTM); CHECK(QABSM); CHECK(QSCAM);
			CHECK(G1M); CHECK(G2M); CHECK(QBKM); CHECK(QPHAM);
			CHECK(QPOL); CHECK(DQPHA);
			CHECK(QSCAG11); CHECK(QSCAG21); CHECK(GSCAG31);
			CHECK(ITER1); CHECK(MXITER1); CHECK(NSCA1);
			CHECK(QSCAG12); CHECK(QSCAG22); CHECK(GSCAG32);
			CHECK(ITER2); CHECK(MXITER2); CHECK(NSCA2);
			CHECK(QSCAG1M); CHECK(QSCAG2M); CHECK(QSCAG3M);

			throw rtmath::debug::xBadInput(str(id));
#undef CHECK
#undef str
		}

#pragma message("Warning: ddOutputSingle needs correct normalization and default values")
		boost::shared_ptr<ddOutputSingle> ddOutputSingle::normalize() const
		{
			// Following the Mishchenko normalization condition, take and compute the Mueller matrix entries 
			// and normalize them. Note: this directly evaluates an integral using a summation method since DDSCAT 
			// does not provide good default angles for quadrature.

			// pf normalization condition is that 
			// 1 = 1/{4\pi} \int_0^\pi d\Theta \sin \Theta P_11(\Theta,\beta,\phi) \int_0^2\pi d\Phi
			double intres = 1.0 / (4.0 * boost::math::constants::pi<double>());

			// This summation is easier than expected because the values vary sinusoidally. In the case of 
			// phi = 0 and 90 degrees, I can find the mean value just by taking te arithmetic mean.

			// Take each pf and multiply by the normalization constraint
			boost::shared_ptr<ddOutputSingle> res(new ddOutputSingle);
			throw debug::xUnimplementedFunction();
			return res;
		}

		void ddOutputSingle::writeFile(const std::string &filename, const std::string &type) const
		{
			using namespace Ryan_Serialization;
			std::string cmeth, uncompressed;

			/// \todo Add a table to determine which file types are automatically compressed on saving.
			/// For those types, automatically apply compression.
#pragma message("TODO: add a table to determine which file types are automatically compressed on saving")

			uncompressed_name(filename, uncompressed, cmeth);
			boost::filesystem::path p(uncompressed);
			boost::filesystem::path pext = p.extension(); // Uncompressed extension

			std::string utype = type;
			if (!utype.size()) utype = pext.string();

			// Serialization gets its own override
			if (Ryan_Serialization::known_format(utype))
			{
				Ryan_Serialization::write<ddOutputSingle>(*this, filename, "rtmath::ddscat::ddOutputSingle");
				return;
			}

			std::ofstream out(filename.c_str(), std::ios_base::out | std::ios_base::binary);
			using namespace boost::iostreams;
			filtering_ostream sout;
			if (cmeth.size())
				prep_compression(cmeth, sout);

			sout.push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
			sout.push(out);

			if (utype == ".sca")
			{
				writeSCA(sout);
			} else if (utype == ".fml")
			{
				writeFML(sout);
			} else if (utype == ".avg")
			{
				writeAVG(sout);
			} else {
				throw rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		void ddOutputSingle::readStatTable(std::istream &in)
		{
			using namespace std;
			string line;
			// First line was detected by the calling function, so it is outside of the istream now.
			//std::getline(in,line); // "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			in >> line; // " JO=1: ";
			for (size_t i=0; i< (size_t) QEXT2; i++)
				in >> _statTable[i];
			in >> line; // " JO=2: ";
			for (size_t i=(size_t) QEXT2; i< (size_t) QEXTM; i++)
				in >> _statTable[i];
			in >> line; // " mean: ";
			for (size_t i=(size_t) QEXTM; i< (size_t) QPOL; i++)
				in >> _statTable[i];
			in >> line // " Qpol= " 
				>> _statTable[(size_t) QPOL] >> line; // "dQpha=";
			in >> _statTable[(size_t) DQPHA];

			std::getline(in,line); // "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca";
			std::getline(in,line);
			in >> line; // " JO=1: ";
			for (size_t i=(size_t) QSCAG11; i< (size_t) QSCAG12; i++)
				in >> _statTable[i];
			in >> line; // " JO=2: ";
			for (size_t i=(size_t) QSCAG12; i< (size_t) QSCAG1M; i++)
				in >> _statTable[i];
			in >> line; // " mean: ";
			for (size_t i=(size_t) QSCAG1M; i< (size_t) NUM_STAT_ENTRIES; i++)
				in >> _statTable[i];
			std::getline(in,line);
		}

		void ddOutputSingle::readFile(const std::string &filename, const std::string &type)
		{
			// First, detect if the file is compressed.
			using namespace Ryan_Serialization;
			std::string cmeth, target, uncompressed;
			// Combination of detection of compressed file, file type and existence.
			if (!detect_compressed(filename, cmeth, target))
				throw rtmath::debug::xMissingFile(filename.c_str());
			uncompressed_name(target, uncompressed, cmeth);

			boost::filesystem::path p(uncompressed);
			boost::filesystem::path pext = p.extension(); // Uncompressed extension

			// Serialization gets its own override
			if (Ryan_Serialization::known_format(pext))
			{
				// This is a serialized file. Verify that it has the correct identifier, and 
				// load the serialized object directly
				Ryan_Serialization::read<ddOutputSingle>(*this, filename, "rtmath::ddscat::ddOutputSingle");
				return;
			}

			std::ifstream in(filename.c_str(), std::ios_base::binary | std::ios_base::in);
			// Consutuct an filtering_iostream that matches the type of compression used.
			using namespace boost::iostreams;
			filtering_istream sin;
			if (cmeth.size())
				prep_decompression(cmeth, sin);
			sin.push(boost::iostreams::newline_filter(boost::iostreams::newline::posix));
			sin.push(in);

			if (type.size()) pext = boost::filesystem::path(type); // pext is first set a few lines above
			if (pext.string() == ".sca")
			{
				readSCA(sin);
			} else if (pext.string() == ".fml")
			{
				readFML(sin);
			} else if (pext.string() == ".avg")
			{
				readAVG(sin);
			} else {
				throw rtmath::debug::xUnknownFileFormat(filename.c_str());
			}
		}

		void ddOutputSingle::version(size_t nv)
		{
			auto obj = ddOutputSingleObj::constructObj("version");
			auto objc = boost::dynamic_pointer_cast<ddver>(obj);
			objc->version(nv);
			_objMap["version"] = obj;
			_version = nv;
		}

		void ddOutputSingle::getTARGET(std::string &target) const
		{
			target = _objMap.at("target")->value();
		}

		void ddOutputSingle::setTARGET(const std::string &target)
		{
			auto obj = ddOutputSingleObj::constructObj("target");
			auto objc = boost::dynamic_pointer_cast<ddtarget>(obj);
			objc->setTarget(target);
			_objMap["target"] = obj;
		}

		void ddOutputSingle::readFML(std::istream &in)
		{
			readHeader(in,"Re(f_11)");
			// Get e1 and e2 in lab frame from the header data
			auto obj1 = boost::dynamic_pointer_cast<ddPolVec>(_objMap.at("incpol1lf"));
			auto obj2 = boost::dynamic_pointer_cast<ddPolVec>(_objMap.at("incpol2lf"));
			std::vector<std::complex<double> > vs(6);
			vs[0] = obj1->getPol(0);
			vs[1] = obj1->getPol(1);
			vs[2] = obj1->getPol(2);
			vs[3] = obj2->getPol(0);
			vs[4] = obj2->getPol(1);
			vs[5] = obj2->getPol(2);
			boost::shared_ptr<const ddScattMatrixConnector> cn =
				ddScattMatrixConnector::fromVector(vs);
			readF(in, cn);
		}

		void ddOutputSingle::readSCA(std::istream &in)
		{
			readHeader(in);
			readStatTable(in);
			readMueller(in);
		}

		void ddOutputSingle::readHeader(std::istream &in, const std::string &sstop)
		{
			using namespace std;
			std::string lin;
			bool headerDone = false;
			size_t line = 0;
			while (!headerDone)
			{
				std::getline(in,lin);
				line++;
				if (lin.find(sstop) != string::npos)
				{
					headerDone = true;
					//cerr << "Header done on line " << line << endl;
					break;
				}
				string key;
				ddOutputSingleObj::findMap(lin,key);
				if (key == "")
				{
					if (lin == "" || lin == "\r") continue;
					cerr << "Unknown line: " << lin << endl;
					continue;
				}
				auto obj = ddOutputSingleObj::constructObj(key);
				istringstream ii(lin);
				obj->read(ii);
				_objMap[key] = obj;
				if (key == "beta") 
					_beta = boost::lexical_cast<double>(obj->value());
				if (key == "theta") 
					_theta = boost::lexical_cast<double>(obj->value());
				if (key == "phi") 
					_phi = boost::lexical_cast<double>(obj->value());
				if (key == "wave") _wave = boost::lexical_cast<double>(obj->value());
				if (key == "aeff") _aeff = boost::lexical_cast<double>(obj->value());
				if (key == "version") _version = boost::lexical_cast<size_t>(obj->value());
			}
		}

		void ddOutputSingle::readAVG(std::istream &in)
		{
			readHeader(in);
			readStatTable(in);
			readMueller(in);
		}

		void ddOutputSingle::writeStatTable(std::ostream &out) const
		{
			using namespace std;
			out << "          Qext       Qabs       Qsca      g(1)=<cos>  <cos^2>     Qbk       Qpha" << endl;
			out << " JO=1: ";
			out.width(11);
			for (size_t i=0; i< (size_t) QEXT2; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i=(size_t) QEXT2; i< (size_t) QEXTM; i++)
				out << "\t" << _statTable[i] ;
			out << endl;
			out.width(0);
			out << " mean: ";
			out.width(11);
			for (size_t i=(size_t) QEXTM; i< (size_t) QPOL; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " Qpol= " << _statTable[(size_t) QPOL] << 
				"                                                  " << 
				"dQpha= ";
			out.width(11);
			out << _statTable[(size_t) DQPHA] << endl;

			out << "         Qsca*g(1)   Qsca*g(2)   Qsca*g(3)   iter  mxiter  Nsca\n";
			out << " JO=1: ";
			out.width(11);
			for (size_t i=(size_t) QSCAG11; i< (size_t) QSCAG12; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " JO=2: ";
			out.width(11);
			for (size_t i=(size_t) QSCAG12; i< (size_t) QSCAG1M; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
			out << " mean: ";
			for (size_t i=(size_t) QSCAG1M; i< (size_t) NUM_STAT_ENTRIES; i++)
				out << "\t" << _statTable[i];
			out << endl;
			out.width(0);
		}

		size_t ddOutputSingle::numP() const
		{
			size_t i = 0;
			for (const auto &m : _scattMatricesRaw)
				if (m->id() == rtmath::ddscat::scattMatrixType::P) ++i;
			return i;
		}

		size_t ddOutputSingle::numF() const
		{
			size_t i = 0;
			for (const auto &m : _scattMatricesRaw)
				if (m->id() == rtmath::ddscat::scattMatrixType::F) ++i;
			return i;
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

		double ddOutputSingle::getStatEntry(stat_entries e) const 
		{
			return _statTable[e];
		}

		void ddOutputSingle::getHeaderMaps(headerMap &res) const
		{
			res = _objMap;
		}

		void ddOutputSingle::writeAVG(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
#define WRITE(x) _objMap.at(x)->write(out,_version)
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
			if (rtmath::ddscat::ddVersions::isVerWithin(_version,72,0))
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
				out << setw(6) << (*it)->phi()  << " ";
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
			if (rtmath::ddscat::ddVersions::isVerWithin(_version,72,0))
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
			if (rtmath::ddscat::ddVersions::isVerWithin(_version,72,0))
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

		boost::shared_ptr<ddOutputSingleObj> ddOutputSingle::getObj(const std::string &id) const
		{
			if (_objMap.count(id) == 0) throw debug::xBadInput(id.c_str());
			return _objMap.at(id);
		}

		double ddOutputSingle::dipoleSpacing() const
		{
			return boost::lexical_cast<double>(_objMap.at("d")->value());
		}

		size_t ddOutputSingle::numDipoles() const
		{
			return boost::lexical_cast<size_t>(_objMap.at("numdipoles")->value());
		}

		std::complex<double> ddOutputSingle::getM() const
		{
			// if (key == "neps") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddM>(new ddM));
			auto obj = boost::dynamic_pointer_cast<ddM>(_objMap.at("neps"));
			return obj->getM();
		}

		void ddOutputSingle::writeF(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(f_11)   Im(f_11)   Re(f_21)   Im(f_21)   Re(f_12)   Im(f_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != scattMatrixType::F) continue;
				boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
					boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it));
				out << endl;
				out.width(6);
				// it->first crds ordering is freq, phi, theta
				out << (*it)->theta() << "\t";
				out << (*it)->phi();
				out.width(11);
				ddScattMatrix::FType f = sf->getF();

				for (size_t j = 0; j < 2; j++)
					for (size_t i=0; i < 2; i++)
					{
						// Note the reversed coordinates. This matches ddscat.
						out << "\t" << f(i,j).real();
						out << "\t" << f(i,j).imag();
					}
			}
		}

		void ddOutputSingle::writeS(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(S_11)   Im(S_11)   Re(S_21)   Im(S_21)   Re(S_12)   Im(S_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != scattMatrixType::F) continue;
				boost::shared_ptr<const ddscat::ddScattMatrixF> sf(
					boost::dynamic_pointer_cast<const ddscat::ddScattMatrixF>(*it));
				out << endl;
				out.width(6);
				// it->first crds ordering is freq, phi, theta
				out << (*it)->theta() << "\t";
				out << (*it)->phi();
				out.width(11);
				ddScattMatrix::FType s = sf->getS();
				for (size_t j = 0; j < 2; j++)
					for (size_t i=0; i < 2; i++)
					{
						// Note the reversed coordinates. This matches ddscat.
						out << "\t" << s(i,j).real();
						out << "\t" << s(i,j).imag();
					}
			}
		}

		ddOutputSingle::ddOutputSingle(const std::string &infile, const std::string &type)
		{
			_init();
			if (infile.size()) readFile(infile,type);
		}

		ddOutputSingle::ddOutputSingle(const ddOutputSingle &base) :
			_version(base._version),
			_muellerMap(base._muellerMap),
			_beta(base._beta),
			_theta(base._theta),
			_phi(base._phi),
			_wave(base._wave),
			_aeff(base._aeff),
			_statTable(base._statTable)
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
			_version = rtmath::ddscat::ddVersions::getDefaultVer();
			_beta = 0;
			_theta = 0;
			_phi = 0;
			_wave = 0;
			_aeff = 0;
			_statTable.resize(NUM_STAT_ENTRIES);

			// theta phi Pol. S_11 S_12 S_21 S_22 S_31 S_41
			_muellerMap[3] = std::pair<size_t, size_t>(0,0);
			_muellerMap[4] = std::pair<size_t, size_t>(0,1);
			_muellerMap[5] = std::pair<size_t, size_t>(1,0);
			_muellerMap[6] = std::pair<size_t, size_t>(1,1);
			_muellerMap[7] = std::pair<size_t, size_t>(2,0);
			_muellerMap[8] = std::pair<size_t, size_t>(3,0);
		}

		ddOutputSingle::~ddOutputSingle() {}

		size_t ddOutputSingle::version() const
		{
			return _version;
		}

		double ddOutputSingle::beta() const
		{
			return _beta;
		}

		double ddOutputSingle::theta() const
		{
			return _theta;
		}

		double ddOutputSingle::phi() const
		{
			return _phi;
		}

		double ddOutputSingle::wave() const
		{
			return _wave;
		}

		double ddOutputSingle::aeff() const
		{
			return _aeff;
		}

		bool ddOutputSingle::operator<(const ddOutputSingle &rhs) const
		{
			if (_wave != rhs._wave) return _wave < rhs._wave;
			if (_aeff != rhs._aeff) return _aeff < rhs._aeff;
			if (_beta != rhs._beta) return _beta < rhs._beta;
			if (_theta != rhs._theta) return _theta < rhs._theta;
			if (_phi != rhs._phi) return _phi < rhs._phi;
			return false;
		}

		/*
		void ddOutputSingle::writeEvans(std::ostream &out, double freq) const
		{
		using namespace std;
		// Takes the scattering data, generates the appropriate
		// phase, extinction matrices and emission vectors for a given
		// frequency, and writes the necessary file.

		// Generate quadrature points
		set<double> qangles;
		// Let's get the quadrature angles from gaussian quadrature
		// Other quadrature methods can be coded in as well
		const size_t deg = 7;
		quadrature::getQuadPtsLeg(deg,qangles);
		// The quadrature points are on interval (-1,1)
		// Need to get mapping between angles in degrees and these points
		// can handle this by mapping mu = cos(theta).

		// For phase functions, need to look at both incoming and outgoing angle
		// in cos(theta)
		// ddscat output always has incoming angle at zero degrees, with a varying output angle.
		// However, the targets may be rotated to simulate the angle change.
		// In this case, however, we likely just have a single ensemble pf from the data
		// TODO!!!!!
		std::map<coords::cyclic<double>, std::shared_ptr<const ddscat::ddScattMatrix> >
		interped;
		// Need to interpolate phase matrices to the correct quadrature points
		for (auto it = qangles.begin(); it != qangles.end(); it++)
		{
		// TODO: convert angle into cyclic coords
		throw rtmath::debug::xUnimplementedFunction();
		coords::cyclic<double> crd;
		std::shared_ptr<const ddscat::ddScattMatrix> interres;
		interpolate(crd, interres);
		interped[crd] = interres;
		}

		// First, write commented header information
		// This includes where the scattering information is from, and a
		// discription of each of these files
		out << "C  ddscat rtmath output for " << endl;
		out << "C  theta " << _theta << " phi " << _phi << " beta " << _beta << endl;
		out << "C  at f = " << _freq << " GHz" << endl;

		// Next is the degree and type of quadrature
		cout << "   8    0   'GAUSSIAN         '" << endl;

		// Output each scattering matrix at the designated quadrature incoming
		// and outgoing angles
		out << "C   SCATTERING MATRIX" << endl;
		for (auto it = interped.begin(); it != interped.end(); ++it)
		{
		// Write incoming angle, outcoming angle, 0
		}

		// Write the extinction matrix at each quadrature angle
		out << "C   EXTINCTION MATRIX" << endl;
		for (auto it = interped.begin(); it != interped.end(); ++it)
		{
		// Write incoming angle
		}

		// Output the emission vectors
		out << "C   EMISSION VECTOR" << endl;
		for (auto it = interped.begin(); it != interped.end(); ++it)
		{
		// Write incoming angle and the four stokes parameters
		}
		// Evans fortran files lack a newline at EOF.
		}
		*/

		/*
		void ddOutputSingle::loadFile(const std::string &filename)
		{
		using namespace std;
		// File loading routine is important!
		// Load a standard .fml file. Parse each line for certain key words.
		clear();
		bool dataseg = false;
		this->_filename = filename;

		// If a shape is not loaded, then try to load the corresponding shapefile
		using namespace boost::filesystem;
		path pshapepath, p, pfile(filename);
		p = pfile.parent_path();
		string shapepath;
		{
		// Figure out where the shape file is located.
		path ptarget = p / "target.out";
		path pshapedat = p / "shape.dat";
		if (exists(ptarget))
		{ pshapepath = ptarget;
		} else if (exists(pshapedat))
		{ pshapepath = pshapedat;
		} else {
		throw rtmath::debug::xMissingFile("shape.dat or target.out");
		}
		shapepath = pshapepath.string();
		if (exists(pshapepath) && !_shape)
		_shape = boost::shared_ptr<shapefile>(new shapefile(shapepath));
		}

		ifstream in(filename.c_str(), std::ifstream::in);
		while (in.good())
		{
		// Read a line
		string lin;
		std::getline(in,lin);
		istringstream lss(lin);
		// Expand line using convenient expansion function
		//vector<string> seg;
		//splitString(lin, ' ', seg);

		// Are we in the data segment?
		if (dataseg)
		{
		// In data segment
		// Entry not modified after this, because it is const
		std::shared_ptr<const ddScattMatrix> nscat (new ddScattMatrix(_freq,lss));
		// Save to the maps and sets
		// Totally assuming that no duplicate entry exists
		_insert(nscat);
		} else {
		// Still in header segment
		string junk;
		// Search for key strings in file

		// BETA
		if (lin.find("BETA") != string::npos)
		{
		lss >> junk; // get rid of first word
		lss >> junk;
		lss >> _beta;
		}
		// THETA
		if (lin.find("THETA") != string::npos)
		{
		lss >> junk; // get rid of first word
		// Theta is unlike Beta and Phi, as there is
		// no space between THETA and =
		lss >> _theta;
		}
		// PHI
		if (lin.find("PHI") != string::npos)
		{
		lss >> junk; // get rid of first word
		lss >> junk;
		lss >> _phi;
		}
		// NAT0
		if (lin.find("NAT0") != string::npos)
		{
		lss >> _numDipoles;
		}
		// AEFF
		if (lin.find("AEFF") != string::npos)
		{
		lss >> junk; // get rid of first word
		lss >> _reff;
		}
		// WAVE
		if (lin.find("WAVE") != string::npos)
		{
		// BAD --- WAVE runs against size...
		//lss >> junk; // get rid of first word
		//lss >> _wavelength;
		// Instead, read wave from column 7 (starting at 0) to 17
		_wavelength = atof( lin.substr( 7, 10 ).c_str() );
		// Also do a conversion from wavelength to frequency,
		// for easier comparisons later
		units::conv_spec wvtof("um","GHz");
		_freq = wvtof.convert(_wavelength);
		}
		// theta --- indicates last line of header
		if (lin.find("Re(f_11)") != string::npos)
		{
		dataseg = true;
		}
		}
		}
		}
		*/

	} // end ddscat
} // end rtmath

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::ddOutputSingleObj &ob)
{
	ob.write(stream);
	return stream;
}





namespace rtmath
{
	namespace ddscat
	{
		template <class Archive>
		void ddOutputSingle::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("version", _version);
			ar & boost::serialization::make_nvp("wave", _wave);
			ar & boost::serialization::make_nvp("aeff", _aeff);
			ar & boost::serialization::make_nvp("objMap", _objMap);
			ar & boost::serialization::make_nvp("beta", _beta);
			ar & boost::serialization::make_nvp("theta", _theta);
			ar & boost::serialization::make_nvp("phi", _phi);
			ar & boost::serialization::make_nvp("statTable", _statTable);
			ar & boost::serialization::make_nvp("muellerMap", _muellerMap);
			ar & boost::serialization::make_nvp("scattMatricesRaw", _scattMatricesRaw);
		}

		template <class Archive>
		void ddOutputSingleObj::serialize(Archive & ar, const unsigned int version)
		{
			ar & boost::serialization::make_nvp("key", key);
		}

		EXPORTINTERNAL(rtmath::ddscat::ddOutputSingle::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleObj::serialize);
	}
}

