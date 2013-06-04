#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/filesystem.hpp>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/units.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"

namespace
{
	class ddver : public ::rtmath::ddscat::ddOutputSingleObj
	{
	public:
		ddver() { _version = rtmath::ddscat::ddVersions::getDefaultVer(); }
		virtual ~ddver() {}
		virtual void write(std::ostream &out)
		{
			out << " DDSCAT --- ";
			out << rtmath::ddscat::ddVersions::getVerAvgHeaderString(_version);
			out << std::endl;
		}
		virtual void read(std::istream &in)
		{
			std::string lin;
			std::getline(in,lin);
			_version = rtmath::ddscat::ddVersions::getVerId(lin);
		}
		size_t _version;
		size_t version() const { return _version; }
		void version(size_t n) { _version = n; }
	};
	class ddstring : public ::rtmath::ddscat::ddOutputSingleObj
	{
	public:
		ddstring() {}
		virtual ~ddstring() {}
		virtual void write(std::ostream &out)
		{
			out << s << std::endl;
		}
		virtual void read(std::istream &in)
		{
			std::getline(in,s);
		}
		virtual std::string value() const { return s; }
		std::string s;
	};
	class ddtarget : public ::rtmath::ddscat::ddOutputSingleObj
	{
	public:
		ddtarget() {}
		virtual ~ddtarget() {}
		virtual void write(std::ostream &out)
		{
			out << " TARGET --- ";
			out << s << std::endl;
		}
		virtual void read(std::istream &in)
		{
			std::string lin;
			std::getline(in,lin);
			size_t p = lin.find("---");
			s = lin.substr(p+3);
			// Remove any leading and lagging spaces
			// Not all Liu avg files are correct in this respect
			boost::algorithm::trim(s);
		}
		virtual std::string value() const { return s; }
		std::string s;
	};
	class ddSval : public ::rtmath::ddscat::ddOutputSingleObj
	{
	public:
		ddSval(const std::string &tail) {this->tail = tail;}
		virtual ~ddSval() {}
		virtual void write(std::ostream &out)
		{
			out << s << "--- " << tail << std::endl;
		}
		virtual void read(std::istream &in)
		{
			std::string lin;
			std::getline(in,lin);
			size_t p = lin.find("--- ");
			s = lin.substr(0,p);
		}
		std::string s, tail;
	};

	template <class T>
	class ddNval : public ::rtmath::ddscat::ddOutputSingleObj
	{
	public:
		ddNval(size_t pos = 0, const std::string &head = "", const std::string &tail = "") {this->pos = pos; this->head = head; this->tail = tail;}
		virtual ~ddNval() {}
		virtual void write(std::ostream &out)
		{
			out << head << val << tail << std::endl;
		}
		virtual std::string value() const
		{
			std::ostringstream out;
			out << val;
			return out.str();
		}
		virtual void read(std::istream &in)
		{
			std::string lin;
			std::getline(in,lin);
			std::istringstream ii(lin);
			for (size_t i=0; i<pos; i++)
				ii >> lin;
			ii >> val;
		}
		size_t pos;
		T val;
		std::string head, tail;
	};

}

namespace rtmath {
	namespace ddscat {

		ddOutputSingleObj::ddOutputSingleObj() { }

		ddOutputSingleObj::~ddOutputSingleObj() { }

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
			throw;
			return res;
		}

		void ddOutputSingle::writeFile(const std::string &filename) const
		{
			// Look at extension of file
			std::ofstream out(filename.c_str());
			boost::filesystem::path p(filename);
			boost::filesystem::path pext = p.extension();
			if (pext.string() == ".sca")
			{
				writeSCA(out);
			} else if (pext.string() == ".fml")
			{
				writeFML(out);
			} else if (pext.string() == ".avg")
			{
				writeAVG(out);
			} else {
				throw rtmath::debug::xBadInput(filename.c_str());
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
			// Look at extension of file
			std::ifstream in(filename.c_str());
			boost::filesystem::path p(filename);
			boost::filesystem::path pext = p.extension();
			if (type.size()) pext = boost::filesystem::path(type);
			if (pext.string() == ".sca")
			{
				readSCA(in);
			} else if (pext.string() == ".fml")
			{
				readFML(in);
			} else if (pext.string() == ".avg")
			{
				readAVG(in);
			} else {
				throw rtmath::debug::xMissingFile(filename.c_str());
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

		void ddOutputSingle::readF(std::istream &in)
		{
			using namespace std;
			// The frequency is needed when reading this matrix
			double freq = 0;
			if (wave())
				freq = units::conv_spec("um","GHz").convert(wave());

			string lin;
			while(in.good())
			{
				std::getline(in,lin);
				if (lin == "") return;
				// Parse the string to get rid of spaces. This is used to determine 
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (std::isalpha(lin.at(0))) continue;
				typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
				boost::char_separator<char> sep("\t ");
				tokenizer t(lin, sep);

				// TODO: check this
				// The ordering is theta, phi, S11 (real and complex), S12, S21, S22
				size_t i=0;
				double vals[10];
				for (auto it = t.begin(); it != t.end(); ++it, ++i)
				{
					vals[i] = boost::lexical_cast<double>(*it);
					//std::cerr << " i " << i << " - " << vals[i] << "\n";
				}
				// ddScattMatrixF constructor takes frequency (GHz) and phi
				boost::shared_ptr<ddScattMatrixF> mat(new ddScattMatrixF(freq, vals[0], vals[1]));
				ddScattMatrix::FType fs;
				fs(0,0) = complex<double>(vals[2],vals[3]);
				fs(1,0) = complex<double>(vals[4],vals[5]);
				fs(0,1) = complex<double>(vals[6],vals[7]);
				fs(1,1) = complex<double>(vals[8],vals[9]);
				mat->setF(fs);

				// TODO: check if another cast is appropriate
				boost::shared_ptr<const ddScattMatrix> matC = 
					boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

				_scattMatricesRaw.insert(matC);
			}
		}

		void ddOutputSingle::readMueller(std::istream &in)
		{
			using namespace std;
			// The frequency is needed when reading this matrix
			double freq = 0;
			if (wave())
				freq = units::conv_spec("um","GHz").convert(wave());

			string lin;
			mMuellerIndices &mIndices = _muellerMap;
			mIndices.clear();

			while(in.good())
			{
				std::getline(in,lin);
				if (lin == "") return;
				// Parse the string to get rid of spaces. This is used to determine 
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				//std::cerr << lin << std::endl;
				// TODO: parse the header line to get the list of matrix entries known
				// TODO: use symmetry relationships in a depGraph to get the other 
				// mueller matrix entries.
				typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
				boost::char_separator<char> sep("\t ");
				tokenizer t(lin, sep);

				// Expecting the first line to begin with theta phi Pol. ...
				if (std::isalpha(lin.at(0)))
				{
					size_t i=0; // Column number
					for (auto it = t.begin(); it != t.end(); ++it, ++i)
					{
						// Mueller entry columns have a '_'
						size_t loc;
						if (loc = it->find('_') == string::npos) continue;
						size_t id = (size_t) atoi(it->substr(loc+1).c_str());
						size_t row = (id / 10) - 1; // Annoying start at 1...
						size_t col = (id % 10) - 1;
						mIndices[i] = std::pair<size_t,size_t>(row,col);
					}

					// TODO: add function that generates the correct mueller relations from here
	#pragma message("Warning: ddOutputSingle needs the Mueller matrix filling routine")
				} else {
					// Parse the Mueller entries

					// TODO: check this
					// The ordering is theta, phi, polarization, and then the 
					// relevant matrix entries
					// theta phi Pol. S_11 S_12 S_21 S_22 S_31 S_41
					vector<double> vals;
					for (auto it = t.begin(); it != t.end(); ++it)
						vals.push_back(boost::lexical_cast<double>(*it));
					// ddScattMatrixF constructor takes frequency (GHz) and phi
					boost::shared_ptr<ddScattMatrixP> mat(new ddScattMatrixP(freq, vals[0], vals[1]));
					ddScattMatrix::PnnType P;

					for (auto ot = mIndices.begin(); ot != mIndices.end(); ++ot)
					{
						P(ot->second.first, ot->second.second) = vals[ot->first]; // See Mueller header read
					}
	#pragma message("Warning: ddOutputSingle needs the Mueller matrix filling routine (part b)")
					mat->setP(P);
					mat->pol(vals[2]);

					boost::shared_ptr<const ddScattMatrix> matC = 
						boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

					_scattMatricesRaw.insert(matC);
				}
			}
		}

		void ddOutputSingle::readFML(std::istream &in)
		{
			readHeader(in,"Re(f_11)");
			//readStatTable(in);
			readF(in);
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
					if (lin == "") continue;
					cerr << "Unknown line: " << lin << endl;
					continue;
				}
				auto obj = ddOutputSingleObj::constructObj(key);
				istringstream ii(lin);
				obj->read(ii);
				_objMap[key] = obj;
				if (key == "beta") _beta = boost::lexical_cast<double>(obj->value());
				if (key == "theta") _theta = boost::lexical_cast<double>(obj->value());
				if (key == "phi") _phi = boost::lexical_cast<double>(obj->value());
				if (key == "wave") _wave = boost::lexical_cast<double>(obj->value());
				if (key == "aeff") _aeff = boost::lexical_cast<double>(obj->value());
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
				"dQpha=";
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

		double ddOutputSingle::getStatEntry(stat_entries e) const 
		{
			return _statTable[e];
		}

		void ddOutputSingle::writeAVG(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			_objMap.at("version")->write(out);
			_objMap.at("target")->write(out);
			_objMap.at("solnmeth")->write(out);
			_objMap.at("polarizability")->write(out);
			_objMap.at("shape")->write(out);
			_objMap.at("numdipoles")->write(out);

			_objMap.at("d/aeff")->write(out);
			_objMap.at("d")->write(out);

			_objMap.at("aeff")->write(out);
			_objMap.at("wave")->write(out);
			_objMap.at("k.aeff")->write(out);
			if (rtmath::ddscat::ddVersions::isVerWithin(_version,72,0))
				_objMap.at("nambient")->write(out);
			_objMap.at("neps")->write(out);
			_objMap.at("tol")->write(out);
			
			_objMap.at("a1tgt")->write(out);
			_objMap.at("a2tgt")->write(out);
			_objMap.at("navg")->write(out);

			_objMap.at("kveclf")->write(out);
			_objMap.at("incpol1lf")->write(out);
			_objMap.at("incpol2lf")->write(out);

			_objMap.at("betarange")->write(out);
			_objMap.at("thetarange")->write(out);
			_objMap.at("phirange")->write(out);

			out << endl;

			_objMap.at("etasca")->write(out);
			
			_objMap.at("avgnumori")->write(out);
			_objMap.at("avgnumpol")->write(out);

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
				out.width(6);
				
				out << (*it)->theta() << "\t";
				out << (*it)->phi()  << "\t";
				out.width(9);
				out << sf->pol();
				out.width(12);
				ddScattMatrix::PnnType p = sf->mueller();
				for (auto ot = _muellerMap.begin(); ot != _muellerMap.end(); ++ot)
				{
					out << "\t" << p(ot->second.first, ot->second.second);
				}
			}
		}

		void ddOutputSingle::writeSCA(std::ostream &out) const
		{
			using namespace std;
			// Make sure that all the necessary variables are known
			// TODO

			// Write the file in the appropriate order
			_objMap.at("version")->write(out);
			_objMap.at("target")->write(out);
			_objMap.at("solnmeth")->write(out);
			_objMap.at("polarizability")->write(out);
			_objMap.at("shape")->write(out);
			_objMap.at("numdipoles")->write(out);

			_objMap.at("d/aeff")->write(out);
			_objMap.at("d")->write(out);
			out << "----- physical extent of target volume in Target Frame ------" << endl;
			_objMap.at("xtf")->write(out);
			_objMap.at("ytf")->write(out);
			_objMap.at("ztf")->write(out);

			_objMap.at("aeff")->write(out);
			_objMap.at("wave")->write(out);
			_objMap.at("k.aeff")->write(out);
			if (rtmath::ddscat::ddVersions::isVerWithin(_version,72,0))
				_objMap.at("nambient")->write(out);
			_objMap.at("neps")->write(out);
			_objMap.at("tol")->write(out);
			
			_objMap.at("a1tgt")->write(out);
			_objMap.at("a2tgt")->write(out);
			_objMap.at("navg")->write(out);

			_objMap.at("kvectf")->write(out);
			_objMap.at("incpol1tf")->write(out);
			_objMap.at("incpol2tf")->write(out);
			_objMap.at("kveclf")->write(out);
			_objMap.at("incpol1lf")->write(out);
			_objMap.at("incpol2lf")->write(out);

			_objMap.at("beta")->write(out);
			_objMap.at("theta")->write(out);
			_objMap.at("phi")->write(out);

			_objMap.at("etasca")->write(out);

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
			_objMap.at("version")->write(out);
			_objMap.at("target")->write(out);
			_objMap.at("solnmeth")->write(out);
			_objMap.at("polarizability")->write(out);
			_objMap.at("shape")->write(out);
			_objMap.at("numdipoles")->write(out);

			_objMap.at("aeff")->write(out);
			_objMap.at("wave")->write(out);
			_objMap.at("k.aeff")->write(out);
			if (rtmath::ddscat::ddVersions::isVerWithin(_version,72,0))
				_objMap.at("nambient")->write(out);
			_objMap.at("neps")->write(out);
			_objMap.at("tol")->write(out);
			_objMap.at("navg")->write(out);
			_objMap.at("a1tgt")->write(out);
			_objMap.at("a2tgt")->write(out);
			_objMap.at("kvectf")->write(out);
			_objMap.at("incpol1tf")->write(out);
			_objMap.at("incpol2tf")->write(out);
			_objMap.at("kveclf")->write(out);
			_objMap.at("incpol1lf")->write(out);
			_objMap.at("incpol2lf")->write(out);
			_objMap.at("beta")->write(out);
			_objMap.at("theta")->write(out);
			_objMap.at("phi")->write(out);

			out << "     Finite target:" << endl;
			out << "     e_m dot E(r) = i*exp(ikr)*f_ml*E_inc(0)/(kr)" << endl;
			out << "     m=1 in scatt. plane, m=2 perp to scatt. plane" << endl;
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

		void ddOutputSingle::writeF(std::ostream &out) const
		{
			using namespace std;
			out << " theta   phi  Re(f_11)   Im(f_11)   Re(f_21)   Im(f_21)   Re(f_12)   Im(f_12)   Re(f_22)   Im(f_22)";
			for (auto it = _scattMatricesRaw.begin(); it != _scattMatricesRaw.end(); ++it)
			{
				if ((*it)->id() != F) continue;
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
				if ((*it)->id() != F) continue;
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

		boost::shared_ptr<ddOutputSingleObj> ddOutputSingleObj::constructObj(const std::string &key)
		{
			// Given the given key, construct the appropriate object.
			// Used in the reading algorithms.
			boost::shared_ptr<ddOutputSingleObj> res;

			if (key == "version") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddver>(new ddver));
			if (key == "target") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddtarget>(new ddtarget));
			if (key == "solnmeth") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval("method of solution") ));
			if (key == "polarizability") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval("prescription for polarizabilies") ));
			if (key == "shape") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval("shape") ));
			if (key == "numdipoles") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<size_t> >(new ddNval<size_t>(0, "", " = NAT0 = number of dipoles") ));
			if (key == "d/aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d/aeff for this target [d=dipole spacing]") ));
			if (key == "d") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d (physical units)") ));

			if (key == "aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  AEFF= ", " = effective radius (physical units)") ));
			if (key == "wave") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  WAVE= ", " = wavelength (in vacuo, physical units)") ));
			if (key == "k.aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "K*AEFF= ", " = 2*pi*aeff/lambda") ));
			if (key == "nambient") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "NAMBIENT= ", " = refractive index of ambient medium") ));

			if (key == "neps") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "tol") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "   TOL= ", " = error tolerance for CCG method") ));
			if (key == "a1tgt") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "a2tgt") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "navg") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  NAVG= ", " = (theta,phi) values used in comp. of Qsca,g") ));
			if (key == "kveclf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "kvectf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "incpol1lf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "incpol2lf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "incpol1tf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "incpol2tf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "betarange") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "thetarange") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "phirange") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "etasca") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = ETASCA = param. controlling # of scatt. dirs used to calculate <cos> etc.") ));
			if (key == "avgnumori") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "avgnumpol") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "xtf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "ytf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "ztf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "beta") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, " BETA = ", " = rotation of target around A1") ));
			if (key == "theta") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, " THETA= ", " = angle between A1 and k") ));
			if (key == "phi") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  PHI = ", " = rotation of A1 around k") ));

			return res;
		}

		void ddOutputSingleObj::findMap(const std::string &line, std::string &res)
		{
			res = "";
			//if (line.find("")!=std::string::npos) res = "";
			if (line.find("DDSCAT ---")!=std::string::npos) res = "version";
			if (line.find("TARGET ---")!=std::string::npos) res = "target";
			if (line.find("--- method of solution")!=std::string::npos) res = "solnmeth";
			if (line.find(" --- prescription for ")!=std::string::npos) res = "polarizability";
			if (line.find("--- shape")!=std::string::npos) res = "shape";
			if (line.find("NAT0")!=std::string::npos) res = "numdipoles";
			if (line.find("= d/aeff for this target")!=std::string::npos) res = "d/aeff";
			if (line.find("= d (physical units)")!=std::string::npos) res = "d";
			if (line.find("effective radius (physical units)")!=std::string::npos) res = "aeff";
			if (line.find("wavelength")!=std::string::npos) res = "wave";
			if (line.find("K*AEFF")!=std::string::npos) res = "k.aeff";
			if (line.find("NAMBIENT")!=std::string::npos) res = "nambient";
			if (line.find("eps.=")!=std::string::npos) res = "neps";
			if (line.find("TOL")!=std::string::npos) res = "tol";
			if (line.find("target axis A1 in Target Frame")!=std::string::npos) res = "a1tgt";
			if (line.find("target axis A2 in Target Frame")!=std::string::npos) res = "a2tgt";
			if (line.find("NAVG")!=std::string::npos) res = "navg";
			if (line.find("k vector (latt. units) in Lab Frame")!=std::string::npos) res = "kveclf";
			if (line.find("k vector (latt. units) in TF")!=std::string::npos) res = "kvectf";
			if (line.find("inc.pol.vec. 1 in LF")!=std::string::npos) res = "incpol1lf";
			if (line.find("inc.pol.vec. 2 in LF")!=std::string::npos) res = "incpol2lf";
			if (line.find("inc.pol.vec. 1 in TF")!=std::string::npos) res = "incpol1tf";
			if (line.find("inc.pol.vec. 2 in TF")!=std::string::npos) res = "incpol2tf"; //
			if (line.find("beta_min")!=std::string::npos) res = "betarange";
			if (line.find("theta_min")!=std::string::npos) res = "thetarange";
			if (line.find("phi_min")!=std::string::npos) res = "phirange";
			if (line.find("ETASCA")!=std::string::npos) res = "etasca";
			if (line.find("target orientations")!=std::string::npos) res = "avgnumori";
			if (line.find("incident polarizations")!=std::string::npos) res = "avgnumpol";
			if (line.find("xmin,xmax")!=std::string::npos) res = "xtf";
			if (line.find("ymin,ymax")!=std::string::npos) res = "ytf";
			if (line.find("zmin,zmax")!=std::string::npos) res = "ztf";
			if (line.find("BETA")!=std::string::npos) res = "beta";
			if (line.find("THETA")!=std::string::npos) res = "theta";
			if (line.find("PHI")!=std::string::npos) res = "phi";
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

