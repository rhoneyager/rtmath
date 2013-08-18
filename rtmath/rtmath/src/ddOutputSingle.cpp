#include "Stdafx-ddscat.h"
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
#include <boost/iostreams/filter/newline.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <Ryan_Serialization/serialization.h>
#include "../rtmath/ddscat/ddOutputSingle.h"
#include "../rtmath/ddscat/ddScattMatrix.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/units.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"
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

namespace rtmath
{
	namespace ddscat {
		namespace ddOutputSingleKeys {
			class ddver : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddver() { _version = rtmath::ddscat::ddVersions::getDefaultVer(); }
				virtual ~ddver() {}
				virtual void write(std::ostream &out, size_t) const override
				{
					out << " DDSCAT --- ";
					out << rtmath::ddscat::ddVersions::getVerAvgHeaderString(_version);
					out << std::endl;
				}
				virtual void read(std::istream &in) override
				{
					std::string lin;
					std::getline(in,lin);
					_version = rtmath::ddscat::ddVersions::getVerId(lin);
				}
				size_t _version;
				size_t version() const { return _version; }
				void version(size_t n) { _version = n; }
				virtual std::string value() const override
				{
					std::ostringstream out;
					out << _version;
					return out.str();
				}
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("version", _version);
				}
			};
			class ddstring : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddstring() {}
				virtual ~ddstring() {}
				virtual void write(std::ostream &out, size_t) const override
				{
					out << s << std::endl;
				}
				virtual void read(std::istream &in) override
				{
					std::getline(in,s);
				}
				virtual std::string value() const override { return s; }
				std::string s;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("str", s);
				}
			};
			class ddtarget : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddtarget() {}
				virtual ~ddtarget() {}
				virtual void write(std::ostream &out, size_t) const override
				{
					out << " TARGET --- ";
					out << s << std::endl;
				}
				virtual void read(std::istream &in) override
				{
					std::string lin;
					std::getline(in,lin);
					size_t p = lin.find("---");
					s = lin.substr(p+3);
					// Remove any leading and lagging spaces
					// Not all Liu avg files are correct in this respect
					boost::algorithm::trim(s);
				}
				void setTarget(const std::string &n) { s = n; }
				virtual std::string value() const override { return s; }
				std::string s;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("target", s);
				}
			};
			class ddSval : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddSval(const std::string &tail = "") {this->tail = tail;}
				virtual ~ddSval() {}
				virtual void write(std::ostream &out, size_t) const override
				{
					out << s << "--- " << tail << std::endl;
				}
				virtual void read(std::istream &in) override
				{
					std::string lin;
					std::getline(in,lin);
					size_t p = lin.find("--- ");
					s = lin.substr(0,p);
				}
				std::string s, tail;
				virtual std::string value() const override { return s; }
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("s", s);
					ar & boost::serialization::make_nvp("tail", tail);
				}
			};

			template <class T>
			class ddNval : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddNval(size_t pos = 0, const std::string &head = "", const std::string &tail = "") {this->pos = pos; this->head = head; this->tail = tail;}
				virtual ~ddNval() {}
				virtual void write(std::ostream &out, size_t) const override
				{
					out << head << val << tail << std::endl;
				}
				virtual std::string value() const override
				{
					std::ostringstream out;
					out << val;
					return out.str();
				}
				virtual void read(std::istream &in) override
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
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("pos", pos);
					ar & boost::serialization::make_nvp("val", val);
					ar & boost::serialization::make_nvp("head", head);
					ar & boost::serialization::make_nvp("tail", tail);
				}
			};

			class ddM : public ::rtmath::ddscat::ddOutputSingleObj
			{
			public:
				ddM() : w(8) {}
				virtual void write(std::ostream &out, size_t) const override
				{
					// Using formatted io operations
					using std::setw;
					out << "n = (" << setw(w) << m.real() << "," << setw(w) << m.imag()
						<< "), eps.= (" << setw(w) << eps.real() << "," << setw(w) << eps.imag()
						<< ")  |m|kd=" << setw(w) << mkd << " for subs. " << subst << std::endl;
				}
				virtual void read(std::istream &in) override
				{
					std::string str;
					std::getline(in,str);
					// todo: fix listed ranges (stars are correct, ends are not, but w is)
					// all ranges are INCLUSIVE
					// mreal in cols 4-11
					// mimag in cols 13-20
					// eps real in 32-40
					// eps imag in 42-49
					// mkd in 59-66
					// substance number in 78+
					double mre, mim, ere, eim;

					// Differernt ddscat versions write their numbers differently. There are 
					// no set ranges. I'll use a slow approach to sieve them out from the 
					// surrounding other symbols. Valid numeric digits are numbers and '.'
					std::vector<std::string> snums;
					char lchar = 0; // last character
					char cchar = 0; // current character
					std::string csnum; // current numeric string

					auto nDone = [&]()
					{
						if (csnum.size()) snums.push_back(csnum);
						csnum.clear();
					};

					for (const auto &c : str)
					{
						cchar = c;
						if (std::isdigit(c)) csnum.push_back(c);
						else if (c == '.' && std::isdigit(lchar)) csnum.push_back(c);
						else if (csnum.size()) nDone();

						lchar = c;
					}
					nDone();

					if (snums.size() < 6) throw debug::xBadInput("Cannot parse refractive index in ddOutputSingleKeys::ddM");

					using boost::lexical_cast;
					using boost::algorithm::trim_copy;
					mre = lexical_cast<double>(snums[0]);
					mim = lexical_cast<double>(snums[1]);
					ere = lexical_cast<double>(snums[2]);
					eim = lexical_cast<double>(snums[3]);
					mkd = lexical_cast<float>(snums[4]);
					subst = lexical_cast<size_t>(snums[5]);
					m = std::complex<double>(mre,mim);
					eps = std::complex<double>(ere,eim);
				}
				virtual std::string value() const override { return std::string(); }
				std::complex<double> getM() const { return m; }
				std::complex<double> getEps() const { return eps; }
				float getMkd() const { return mkd; }
				std::complex<double> m, eps;
				float mkd;
				const size_t w;
				size_t subst;
			private:
				friend class boost::serialization::access;
				template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp(
						"base",
						boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
					ar & boost::serialization::make_nvp("m", m);
					ar & boost::serialization::make_nvp("eps", eps);
					ar & boost::serialization::make_nvp("mkd", mkd);
					ar & boost::serialization::make_nvp("subst", subst);
				}
			};
		}
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingle);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleObj);

BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddver);
BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddstring);
BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddtarget);
BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddSval);
BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>);
BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>);
BOOST_CLASS_EXPORT(rtmath::ddscat::ddOutputSingleKeys::ddM);

/*
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddver::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddstring::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddtarget::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddSval::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddM::serialize);
*/

namespace rtmath {
	namespace ddscat {
		using namespace rtmath::ddscat::ddOutputSingleKeys;

		std::string getStatNameFromId(stat_entries id)
		{
#define str(s) #s
#define CHECK(x) if(id==x) return str(x)
			/*
			*			QEXT1,QABS1,QSCA1,G11,G21,QBK1,QPHA1,
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

		ddOutputSingleObj::ddOutputSingleObj() { }

		ddOutputSingleObj::~ddOutputSingleObj() { }

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
				// Parse the string to get rid of spaces. This is used to determine 
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (!lin.size()) continue;
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
						//std::cerr << "\t i: " << i << " it: " << *it << std::endl;
						// Mueller entry columns have a '_'
						size_t loc = it->find("_");
						if (loc == string::npos) continue;
						//std::cerr << it->substr(loc+1) << std::endl;
						size_t id = (size_t) atoi(it->substr(loc+1).c_str());
						size_t row = (id / 10) - 1; // Annoying start at 1...
						size_t col = (id % 10) - 1;
						//std::cerr << "mIndices loc: " << loc << " id: " << id << " i: " << i << " row: " << row << " col: " << col << std::endl;
						mIndices[i] = std::pair<size_t,size_t>(row,col);
					}

					// TODO: add function that generates the correct mueller relations from here
#pragma message("Warning: ddOutputSingle needs the Mueller matrix filling routine")
				} else {
					// Parse the Mueller entries
					//std::cerr << "Parsing " << lin << std::endl;
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
					//std::cerr << _scattMatricesRaw.size() << " elements\n";
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
					if (lin == "" || lin == "\r") continue;
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
				if (m->id() == rtmath::ddscat::P) ++i;
			return i;
		}

		size_t ddOutputSingle::numF() const
		{
			size_t i = 0;
			for (const auto &m : _scattMatricesRaw)
				if (m->id() == rtmath::ddscat::F) ++i;
			return i;
		}

		void ddOutputSingle::getScattMatrices(scattMatricesContainer& c) const
		{
			c = _scattMatricesRaw;
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
				out << setprecision(5) << setw(8) << sf->pol() << " ";
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
			out << "----- physical extent of target volume in Target Frame ------" << endl;
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
			// DDSCAT 7.3+ write the solution method and
			// polarizability differently
			// TODO: give these a specialized version-dependent
			// string class that holds a pointer to the writer?
#pragma message("Handle solnmeth and polarizability version differences when writing")
			{
				const std::string solnmeth_old = "method of solution";
				const std::string solnmeth_new = "DDA method";
				const std::string polarizability_old = "prescription for polarizabilities";
				const std::string polarizability_new = "CCG method";
				std::string solnmeth, polarizability;
				size_t version = 0;
				if (!version) version = ddVersions::getDefaultVer();
				if (ddVersions::isVerWithin(version,73,0))
				{
					solnmeth = solnmeth_new;
					polarizability = polarizability_new;
				} else {
					solnmeth = solnmeth_old;
					polarizability = polarizability_old;
				}
				if (key == "solnmeth") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(solnmeth) ));
				if (key == "polarizability") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(polarizability) ));
			}
			if (key == "shape") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval("shape") ));
			if (key == "numdipoles") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<size_t> >(new ddNval<size_t>(0, "", " = NAT0 = number of dipoles") ));
			if (key == "d/aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d/aeff for this target [d=dipole spacing]") ));
			if (key == "d") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d (physical units)") ));

			if (key == "aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  AEFF= ", " = effective radius (physical units)") ));
			if (key == "wave") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  WAVE= ", " = wavelength (in vacuo, physical units)") ));
			if (key == "k.aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "K*AEFF= ", " = 2*pi*aeff/lambda") ));
			if (key == "nambient") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "NAMBIENT= ", " = refractive index of ambient medium") ));

			if (key == "neps") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddM>(new ddM));

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
			else if (line.find("TARGET ---")!=std::string::npos) res = "target";
			// DDSCAT 7.3 writes solnmeth and polarizability
			// lines differently
			else if (line.find("--- method of solution")!=std::string::npos || line.find("--- DDA method")!=std::string::npos) res = "solnmeth";
			else if (line.find(" --- prescription for ")!=std::string::npos || line.find("--- CCG method")!=std::string::npos) res = "polarizability";
			else if (line.find("--- shape")!=std::string::npos) res = "shape";
			else if (line.find("NAT0")!=std::string::npos) res = "numdipoles";
			else if (line.find("= d/aeff for this target")!=std::string::npos) res = "d/aeff";
			else if (line.find("= d (physical units)")!=std::string::npos) res = "d";
			else if (line.find("effective radius (physical units)")!=std::string::npos) res = "aeff";
			else if (line.find("wavelength")!=std::string::npos) res = "wave";
			else if (line.find("K*AEFF")!=std::string::npos) res = "k.aeff";
			else if (line.find("NAMBIENT")!=std::string::npos) res = "nambient";
			else if (line.find("eps.=")!=std::string::npos) res = "neps";
			else if (line.find("TOL")!=std::string::npos) res = "tol";
			else if (line.find("target axis A1 in Target Frame")!=std::string::npos) res = "a1tgt";
			else if (line.find("target axis A2 in Target Frame")!=std::string::npos) res = "a2tgt";
			else if (line.find("NAVG")!=std::string::npos) res = "navg";
			else if (line.find("k vector (latt. units) in Lab Frame")!=std::string::npos) res = "kveclf";
			else if (line.find("k vector (latt. units) in TF")!=std::string::npos) res = "kvectf";
			else if (line.find("inc.pol.vec. 1 in LF")!=std::string::npos) res = "incpol1lf";
			else if (line.find("inc.pol.vec. 2 in LF")!=std::string::npos) res = "incpol2lf";
			else if (line.find("inc.pol.vec. 1 in TF")!=std::string::npos) res = "incpol1tf";
			else if (line.find("inc.pol.vec. 2 in TF")!=std::string::npos) res = "incpol2tf"; //
			else if (line.find("beta_min")!=std::string::npos) res = "betarange"; // NOTE: else if used because BETA vould conflict with the id
			else if (line.find("theta_min")!=std::string::npos) res = "thetarange";
			else if (line.find("phi_min")!=std::string::npos) res = "phirange";
			else if (line.find("ETASCA")!=std::string::npos) res = "etasca";
			else if (line.find("target orientations")!=std::string::npos) res = "avgnumori";
			else if (line.find("incident polarizations")!=std::string::npos) res = "avgnumpol";
			else if (line.find("xmin,xmax")!=std::string::npos) res = "xtf";
			else if (line.find("ymin,ymax")!=std::string::npos) res = "ytf";
			else if (line.find("zmin,zmax")!=std::string::npos) res = "ztf";
			else if (line.find("BETA")!=std::string::npos) res = "beta";
			else if (line.find("THETA")!=std::string::npos) res = "theta";
			else if (line.find("PHI")!=std::string::npos) res = "phi";
		}

		bool ddOutputSingleObj::operator==(const ddOutputSingleObj &rhs) const
		{
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
			// The derived classes handle everything!
			// All of their serialization methods are private and exist only in this file.
		}

		EXPORTINTERNAL(rtmath::ddscat::ddOutputSingle::serialize);
		EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleObj::serialize);
	}
}

