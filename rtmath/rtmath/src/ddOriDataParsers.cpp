#include "Stdafx-ddscat.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/iostreams/filter/newline.hpp>
#include <boost/math/constants/constants.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix_core.hpp>
#include <boost/spirit/include/phoenix_operator.hpp>
#include <boost/spirit/include/phoenix_stl.hpp>
#include <boost/spirit/include/qi_omit.hpp>
#include <boost/spirit/include/qi_repeat.hpp>

#include "../rtmath/ddscat/ddOriData.h"
#include "../rtmath/macros.h"
#include "ddOriDataParsers.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/units.h"

#include "../rtmath/refract.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace ddscat {
		namespace ddOriDataParsers {
			namespace qi = boost::spirit::qi;
			namespace ascii = boost::spirit::ascii;
			namespace phoenix = boost::phoenix;

			/// Parses a set number of doubles mixed with text in a line
			template <typename Iterator>
			bool parse_string_ddNval(Iterator first, Iterator last, size_t pos, double &val)
			{
				using qi::double_;
				using qi::ulong_;
				using qi::char_;
				using qi::phrase_parse;
				using qi::_1;
				using ascii::space;
				using phoenix::push_back;
				using qi::repeat;
				using qi::omit;

				bool r = phrase_parse(first, last,

					//  Begin grammar
					(
					omit[repeat(pos)[*char_("a-zA-Z.*=,/-()<>^_:")]] >> double_
					)
					,
					//  End grammar

					space, val);

				if (first != last) // fail if we did not get a full match
					return false;
				return r;
			}

			template <typename Iterator>
			bool parse_string_ddNvals(Iterator first, Iterator last, size_t pos, std::vector<double> &val)
			{
				using qi::double_;
				using qi::ulong_;
				using qi::char_;
				using qi::phrase_parse;
				using qi::_1;
				using ascii::space;
				using phoenix::push_back;
				using qi::repeat;
				using qi::omit;

				bool r = phrase_parse(first, last,

					//  Begin grammar
					(
					omit[repeat(pos)[*char_("a-zA-Z.*=,/-()<>^_:")]] >> double_[push_back(phoenix::ref(val), _1)]
					)
					,
					//  End grammar

					space);

				//if (first != last) // fail if we did not get a full match
				//	return false;
				return r;
			}

			/** \brief Parses space-separated numbers.
			*
			* \see rtmath::ddscat::ddOutputSingle::readF
			* \see rtmath::ddscat::ddOutputSingle::readMueller
			**/
			template <typename Iterator>
			bool parse_numbers_space(Iterator first, Iterator last, std::vector<double>& v)
			{
				using qi::double_;
				using qi::phrase_parse;
				using qi::_1;
				using ascii::space;
				using phoenix::push_back;

				bool r = phrase_parse(first, last,

					//  Begin grammar
					(
					*double_
					)
					,
					//  End grammar

					space, v);

				if (first != last) // fail if we did not get a full match
					return false;
				return r;
			}

			void version::write(std::ostream &out, size_t v){
				out << " DDSCAT --- ";
				out << rtmath::ddscat::ddVersions::getVerAvgHeaderString(v);
				out << std::endl;
			}

			size_t version::read(std::istream &in, size_t)
			{
				std::string lin;
				std::getline(in, lin);
				return rtmath::ddscat::ddVersions::getVerId(lin);
			}

			void simpleString::write(std::ostream &out, size_t, const std::string &s, const std::string &p)
			{
				out << " " << p << " --- ";
				out << s << std::endl;
			}

			void simpleString::read(std::istream &in, std::string &s)
			{
				std::string lin;
				std::getline(in, lin);
				size_t p = lin.find("---");
				s = lin.substr(p + 3);
				// Remove any leading and lagging spaces
				// Not all Liu avg files are correct in this respect
				boost::algorithm::trim(s);
			}

			void simpleStringRev::write(std::ostream &out, size_t, const std::string &s, const std::string &p)
			{
				out << " " << s << " --- ";
				out << p << std::endl;
			}

			void simpleStringRev::read(std::istream &in, std::string &s)
			{
				std::string lin;
				std::getline(in, lin);
				size_t p = lin.find("---");
				s = lin.substr(0, p - 1);
				// Remove any leading and lagging spaces
				// Not all Liu avg files are correct in this respect
				boost::algorithm::trim(s);
			}

			template struct simpleNumRev < double > ;
			template struct simpleNumRev < size_t > ;

			template struct simpleNumCompound < double > ;
			template struct simpleNumCompound < size_t > ;

			void refractive::write(std::ostream &out, size_t, size_t inum, const std::complex<double> &m, double k, double d)
			{
				std::complex<double> eps;
				rtmath::refract::mToE(m, eps);
				double mkd = abs(m) * k * d;
				const size_t wd = 8;
				out << "n= ( "
					<< std::setw(wd) << m.real() << std::setw(0) << " ,  "
					<< std::setw(wd) << m.imag() << std::setw(0) << "),  eps.= (  "
					<< std::setw(wd) << eps.real() << std::setw(0) << " ,  "
					<< std::setw(wd) << eps.imag() << std::setw(0) << ")  |m|kd="
					<< std::setw(wd) << mkd << std::setw(0) << " for subs. " << inum << std::endl;
			}

			void refractive::read(std::istream &in, size_t &subst, std::complex<double> &m)
			{
				std::string str;
				std::getline(in, str);
				read(str, subst, m);
			}

			void refractive::read(const std::string &str, size_t &subst, std::complex<double> &m)
			{
				// todo: fix listed ranges (stars are correct, ends are not, but w is)
				// all ranges are INCLUSIVE
				// mreal in cols 4-11
				// mimag in cols 13-20
				// eps real in 32-40
				// eps imag in 42-49
				// mkd in 59-66
				// substance number in 78+
				double mre, mim, ere, eim, mkd;
				std::complex<double> eps;

				// Different ddscat versions write their numbers differently. There are
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

				if (snums.size() < 6) RTthrow debug::xBadInput("Cannot parse refractive index in ddOriDataParsers::refractive");

				using namespace rtmath::macros;
				for (size_t i = 0; i < 6; ++i)
					boost::algorithm::trim(snums[i]);
				mre = fastCast<double>(snums[0]);
				mim = fastCast<double>(snums[1]);
				ere = fastCast<double>(snums[2]);
				eim = fastCast<double>(snums[3]);
				mkd = fastCast<float>(snums[4]);
				subst = fastCast<size_t>(snums[5]);
				m = std::complex<double>(mre, mim);
				eps = std::complex<double>(ere, eim);
			}

			void ddRot1d::write(std::ostream &out, size_t, const std::string &fieldname,
				double min, double max, size_t n, const std::string &fieldnamecaps)
			{
				const size_t w = 7;
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				out << setprecision(5) << setw(w) << min << " "
					<< setprecision(5) << setw(w) << max << " = "
					<< fieldname << "_min, " << fieldname << "_max ; " << fieldnamecaps
					<< "=" << n << "\n";
			}

			void ddRot1d::read(std::istream &in, std::string &fieldname, double &min, double &max, size_t &n)
			{
				std::string str;
				std::getline(in, str);
				std::string fieldnamecaps;

				// Retrieving three numbers and two strings
				// Thankfully, the fields are nicely aligned
				// min is from position 0 to 8 (not inclusive)
				// max is from 9 to 16
				// fieldname is from 19 to an underscore
				// fieldnamecaps is from 41 to 47
				// n is from 48 to the end of the line
				min = macros::m_atof<double>(str.data(), 8);
				max = macros::m_atof<double>(str.data() + 9, 8);
				fieldname = str.substr(19, str.find_first_of('_', 19) - 19);
				fieldnamecaps = str.substr(41, 6);
				n = (size_t)macros::m_atoi<size_t>(str.data() + 48);
			}

			void ddPolVec::write(std::ostream &out, size_t, const std::vector<std::complex<double> > &pols,
				size_t vecnum, frameType frame)
			{
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				const size_t p = 5, w = 8;
				out << " ( " << setprecision(p) << setw(w) << pols[0].real() << ","
					<< setw(w) << pols[0].imag() << ")(" << setw(w) << pols[1].real()
					<< "," << setw(w) << pols[1].imag() << ")(" << setw(w)
					<< pols[2].real() << "," << setw(w) << pols[2].imag()
					<< ")=inc.pol.vec. " << vecnum << " in ";
				(frame == frameType::LF) ? out << "LF\n" : out << "TF\n";
			}
			void ddPolVec::read(std::istream &in, std::vector<std::complex<double> > &pols, size_t &vecnum, frameType &frame)
			{
				std::string str;
				std::getline(in, str);

				// The first six numbers are doubles. The seventh is a size_t.
				// The last two characters describe the frame.
				std::vector<std::string> snums;
				snums.reserve(30);
				char lchar = 0; // last character
				char cchar = 0; // current character
				std::string csnum; // current numeric string
				csnum.reserve(200);

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
					else if (c == '-') csnum.push_back(c);
					else if (csnum.size()) nDone();

					lchar = c;
				}
				nDone();

				if (snums.size() < 7) RTthrow debug::xBadInput(
					"Cannot parse inc.pol.vec. numbers in ddOriDataParsers::ddPolVec");

				using namespace rtmath::macros;
				using boost::algorithm::trim_copy;
				using std::complex;
				pols.resize(3);
				pols[0] = complex<double>(macros::m_atof<double>(snums[0].c_str()), macros::m_atof<double>(snums[1].c_str()));
				pols[1] = complex<double>(macros::m_atof<double>(snums[2].c_str()), macros::m_atof<double>(snums[3].c_str()));
				pols[2] = complex<double>(macros::m_atof<double>(snums[4].c_str()), macros::m_atof<double>(snums[5].c_str()));
				vecnum = fastCast<size_t>(snums[6]);
				auto it = str.find_last_of('F');
				if (it == std::string::npos) RTthrow debug::xBadInput(
					"Cannot parse inc.pol.vec. in ddOriDataParsers::ddPolVec for frame identifier");
				it--;
				if (str.at(it) == 'L') frame = frameType::LF;
				else if (str.at(it) == 'T') frame = frameType::TF;
				else RTthrow debug::xBadInput(
					"Cannot parse inc.pol.vec. in ddOriDataParsers::ddPolVec for frame identifier (b)");
			}

			void ddAxisVec::write(std::ostream &out, size_t, const std::vector<double > &v,
				size_t axisnum, frameType frame)
			{
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				const size_t p = 5, w = 8;
				out << "( " << setprecision(p) << setw(w) << v[0] << "  "
					<< setprecision(p) << setw(w) << v[1] << "  "
					<< setprecision(p) << setw(w) << v[2] << "  "
					<< " ) = ";
				if (axisnum)
				{
					out << "target axis A" << axisnum << " in ";
				}
				else {
					out << "k vector (latt. units) in ";
				}
				(frame == frameType::LF) ? out << "Lab Frame\n" : out << "Target Frame\n";
			}
			void ddAxisVec::read(std::istream &in, std::vector<double> &v, size_t &axisnum, frameType &frame)
			{
				std::string str;
				std::getline(in, str);

				// The first three numbers are doubles.
				std::vector<std::string> snums;
				snums.reserve(30);
				char lchar = 0; // last character
				char cchar = 0; // current character
				std::string csnum; // current numeric string
				csnum.reserve(200);

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
					else if (c == '-') csnum.push_back(c);
					else if (csnum.size()) nDone();

					lchar = c;
				}
				nDone();

				using namespace rtmath::macros;
				using boost::algorithm::trim_copy;
				using std::complex;
				v.resize(3);
				v[0] = macros::m_atof<double>(snums[0].c_str());
				v[1] = macros::m_atof<double>(snums[1].c_str());
				v[2] = macros::m_atof<double>(snums[2].c_str());

				axisnum = 0;
				if (str.at(45) == '1') axisnum = 1;
				if (str.at(45) == '2') axisnum = 2;
				if (axisnum)
				{
					if (str.at(50) == 'T') frame = frameType::TF;
					else frame = frameType::LF;
				}
				else {
					if (str.at(58) == 'T') frame = frameType::TF;
					else frame = frameType::LF;
				}
			}

			void ddPhysExtent::write(std::ostream &out, size_t, const double a, const double b,
				char axisname)
			{
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				const size_t p = 6, w = 12;
				out << "  " << setprecision(p) << setw(w) << std::right << a << "  "
					<< setprecision(p) << setw(w) << std::right << b << " = " << axisname << "min,"
					<< axisname << "max (physical units)\n";
			}
			void ddPhysExtent::read(std::istream &in, double &a, double &b, char &axisname)
			{
				std::string str;
				std::getline(in, str);

				// The first two numbers are doubles.
				std::vector<std::string> snums;
				snums.reserve(30);
				char lchar = 0; // last character
				char cchar = 0; // current character
				std::string csnum; // current numeric string
				csnum.reserve(200);

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
					else if (c == '-') csnum.push_back(c);
					else if (csnum.size()) nDone();

					lchar = c;
				}
				nDone();

				using namespace rtmath::macros;
				using boost::algorithm::trim_copy;
				a = macros::m_atof<double>(snums[0].c_str());
				b = macros::m_atof<double>(snums[1].c_str());
				axisname = str.at(31);
			}
		}

		void ddOriData::readMuellerDDSCAT(std::istream &in)
		{
			using namespace std;
			using namespace rtmath::ddscat::ddOriDataParsers;

			auto od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			//auto &os = _parent.oridata_s.at(_row);
			//auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);

			// The frequency is needed when reading this matrix
			const double f = freq();

			string lin;
			mMuellerIndices mIndices;// = _muellerMap;
			mIndices.clear();
			vector<double> vals;
			vals.reserve(10);

			while (in.good())
			{
				std::getline(in, lin);
				// Parse the string to get rid of spaces. This is used to determine
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (!lin.size()) continue;
				//std::cerr << lin << std::endl;
				// TODO: parse the header line to get the list of matrix entries known
				// TODO: use symmetry relationships in a depGraph to get the other
				// mueller matrix entries.

				// Expecting the first line to begin with theta phi Pol. ...
				if (std::isalpha(lin.at(0)))
				{
					typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
					boost::char_separator<char> sep("\t ");
					tokenizer t(lin, sep);
					size_t i = 0; // Column number
					for (auto it = t.begin(); it != t.end(); ++it, ++i)
					{
						//std::cerr << "\t i: " << i << " it: " << *it << std::endl;
						// Mueller entry columns have a '_'
						size_t loc = it->find("_");
						if (loc == string::npos) continue;
						//std::cerr << it->substr(loc+1) << std::endl;
						size_t id = macros::m_atoi<size_t>(it->substr(loc + 1).c_str());
						size_t row = (id / 10) - 1; // Annoying start at 1...
						size_t col = (id % 10) - 1;
						//std::cerr << "mIndices loc: " << loc << " id: " << id << " i: " << i << " row: " << row << " col: " << col << std::endl;
						mIndices[i] = std::pair<size_t, size_t>(row, col);
					}

				}
				else {
					// Parse the Mueller entries
					//std::cerr << "Parsing " << lin << std::endl;
					// TODO: check this
					// The ordering is theta, phi, polarization, and then the
					// relevant matrix entries
					// theta phi Pol. S_11 S_12 S_21 S_22 S_31 S_41
					vals.clear();
					if (!parse_numbers_space(lin.begin(), lin.end(), vals))
						throw debug::xBadInput("Cannot parse Mueller entry");

					//for (auto it = t.begin(); it != t.end(); ++it)
					//	vals.push_back(rtmath::macros::m_atof(it->data(), it->size())); // Speedup using my own atof
					//vals.push_back(boost::lexical_cast<double>(*it));
					// ddScattMatrixF constructor takes frequency (GHz) and phi
					boost::shared_ptr<ddScattMatrixP> mat(new ddScattMatrixP(f, vals[0], vals[1]));
					ddScattMatrix::PnnType P;

					size_t j = 0;
					for (auto ot = mIndices.begin(); ot != mIndices.end(); ++ot, ++j)
					{
						P(ot->first, ot->second) = vals[j]; // See Mueller header read
					}
					mat->setP(P);
					mat->polLin(vals[2]);

					boost::shared_ptr<const ddScattMatrix> matC =
						boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

					/// \note Actual read of mueller matrix entries disabled
					std::cerr << "Actual read of mueller matrix entries disabled\n";
					RTthrow debug::xUnimplementedFunction();
					//_scattMatricesRaw.push_back(matC);
					//std::cerr << _scattMatricesRaw.size() << " elements\n";
				}
			}

			//_statTable_Size_ts.at(stat_entries_size_ts::NUMP) = _scattMatricesRaw.size();
		}

		void ddOriData::readF_DDSCAT(std::istream &in,
			boost::shared_ptr<const ddScattMatrixConnector> eProvider)
		{
			using namespace std;
			using namespace rtmath::ddscat::ddOriDataParsers;

			auto od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			//auto &os = _parent.oridata_s.at(_row);
			//auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);

			// The frequency is needed when reading this matrix
			const double f = freq();

			string lin;

			std::vector<double> vals;
			vals.reserve(10);

			while (in.good())
			{
				std::getline(in, lin);
				if (lin == "") return;
				// Parse the string to get rid of spaces. This is used to determine
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (std::isalpha(lin.at(0))) continue;

				vals.clear();
				if (!parse_numbers_space(lin.begin(), lin.end(), vals))
					throw debug::xBadInput("Cannot parse F entry");

				// ddScattMatrixF constructor takes frequency (GHz) and phi
				//boost::shared_ptr<ddScattMatrixF> mat(new ddScattMatrixF
				//	(freq, vals[0], vals[1], 0, 0, eProvider));
				ddScattMatrixF mat(f, vals[0], vals[1], 0, 0, eProvider);
				ddScattMatrix::FType fs;
				fs(0, 0) = complex<double>(vals[2], vals[3]);
				fs(1, 0) = complex<double>(vals[4], vals[5]);
				fs(0, 1) = complex<double>(vals[6], vals[7]);
				fs(1, 1) = complex<double>(vals[8], vals[9]);
				mat.setF(fs);

				//boost::shared_ptr<const ddScattMatrix> matC =
				//	boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

				//_scattMatricesRaw.push_back(matC);
				_scattMatricesRaw.push_back(mat);
			}

			//_statTable_Size_ts.at(stat_entries_size_ts::NUMF) = _scattMatricesRaw.size();
		}

		void ddOriData::readS_ADDA(std::istream &in)
		{
			throw rtmath::debug::xUnimplementedFunction();
			/*
			using namespace std;
			using namespace rtmath::ddscat::ddOriDataParsers;

			auto od = _parent.oridata_d.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_DOUBLES>(_row, 0);
			//auto &os = _parent.oridata_s.at(_row);
			//auto &oi = _parent.oridata_i.block<1, ddOutput::stat_entries::NUM_STAT_ENTRIES_INTS>(_row, 0);

			// The frequency is needed when reading this matrix
			const double f = freq();

			string lin;

			std::vector<double> vals;
			vals.reserve(10);

			while (in.good())
			{
				std::getline(in, lin);
				if (lin == "") return;
				// Parse the string to get rid of spaces. This is used to determine
				// if we are still in the S matrix header or in the actual data
				boost::trim(lin);
				if (std::isalpha(lin.at(0))) continue;

				vals.clear();
				if (!parse_numbers_space(lin.begin(), lin.end(), vals))
					throw debug::xBadInput("Cannot parse S entry");

				// TODO: Add check to see if phi is a column or not!!!!!

				// ddScattMatrixF constructor takes frequency (GHz) and phi
				//boost::shared_ptr<ddScattMatrixF> mat(new ddScattMatrixF
				//	(freq, vals[0], vals[1], 0, 0, eProvider));
				ddScattMatrixS mat(f, vals[0], vals[1], 0, 0, eProvider);
				ddScattMatrix::FType fs;
				fs(0, 0) = complex<double>(vals[2], vals[3]);
				fs(1, 0) = complex<double>(vals[4], vals[5]);
				fs(0, 1) = complex<double>(vals[6], vals[7]);
				fs(1, 1) = complex<double>(vals[8], vals[9]);
				mat.setF(fs);

				//boost::shared_ptr<const ddScattMatrix> matC =
				//	boost::dynamic_pointer_cast<const ddScattMatrix>(mat);

				//_scattMatricesRaw.push_back(matC);
				_scattMatricesRaw.push_back(mat);
			}

			//_statTable_Size_ts.at(stat_entries_size_ts::NUMF) = _scattMatricesRaw.size();
			*/
		}

	}
}
