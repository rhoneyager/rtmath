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
#include "ddOriDataParsers.h"
#include "../rtmath/ddscat/ddVersions.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
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
					omit[repeat(pos)[*char_("a-zA-Z.*=,/-()<>^_:")]] >> double_[push_back(phoenix::ref(v), _1)]
					)
					,
					//  End grammar

					space);

				//if (first != last) // fail if we did not get a full match
				//	return false;
				return r;
			}

			/*
			struct ddM
			{
				static const size_t w = 8;
				static void write(std::ostream &out, size_t) 
				{
					// Using formatted io operations
					using std::setw;
					out << "n = (" << setw(w) << m.real() << "," << setw(w) << m.imag()
						<< "), eps.= (" << setw(w) << eps.real() << "," << setw(w) << eps.imag()
						<< ")  |m|kd=" << setw(w) << mkd << " for subs. " << subst << std::endl;
				}
				static void read(std::istream &in)
				{
					std::string str;
					std::getline(in, str);
					// todo: fix listed ranges (stars are correct, ends are not, but w is)
					// all ranges are INCLUSIVE
					// mreal in cols 4-11
					// mimag in cols 13-20
					// eps real in 32-40
					// eps imag in 42-49
					// mkd in 59-66
					// substance number in 78+
					double mre, mim, ere, eim;

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

					if (snums.size() < 6) RTthrow debug::xBadInput("Cannot parse refractive index in ddOutputSingleKeys::ddM");

					using boost::lexical_cast;
					using boost::algorithm::trim_copy;
					mre = lexical_cast<double>(snums[0]);
					mim = lexical_cast<double>(snums[1]);
					ere = lexical_cast<double>(snums[2]);
					eim = lexical_cast<double>(snums[3]);
					mkd = lexical_cast<float>(snums[4]);
					subst = lexical_cast<size_t>(snums[5]);
					m = std::complex<double>(mre, mim);
					eps = std::complex<double>(ere, eim);
				}
			};
			struct ddPolVec {
				static const size_t w = 8;
				static const size_t p = 5;
				
				static void write(std::ostream &out, size_t) 
				{
					// Using formatted io operations
					using std::setw;
					using std::setprecision;
					out << " ( " << setprecision(5) << setw(w) << pols[0].real() << ","
						<< setw(w) << pols[0].imag() << ")(" << setw(w) << pols[1].real()
						<< "," << setw(w) << pols[1].imag() << ")(" << setw(w)
						<< pols[2].real() << "," << setw(w) << pols[2].imag()
						<< ")=inc.pol.vec. " << vecnum << " in ";
					(frame == frameType::LF) ? out << "LF\n" : out << "TF\n";
				}
				static void read(std::istream &in)
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
						else if (csnum.size()) nDone();

						lchar = c;
					}
					nDone();

					if (snums.size() < 7) RTthrow debug::xBadInput(
						"Cannot parse inc.pol.vec. numbers in ddOutputSingleKeys::ddPolVec");

					using boost::lexical_cast;
					using boost::algorithm::trim_copy;
					using std::complex;
					pols[0] = complex<double>(macros::m_atof(snums[0].c_str()), macros::m_atof(snums[1].c_str()));
					pols[1] = complex<double>(macros::m_atof(snums[2].c_str()), macros::m_atof(snums[3].c_str()));
					pols[2] = complex<double>(macros::m_atof(snums[4].c_str()), macros::m_atof(snums[5].c_str()));
					vecnum = lexical_cast<size_t>(snums[6]);
					auto it = str.find_last_of('F');
					if (it == std::string::npos) RTthrow debug::xBadInput(
						"Cannot parse inc.pol.vec. in ddOutputSingleKeys::ddPolVec for frame identifier");
					it--;
					if (str.at(it) == 'L') frame = frameType::LF;
					else if (str.at(it) == 'T') frame = frameType::TF;
					else RTthrow debug::xBadInput(
						"Cannot parse inc.pol.vec. in ddOutputSingleKeys::ddPolVec for frame identifier (b)");
				}
			};
			struct ddRot1d {
				static const size_t w = 7;
				static const size_t p = 3;

				static void write(std::ostream &out, size_t) 
				{
					// Using formatted io operations
					using std::setw;
					using std::setprecision;
					out << setprecision(5) << setw(w) << min << " "
						<< setprecision(5) << setw(w) << max << " = "
						<< fieldname << "_min, " << fieldname << "_max ; " << fieldnamecaps
						<< "=" << n << "\n";
				}
				static void read(std::istream &in)
				{
					std::string str;
					std::getline(in, str);

					// Retrieving three numbers and two strings
					// Thankfully, the fields are nicely aligned
					// min is from position 0 to 8 (not inclusive)
					// max is from 9 to 16
					// fieldname is from 19 to an underscore
					// fieldnamecaps is from 41 to 47
					// n is from 48 to the end of the line
					min = macros::m_atof(str.data(), 8);
					max = macros::m_atof(str.data() + 9, 8);
					fieldname = str.substr(19, str.find_first_of('_', 19) - 19);
					fieldnamecaps = str.substr(41, 6);
					n = (size_t)macros::m_atoi(str.data() + 48);
				}
			};

			*/


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


			template struct simpleNumRev < double >;
			template struct simpleNumRev < size_t >;

			template struct simpleNumCompound < double >;
			template struct simpleNumCompound < size_t >;

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
				min = macros::m_atof(str.data(), 8);
				max = macros::m_atof(str.data() + 9, 8);
				fieldname = str.substr(19, str.find_first_of('_', 19) - 19);
				fieldnamecaps = str.substr(41, 6);
				n = (size_t)macros::m_atoi(str.data() + 48);
			}

			void ddPolVec::write(std::ostream &out, size_t, const std::vector<std::complex<double> > &pols, 
				size_t vecnum, frameType frame)
			{
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				const size_t p = 5, w=8;
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
				pols[0] = complex<double>(macros::m_atof(snums[0].c_str()), macros::m_atof(snums[1].c_str()));
				pols[1] = complex<double>(macros::m_atof(snums[2].c_str()), macros::m_atof(snums[3].c_str()));
				pols[2] = complex<double>(macros::m_atof(snums[4].c_str()), macros::m_atof(snums[5].c_str()));
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
				} else {
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
					else if (csnum.size()) nDone();

					lchar = c;
				}
				nDone();

				using namespace rtmath::macros;
				using boost::algorithm::trim_copy;
				using std::complex;
				v.resize(3);
				v[0] = macros::m_atof(snums[0].c_str());
				v[1] = macros::m_atof(snums[1].c_str());
				v[2] = macros::m_atof(snums[2].c_str());

				axisnum = 0;
				if (str.at(45) == '1') axisnum = 1;
				if (str.at(45) == '2') axisnum = 2;
				if (axisnum)
				{
					if (str.at(50) == 'T') frame = frameType::TF;
					else frame = frameType::LF;
				} else {
					if (str.at(58) == 'T') frame = frameType::TF;
					else frame = frameType::LF;
				}
			}





		}



	}


}
