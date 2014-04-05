#include "Stdafx-ddscat.h"
#include <iostream>
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
#include "../rtmath/ddscat/rotations.h"
#include "../rtmath/units.h"
#include "../rtmath/macros.h"
#include "../rtmath/quadrature.h"
#include "../rtmath/error/error.h"


namespace rtmath
{
	namespace ddscat {
		namespace ddOutputSingleKeys {

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

			//template <>
			//bool parse_string_ddNval(Iterator first, Iterator last, size_t pos, double &val);

			ddM::ddM() : w(8) {}
			ddM::~ddM() {}
			void ddM::write(std::ostream &out, size_t) const 
			{
				// Using formatted io operations
				using std::setw;
				out << "n = (" << setw(w) << m.real() << "," << setw(w) << m.imag()
					<< "), eps.= (" << setw(w) << eps.real() << "," << setw(w) << eps.imag()
					<< ")  |m|kd=" << setw(w) << mkd << " for subs. " << subst << std::endl;
			}
			void ddM::read(std::istream &in) 
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
			std::string ddM::value() const  { return std::string(); }
			std::complex<double> ddM::getM() const { return m; }
			std::complex<double> ddM::getEps() const { return eps; }
			float ddM::getMkd() const { return mkd; }
			template<class Archive>
			void ddM::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("m", m);
				ar & boost::serialization::make_nvp("eps", eps);
				ar & boost::serialization::make_nvp("mkd", mkd);
				ar & boost::serialization::make_nvp("subst", subst);
			}

			ddPolVec::ddPolVec() : w(8), p(5), frame(frameType::LF), vecnum(0)
			{
				pols[0] = std::complex<double>(0, 0);
				pols[1] = std::complex<double>(0, 0);
				pols[2] = std::complex<double>(0, 0);
			}
			ddPolVec::~ddPolVec() {}
			void ddPolVec::write(std::ostream &out, size_t) const 
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
			void ddPolVec::read(std::istream &in) 
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
			std::string ddPolVec::value() const  { return std::string(); }
			std::complex<double> ddPolVec::getPol(size_t n) const { return pols[n]; }
			frameType ddPolVec::getFrame() const { return frame; }
			size_t ddPolVec::getVecnum() const { return vecnum; }
			template<class Archive>
			void ddPolVec::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("vecnum", vecnum);
				ar & boost::serialization::make_nvp("frame", frame);
				ar & boost::serialization::make_nvp("pols", pols);
			}

			ddRot1d::ddRot1d() : w(7), p(3), min(0), max(0), n(0) {}
			ddRot1d::~ddRot1d() {}
			void ddRot1d::write(std::ostream &out, size_t) const 
			{
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				out << setprecision(5) << setw(w) << min << " "
					<< setprecision(5) << setw(w) << max << " = "
					<< fieldname << "_min, " << fieldname << "_max ; " << fieldnamecaps
					<< "=" << n << "\n";
			}
			void ddRot1d::read(std::istream &in) 
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
			std::string ddRot1d::value() const  { return std::string(); }

			template<class Archive>
			void ddRot1d::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("min", min);
				ar & boost::serialization::make_nvp("max", max);
				ar & boost::serialization::make_nvp("n", n);
				ar & boost::serialization::make_nvp("fieldname", fieldname);
				ar & boost::serialization::make_nvp("fieldnamecaps", fieldnamecaps);
			}
		}

		
		
		boost::shared_ptr<ddOutputSingleObj> ddOutputSingleObj::constructObj(const std::string &key)
		{
			using namespace rtmath::ddscat::ddOutputSingleKeys;

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
				if (ddVersions::isVerWithin(version, 73, 0))
				{
					solnmeth = solnmeth_new;
					polarizability = polarizability_new;
				}
				else {
					solnmeth = solnmeth_old;
					polarizability = polarizability_old;
				}
				if (key == "solnmeth") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(solnmeth)));
				if (key == "polarizability") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(polarizability)));
			}
			if (key == "shape") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval("shape")));
			if (key == "numdipoles") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<size_t> >(new ddNval<size_t>(0, "", " = NAT0 = number of dipoles")));
			if (key == "d/aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d/aeff for this target [d=dipole spacing]")));
			if (key == "d") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d (physical units)")));

			if (key == "aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  AEFF= ", " = effective radius (physical units)")));
			if (key == "wave") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  WAVE= ", " = wavelength (in vacuo, physical units)")));
			if (key == "k.aeff") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "K*AEFF= ", " = 2*pi*aeff/lambda")));
			if (key == "nambient") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "NAMBIENT= ", " = refractive index of ambient medium")));

			if (key == "neps") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddM>(new ddM));

			if (key == "tol") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "   TOL= ", " = error tolerance for CCG method")));
			if (key == "a1tgt") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "a2tgt") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "navg") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  NAVG= ", " = (theta,phi) values used in comp. of Qsca,g")));
			if (key == "kveclf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "kvectf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "incpol1lf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));
			if (key == "incpol2lf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));
			if (key == "incpol1tf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));
			if (key == "incpol2tf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));

			if (key == "betarange") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddRot1d>(new ddRot1d));
			if (key == "thetarange") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddRot1d>(new ddRot1d));
			if (key == "phirange") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddRot1d>(new ddRot1d));
			if (key == "etasca") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = ETASCA = param. controlling # of scatt. dirs used to calculate <cos> etc.")));
			if (key == "avgnumori") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "avgnumpol") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "xtf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "ytf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "ztf") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			if (key == "beta") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(2, " BETA = ", " = rotation of target around A1")));
			if (key == "theta") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, " THETA= ", " = angle between A1 and k")));
			if (key == "phi") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(2, "  PHI = ", " = rotation of A1 around k")));

			if (key == "targetperiodicity") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "fmldotline") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "mdef") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
			if (key == "physextent") res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

			res->setKey(key);
			return res;
		}

		void ddOutputSingleObj::findMap(const std::string &line, std::string &res)
		{
			using namespace rtmath::ddscat::ddOutputSingleKeys;

			res = "";
			//if (line.find("")!=std::string::npos) res = "";
			if (line.find("DDSCAT ---") != std::string::npos) res = "version";
			else if (line.find("TARGET ---") != std::string::npos) res = "target";
			// DDSCAT 7.3 writes solnmeth and polarizability
			// lines differently
			else if (line.find("--- method of solution") != std::string::npos || line.find("--- DDA method") != std::string::npos) res = "solnmeth";
			else if (line.find(" --- prescription for ") != std::string::npos || line.find("--- CCG method") != std::string::npos) res = "polarizability";
			else if (line.find("--- shape") != std::string::npos) res = "shape";
			else if (line.find("NAT0") != std::string::npos) res = "numdipoles";
			else if (line.find("= d/aeff for this target") != std::string::npos) res = "d/aeff";
			else if (line.find("= d (physical units)") != std::string::npos) res = "d";
			else if (line.find("physical extent of target volume") != std::string::npos) res = "physextent";
			else if (line.find("effective radius (physical units)") != std::string::npos) res = "aeff";
			else if (line.find("wavelength") != std::string::npos) res = "wave";
			else if (line.find("K*AEFF") != std::string::npos) res = "k.aeff";
			else if (line.find("NAMBIENT") != std::string::npos) res = "nambient";
			else if (line.find("eps.=") != std::string::npos) res = "neps";
			else if (line.find("TOL") != std::string::npos) res = "tol";
			else if (line.find("target axis A1 in") != std::string::npos) res = "a1tgt"; // target or lab frame
			else if (line.find("target axis A2 in") != std::string::npos) res = "a2tgt";
			else if (line.find("NAVG") != std::string::npos) res = "navg";
			else if (line.find("k vector (latt. units) in Lab Frame") != std::string::npos) res = "kveclf";
			else if (line.find("k vector (latt. units) in TF") != std::string::npos) res = "kvectf";
			else if (line.find("inc.pol.vec. 1 in LF") != std::string::npos) res = "incpol1lf";
			else if (line.find("inc.pol.vec. 2 in LF") != std::string::npos) res = "incpol2lf";
			else if (line.find("inc.pol.vec. 1 in TF") != std::string::npos) res = "incpol1tf";
			else if (line.find("inc.pol.vec. 2 in TF") != std::string::npos) res = "incpol2tf"; //
			else if (line.find("beta_min") != std::string::npos) res = "betarange"; // NOTE: else if used because BETA vould conflict with the id
			else if (line.find("theta_min") != std::string::npos) res = "thetarange";
			else if (line.find("phi_min") != std::string::npos) res = "phirange";
			else if (line.find("ETASCA") != std::string::npos) res = "etasca";
			else if (line.find("target orientations") != std::string::npos) res = "avgnumori";
			else if (line.find("incident polarizations") != std::string::npos) res = "avgnumpol";
			else if (line.find("xmin,xmax") != std::string::npos) res = "xtf";
			else if (line.find("ymin,ymax") != std::string::npos) res = "ytf";
			else if (line.find("zmin,zmax") != std::string::npos) res = "ztf";
			else if (line.find("BETA") != std::string::npos) res = "beta";
			else if (line.find("THETA") != std::string::npos) res = "theta";
			else if (line.find("PHI") != std::string::npos) res = "phi";
			else if (line.find("target:") != std::string::npos) res = "targetperiodicity";
			else if (line.find("dot") != std::string::npos) res = "fmldotline";
			else if (line.find("perp to scatt") != std::string::npos) res = "mdef";
		}


		template <class Archive>
		void ddOutputSingle::serialize(Archive & ar, const unsigned int version)
		{

			ar & boost::serialization::make_nvp("objMap", _objMap);
			ar & boost::serialization::make_nvp("statTable", _statTable);
			ar & boost::serialization::make_nvp("statTable_Size_ts", _statTable_Size_ts);
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

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingle);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleObj);

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddM);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddPolVec);

/*
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddver::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddstring::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddtarget::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddSval::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>::serialize);
EXPORTINTERNAL(rtmath::ddscat::ddOutputSingleKeys::ddM::serialize);
*/

