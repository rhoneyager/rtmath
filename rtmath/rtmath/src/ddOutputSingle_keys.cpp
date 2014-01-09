#include "Stdafx-ddscat.h"
#include <boost/algorithm/string/trim.hpp>
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
#include "../rtmath/macros.h"
#include "../rtmath/error/error.h"

/// Internal namespace for the reader parsers
namespace {
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
}


namespace rtmath
{
	namespace ddscat {
		namespace ddOutputSingleKeys {

			ddOutputSingleObj::ddOutputSingleObj(mapKeys key) : key(key) {}

			ddOutputSingleObj::~ddOutputSingleObj() {}

			boost::shared_ptr<ddOutputSingleObj> ddOutputSingleObj::clone() const
			{
				boost::shared_ptr<ddOutputSingleObj> res = constructObj(key);
				std::ostringstream out;
				write(out);
				std::string sin = out.str();
				std::istringstream in(sin);
				res->read(in);
				return res;
			}


			boost::shared_ptr<ddOutputSingleObj> ddOutputSingleObj::constructObj(mapKeys key)
			{
				// Given the given key, construct the appropriate object.
				// Used in the reading algorithms.
				boost::shared_ptr<ddOutputSingleObj> res;

				if (key == mapKeys::VERSION) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddver>(new ddver));
				if (key == mapKeys::TARGET) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddtarget>(new ddtarget));
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
					if (key == mapKeys::SOLNMETH) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(solnmeth)));
					if (key == mapKeys::POLARIZABILITY) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(polarizability)));
				}
				if (key == mapKeys::SHAPE) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddSval>(new ddSval(mapKeys::SHAPE)));
				if (key == mapKeys::NUMDIPOLES) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<size_t> >(new ddNval<size_t>(0, "", " = NAT0 = number of dipoles")));
				if (key == mapKeys::D_AEFF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d/aeff for this target [d=dipole spacing]")));
				if (key == mapKeys::D) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = d (physical units)")));

				if (key == mapKeys::AEFF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  AEFF= ", " = effective radius (physical units)")));
				if (key == mapKeys::WAVE) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  WAVE= ", " = wavelength (in vacuo, physical units)")));
				if (key == mapKeys::K_AEFF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "K*AEFF= ", " = 2*pi*aeff/lambda")));
				if (key == mapKeys::NAMBIENT) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "NAMBIENT= ", " = refractive index of ambient medium")));

				if (key == mapKeys::NEPS) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddM>(new ddM));

				if (key == mapKeys::TOL) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "   TOL= ", " = error tolerance for CCG method")));
				if (key == mapKeys::A1TGT) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::A2TGT) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

				if (key == mapKeys::NAVG) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, "  NAVG= ", " = (theta,phi) values used in comp. of Qsca,g")));
				if (key == mapKeys::KVECLF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::KVECTF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::INCPOL1LF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));
				if (key == mapKeys::INCPOL2LF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));
				if (key == mapKeys::INCPOL1TF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));
				if (key == mapKeys::INCPOL2TF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddPolVec>(new ddPolVec));

				if (key == mapKeys::BETARANGE) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::THETARANGE) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::PHIRANGE) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::ETASCA) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(0, "", " = ETASCA = param. controlling # of scatt. dirs used to calculate <cos> etc.")));
				if (key == mapKeys::AVGNUMORI) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::AVGNUMPOL) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

				if (key == mapKeys::XTF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::YTF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::ZTF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

				if (key == mapKeys::BETA) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(2, " BETA = ", " = rotation of target around A1")));
				if (key == mapKeys::THETA) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(1, " THETA= ", " = angle between A1 and k")));
				if (key == mapKeys::PHI) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddNval<double> >(new ddNval<double>(2, "  PHI = ", " = rotation of A1 around k")));

				if (key == mapKeys::TARGETPERIODICITY) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::FMLDOTLINE) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::MDEF) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));
				if (key == mapKeys::PHYSEXTENT) res = boost::dynamic_pointer_cast<ddOutputSingleObj>(boost::shared_ptr<ddstring>(new ddstring));

				res->setKey(key);
				return res;
			}

			boost::shared_ptr<ddOutputSingleObj> ddOutputSingleObj::findMap(const std::string &line)
			{
				mapKeys res = mapKeys::UNKNOWN;
				//if (line.find("")!=std::string::npos) res = "";
				if (line.find("DDSCAT ---") != std::string::npos) res = mapKeys::VERSION;
				else if (line.find("TARGET ---") != std::string::npos) res = mapKeys::TARGET;
				// DDSCAT 7.3 writes solnmeth and polarizability
				// lines differently
				else if (line.find("--- method of solution") != std::string::npos || line.find("--- DDA method") != std::string::npos) res = mapKeys::SOLNMETH;
				else if (line.find(" --- prescription for ") != std::string::npos || line.find("--- CCG method") != std::string::npos) res = mapKeys::POLARIZABILITY;
				else if (line.find("--- shape") != std::string::npos) res = mapKeys::SHAPE;
				else if (line.find("NAT0") != std::string::npos) res = mapKeys::NUMDIPOLES;
				else if (line.find("= d/aeff for this target") != std::string::npos) res = mapKeys::D_AEFF;
				else if (line.find("= d (physical units)") != std::string::npos) res = mapKeys::D;
				else if (line.find("physical extent of target volume") != std::string::npos) res = mapKeys::PHYSEXTENT;
				else if (line.find("effective radius (physical units)") != std::string::npos) res = mapKeys::AEFF;
				else if (line.find("wavelength") != std::string::npos) res = mapKeys::WAVE;
				else if (line.find("K*AEFF") != std::string::npos) res = mapKeys::K_AEFF;
				else if (line.find("nambient") != std::string::npos) res = mapKeys::NAMBIENT;
				else if (line.find("eps.=") != std::string::npos) res = mapKeys::NEPS;
				else if (line.find("tol") != std::string::npos) res = mapKeys::TOL;
				else if (line.find("target axis A1 in") != std::string::npos) res = mapKeys::A1TGT; // target or lab frame
				else if (line.find("target axis A2 in") != std::string::npos) res = mapKeys::A2TGT;
				else if (line.find("navg") != std::string::npos) res = mapKeys::NAVG;
				else if (line.find("k vector (latt. units) in Lab Frame") != std::string::npos) res = mapKeys::KVECLF;
				else if (line.find("k vector (latt. units) in TF") != std::string::npos) res = mapKeys::KVECTF;
				else if (line.find("inc.pol.vec. 1 in LF") != std::string::npos) res = mapKeys::INCPOL1LF;
				else if (line.find("inc.pol.vec. 2 in LF") != std::string::npos) res = mapKeys::INCPOL2LF;
				else if (line.find("inc.pol.vec. 1 in TF") != std::string::npos) res = mapKeys::INCPOL1TF;
				else if (line.find("inc.pol.vec. 2 in TF") != std::string::npos) res = mapKeys::INCPOL2TF; //
				else if (line.find("beta_min") != std::string::npos) res = mapKeys::BETARANGE; // NOTE: else if used because BETA vould conflict with the id
				else if (line.find("theta_min") != std::string::npos) res = mapKeys::THETARANGE;
				else if (line.find("phi_min") != std::string::npos) res = mapKeys::PHIRANGE;
				else if (line.find("etasca") != std::string::npos) res = mapKeys::ETASCA;
				else if (line.find("target orientations") != std::string::npos) res = mapKeys::AVGNUMORI;
				else if (line.find("incident polarizations") != std::string::npos) res = mapKeys::AVGNUMPOL;
				else if (line.find("xmin,xmax") != std::string::npos) res = mapKeys::XTF;
				else if (line.find("ymin,ymax") != std::string::npos) res = mapKeys::YTF;
				else if (line.find("zmin,zmax") != std::string::npos) res = mapKeys::ZTF;
				else if (line.find("beta") != std::string::npos) res = mapKeys::BETA;
				else if (line.find("theta") != std::string::npos) res = mapKeys::THETA;
				else if (line.find("phi") != std::string::npos) res = mapKeys::PHI;
				else if (line.find("target:") != std::string::npos) res = mapKeys::TARGETPERIODICITY;
				else if (line.find("dot") != std::string::npos) res = mapKeys::FMLDOTLINE;
				else if (line.find("perp to scatt") != std::string::npos) res = mapKeys::MDEF;

				return constructObj(res);
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



			ddver::ddver() { std::get<0>(val) = rtmath::ddscat::ddVersions::getDefaultVer(); }

			void ddver::write(std::ostream &out, size_t) const
			{
				out << " DDSCAT --- ";
				out << rtmath::ddscat::ddVersions::getVerAvgHeaderString(std::get<0>(val));
				out << std::endl;
			}

			void ddver::read(std::istream &in)
			{
				std::string lin;
				std::getline(in, lin);
				std::get<0>(val) = rtmath::ddscat::ddVersions::getVerId(lin);
			}

			template<class Archive>
			void ddver::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object
					<rtmath::ddscat::ddOutputSingleKeys::hasValue<size_t> >(*this));
			}


			void ddstring::write(std::ostream &out, size_t) const
			{
				out << std::get<0>(val) << std::endl;
			}
			void ddstring::read(std::istream &in)
			{
				std::getline(in, std::get<0>(val));
			}

			template<class Archive>
			void ddstring::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object
					<rtmath::ddscat::ddOutputSingleKeys::hasValue<std::string> >(*this));
			}

			void ddtarget::write(std::ostream &out, size_t) const
			{
				out << " TARGET --- ";
				out << std::get<0>(val) << std::endl;
			}
			void ddtarget::read(std::istream &in)
			{
				std::string lin;
				std::getline(in, lin);
				size_t p = lin.find("---");
				std::get<0>(val) = lin.substr(p + 3);
				// Remove any leading and lagging spaces
				// Not all Liu avg files are correct in this respect
				boost::algorithm::trim(std::get<0>(val));
			}

			template<class Archive>
			void ddtarget::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object
					<rtmath::ddscat::ddOutputSingleKeys::hasValue<std::string> >(*this));
			}

			ddSval::ddSval() {}
			ddSval::ddSval(const std::string &start, const std::string &end)
			{
				head() = start;
				tail() = end;
			}
			void ddSval::write(std::ostream &out, size_t) const
			{
				out << std::get<0>(val) << "--- " << std::get<1>(val) << std::endl;
			}
			void ddSval::read(std::istream &in)
			{
				std::string lin;
				std::getline(in, lin);
				size_t p = lin.find("--- ");
				std::get<0>(val) = lin.substr(0, p);
				std::get<1>(val) = lin.substr(p + 3);
				boost::algorithm::trim(std::get<1>(val));
			}

			template<class Archive>
			void ddSval::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object
					<rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::string, std::string> >(*this));
			}

			ddNval::dNval(size_t pos, const std::string &head, const std::string &tail)
			{
				value<0>() = pos;
				//value<1>(); --- has type T
				value<2>() = head;
				value<3>() = tail;
			}

			void ddNval::write(std::ostream &out, size_t) const
			{
				out << head() << value<1>() << tail() << std::endl;
			}
			void ddNval::read(std::istream &in)
			{
				std::string lin, junk;
				std::getline(in, lin);
				double pv = 0;
				parse_string_ddNval(lin.begin(), lin.end(), pos, pv);
				Nval() = static_cast<T>(pv);
			}


			ddM::ddM() : w(8) {}

			void ddM::write(std::ostream &out, size_t) const
			{
				// Using formatted io operations
				using std::setw;
				auto &m = std::get<0>(val);
				auto &eps = std::get<1>(val);
				auto &mkd = std::get<2>(val);
				auto &subst = std::get<3>(val);

				out << "n = (" << setw(w) << m.real() << "," << setw(w) << m.imag()
					<< "), eps.= (" << setw(w) << eps.real() << "," << setw(w) << eps.imag()
					<< ")  |m|kd=" << setw(w) << mkd << " for subs. " << subst << std::endl;
			}

			void ddM::read(std::istream &in)
			{
				std::string str;
				auto &m = std::get<0>(val);
				auto &eps = std::get<1>(val);
				auto &mkd = std::get<2>(val);
				auto &subst = std::get<3>(val);

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

				if (snums.size() < 6) throw debug::xBadInput("Cannot parse refractive index in ddOutputSingleKeys::ddM");

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
				
			template<class Archive>
			void ddM::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<
					hasMultipleValues<std::complex<double>,
					std::complex<double>, float, size_t > > (*this));
			}

			ddPolVec::ddPolVec() : w(8), p(5)
			{
				Frame() = frameType::LF;
				VecNum() = 0;
			}

			void ddPolVec::write(std::ostream &out, size_t) const
			{
				// Using formatted io operations
				using std::setw;
				using std::setprecision;
				out << " ( " << setprecision(p) << setw(w) << Pol<0>().real() << ","
					<< setw(w) << Pol<0>().imag() << ")(" << setw(w) << Pol<1>().real()
					<< "," << setw(w) << Pol<1>().imag() << ")(" << setw(w)
					<< Pol<2>().real() << "," << setw(w) << Pol<2>().imag()
					<< ")=inc.pol.vec. " << VecNum() << " in ";
				(Frame() == frameType::LF) ? out << "LF\n" : out << "TF\n";
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

				if (snums.size() < 7) throw debug::xBadInput(
					"Cannot parse inc.pol.vec. numbers in ddOutputSingleKeys::ddPolVec");

				using boost::lexical_cast;
				using boost::algorithm::trim_copy;
				using std::complex;
				Pol<0>() = complex<double>(macros::m_atof(snums[0].c_str()), macros::m_atof(snums[1].c_str()));
				Pol<1>() = complex<double>(macros::m_atof(snums[2].c_str()), macros::m_atof(snums[3].c_str()));
				Pol<2>() = complex<double>(macros::m_atof(snums[4].c_str()), macros::m_atof(snums[5].c_str()));
				VecNum() = lexical_cast<size_t>(snums[6]);
				auto it = str.find_last_of('F');
				if (it == std::string::npos) throw debug::xBadInput(
					"Cannot parse inc.pol.vec. in ddOutputSingleKeys::ddPolVec for frame identifier");
				it--;
				if (str.at(it) == 'L') Frame() = frameType::LF;
				else if (str.at(it) == 'T') Frame() = frameType::TF;
				else throw debug::xBadInput(
					"Cannot parse inc.pol.vec. in ddOutputSingleKeys::ddPolVec for frame identifier (b)");
			}

			template<class Archive>
			void ddPolVec::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<
					hasMultipleValues<std::complex<double>,
					std::complex<double>, std::complex<double>,
					frameType, size_t > >(*this));
			}
		}
	}
}

BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingle);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddOutputSingleObj);


BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::hasValue<size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::hasValue<std::string>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::string, std::string>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<std::complex<double>, std::complex<double>, float, size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<size_t, size_t, std::string, std::string>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::hasMultipleValues<size_t, double, std::string, std::string>);


BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddver);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddstring);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddtarget);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddSval);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddNval<size_t>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddNval<double>);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddM);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddPolVec);
