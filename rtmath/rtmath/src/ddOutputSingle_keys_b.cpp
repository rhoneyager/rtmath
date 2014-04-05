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


			ddver::ddver() { _version = rtmath::ddscat::ddVersions::getDefaultVer(); }
			ddver::~ddver() {}
			void ddver::write(std::ostream &out, size_t) const 
			{
				out << " DDSCAT --- ";
				out << rtmath::ddscat::ddVersions::getVerAvgHeaderString(_version);
				out << std::endl;
			}
			void ddver::read(std::istream &in) 
			{
				std::string lin;
				std::getline(in, lin);
				_version = rtmath::ddscat::ddVersions::getVerId(lin);
			}
			size_t ddver::version() const { return _version; }
			void ddver::version(size_t n) { _version = n; }
			std::string ddver::value() const 
			{
				std::ostringstream out;
				out << _version;
				return out.str();
			}
			template<class Archive>
			void ddver::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("version", _version);
			}

			ddstring::ddstring() {}
			ddstring::~ddstring() {}
			void ddstring::write(std::ostream &out, size_t) const 
			{
				out << s << std::endl;
			}
			void ddstring::read(std::istream &in) 
			{
				std::getline(in, s);
			}
			std::string ddstring::value() const  { return s; }
			template<class Archive>
			void ddstring::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("str", s);
			}
			ddtarget::ddtarget() {}
			ddtarget::~ddtarget() {}
			void ddtarget::write(std::ostream &out, size_t) const 
			{
				out << " TARGET --- ";
				out << s << std::endl;
			}
			void ddtarget::read(std::istream &in) 
			{
				std::string lin;
				std::getline(in, lin);
				size_t p = lin.find("---");
				s = lin.substr(p + 3);
				// Remove any leading and lagging spaces
				// Not all Liu avg files are correct in this respect
				boost::algorithm::trim(s);
			}
			void ddtarget::setTarget(const std::string &n) { s = n; }
			std::string ddtarget::value() const  { return s; }
			template<class Archive>
			void ddtarget::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("target", s);
			}
			ddSval::ddSval(const std::string &tail) { this->tail = tail; }
			ddSval::~ddSval() {}
			void ddSval::write(std::ostream &out, size_t) const 
			{
				out << s << "--- " << tail << std::endl;
			}
			void ddSval::read(std::istream &in) 
			{
				std::string lin;
				std::getline(in, lin);
				size_t p = lin.find("--- ");
				s = lin.substr(0, p);
			}
			std::string ddSval::value() const  { return s; }
			template<class Archive>
			void ddSval::serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp(
					"base",
					boost::serialization::base_object<rtmath::ddscat::ddOutputSingleObj>(*this));
				ar & boost::serialization::make_nvp("s", s);
				ar & boost::serialization::make_nvp("tail", tail);
			}

		}

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

	}
}


BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddver);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddstring);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddtarget);
BOOST_CLASS_EXPORT_IMPLEMENT(rtmath::ddscat::ddOutputSingleKeys::ddSval);
