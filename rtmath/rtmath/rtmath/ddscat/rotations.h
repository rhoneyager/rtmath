#pragma once

#include <memory>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <set>
#include <vector>
#include <complex>
#include <boost/tokenizer.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp> 
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/shared_ptr.hpp>
#include "ddpar.h"
#include "../common_templates.h"

namespace rtmath {
	namespace ddscat {

		class rotationsBase : public hashable
		{
		public:
			rotationsBase()
				:
			_bMin(0), _bMax(360), _bN(6),
				_tMin(0), _tMax(90), _tN(6),
				_pMin(0), _pMax(180), _pN(6)
			{ }
			virtual ~rotationsBase();
		protected:
			double _bMin, _bMax;
			double _tMin, _tMax;
			double _pMin, _pMax;
			size_t _bN, _tN, _pN;
			friend class boost::serialization::access;
			friend struct std::less<rtmath::ddscat::rotationsBase >;
		public:
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("bMin", _bMin);
				ar & boost::serialization::make_nvp("bMax",_bMax);
				ar & boost::serialization::make_nvp("bN",_bN);
				ar & boost::serialization::make_nvp("tMin", _tMin);
				ar & boost::serialization::make_nvp("tMax", _tMax);
				ar & boost::serialization::make_nvp("tN", _tN);
				ar & boost::serialization::make_nvp("pMin",_pMin);
				ar & boost::serialization::make_nvp("pMax",_pMax);
				ar & boost::serialization::make_nvp("pN", _pN);
			}
		};

		class rotations : public rotationsBase
		{
		public:
			rotations();
			rotations(const ddPar &src);
			rotations(double bMin, double bMax, size_t bN,
				double tMin, double tMax, size_t tN,
				double pMin, double pMax, size_t pN);
			static boost::shared_ptr<rotations> create(
				double bMin, double bMax, size_t bN,
				double tMin, double tMax, size_t tN,
				double pMin, double pMax, size_t pN);
			static boost::shared_ptr<rotations> create();
			static boost::shared_ptr<rotations> create(const ddPar &src);
			virtual ~rotations();
			double bMin() const { return _bMin; }
			double bMax() const { return _bMax; }
			size_t bN() const { return _bN; }
			double tMin() const { return _tMin; }
			double tMax() const { return _tMax; }
			size_t tN() const { return _tN; }
			double pMin() const { return _pMin; }
			double pMax() const { return _pMax; }
			size_t pN() const { return _pN; }
			// ddPar output function
			void out(ddPar &dest) const;
			void betas(std::string &dest) const;
			void thetas(std::string &dest) const;
			void phis(std::string &dest) const;

			bool operator==(const rotations &rhs) const;
			bool operator!=(const rotations &rhs) const;
			bool operator<(const rotations &rhs) const;

			// TODO: add stream-like function alias
			friend struct std::less<rtmath::ddscat::rotations >;
			friend class boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(rotationsBase);
			}
		};

		// Supporting code to allow boost unordered map

		inline std::size_t hash_value(rtmath::ddscat::rotations const& x) { return (size_t) x.hash(); }

	}
}


// Supporting code to allow an unordered map of mapid (for damatrix)
// Using standard namespace for C++11
namespace std {
	template <> struct hash<rtmath::ddscat::rotations>
	{
		size_t operator()(const rtmath::ddscat::rotations & x) const
		{
			// Really need to cast for the unordered map to work
			return (size_t) x.hash();
		}
	};

}; // end namespace std

