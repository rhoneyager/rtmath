#pragma once
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/access.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/set.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include "../matrixop.h"
#include "../coords.h"
#include "shapes.h"

namespace rtmath {
	namespace ddscat {

		class shapeFileStats;

		class shapefile : public boost::enable_shared_from_this<shapefile>
		{
		public:
			shapefile(const std::string &filename);
			shapefile(std::istream &in);
			~shapefile();
			void print(std::ostream &out) const;
			void read(const std::string &filename);
			void read(std::istream &in);
			void write(const std::string &fname) const;
			void write(std::ostream &out) const;
			boost::shared_ptr<shapefile> getPtr() const;
		private:
			shapefile();
			void _init();
			std::string _filename;
			//std::map<size_t, std::shared_ptr<const matrixop> >
			//	_moments;
			boost::shared_ptr<const matrixop> _lattice;
			std::map<size_t, matrixop > _latticePts;
			std::map<size_t, matrixop > _latticePtsRi;
			std::map<size_t, matrixop > _latticePtsStd;
			size_t _numPoints;
			std::string _desc;
			boost::shared_ptr<matrixop> _a1, _a2, _a3;
			boost::shared_ptr<const matrixop > _d;
			boost::shared_ptr<const matrixop > _x0, _xd;
			//std::shared_ptr<matrixop> _I; // Moments of inertia (not counting mass) in xyz coords
			friend class shapeFileStats;
			friend class boost::serialization::access;
		public:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("filename", _filename);
					//ar & BOOST_SERIALIZATION_NVP(_filename);
				}
		};

		class shapeFileStats
		{
		public:
			shapeFileStats(const shapefile &shp, double beta = 0, double theta = 0, double phi = 0);
			shapeFileStats(const boost::shared_ptr<const shapefile> &shp, double beta = 0, double theta = 0, double phi = 0);
			inline size_t N() const {return _N;}

			// Set rotation matrix, with each value in degrees
			void setRot(double beta, double theta, double phi);
		private:
			size_t _N;// Number of dipoles

			// Center of mass

			// Inertia tensor

			// The object
			boost::shared_ptr<const shapefile> _shp;
			// The rotations
			boost::shared_ptr<matrixop> _rot;
			double _beta, _theta, _phi;
			// Functions
			void _init();
			void _calcOtherStats();
			friend class boost::serialization::access;
		public:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(shapeModifiable);
					ar & boost::serialization::make_nvp("N", _N);
					ar & boost::serialization::make_nvp("beta", _beta);
					ar & boost::serialization::make_nvp("theta", _theta);
					ar & boost::serialization::make_nvp("phi", _phi);
					//ar & BOOST_SERIALIZATION_NVP(_shp);
					// TODO: add rotations (needs matrixop serialization)
				}
		};

	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);

