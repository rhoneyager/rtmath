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
#include <boost/serialization/shared_ptr.hpp>
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
			void read(std::istream &in, size_t length = 0);
			void readString(const std::string &in);
			void write(const std::string &fname) const;
			void write(std::ostream &out) const;
			boost::shared_ptr<shapefile> getPtr() const;
		private:
			shapefile();
			void _init();
			std::string _filename;
			//boost::shared_ptr<const matrixop> _lattice;
			std::vector<matrixop> _latticePts;
			std::vector<matrixop> _latticePtsRi;
			std::vector<matrixop> _latticePtsStd; // Normalized coord transform
			size_t _numPoints;
			std::string _desc;
			// Specified in shape.dat
			boost::shared_ptr<matrixop> _a1, _a2, _a3; // except for a3
			boost::shared_ptr<const matrixop > _d;
			boost::shared_ptr<const matrixop > _x0, _xd;
			
			friend class shapeFileStatsBase;
			friend class shapeFileStats;
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("filename", _filename);
					//ar & BOOST_SERIALIZATION_NVP(_filename);
				}
		};


		class shapeFileStatsBase
		{
		public:
			inline size_t N() const {return _N;}
			// Set rotation matrix, with each value in degrees
			void setRot(double beta, double theta, double phi);
		protected:
			shapeFileStatsBase();
			virtual ~shapeFileStatsBase();
			void _calcStats();

			size_t _N;// Number of dipoles

			// Moments
			matrixop mom1, mom2;
			// Center of mass
			matrixop cm;
			// Inertia tensor

			// The object
			boost::shared_ptr<const shapefile> _shp;
			// The rotations
			matrixop rot;
			double beta, theta, phi;
		private:
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("shapefile", _shp);
					ar & boost::serialization::make_nvp("N", _N);
					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & BOOST_SERIALIZATION_NVP(rot);
					ar & BOOST_SERIALIZATION_NVP(mom1);
					ar & BOOST_SERIALIZATION_NVP(mom2);
					ar & BOOST_SERIALIZATION_NVP(cm);
				}
		};

		class shapeFileStats : public shapeFileStatsBase
		{
		public:
			shapeFileStats();
			shapeFileStats(const shapefile &shp, double beta = 0, double theta = 0, double phi = 0);
			shapeFileStats(const boost::shared_ptr<const shapefile> &shp, double beta = 0, double theta = 0, double phi = 0);

		private:
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(shapeFileStatsBase);
				}
		};

	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);

//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStatsBase)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapeFileStats)
