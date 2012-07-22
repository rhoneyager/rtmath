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
			void read(const std::string &filename = "");
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
			// a1 and a2 are the INITIAL vectors (before rotation!)
			// usually a1 = x_lf, a2 = y_lf
			// choice of a1 and a2 can reorient the shape (useful for KE, PE constraints)
			matrixop _a1, _a2, _a3; // a3 = a1 x a2
			matrixop _d;
			matrixop _x0, _xd;
			
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

		// Lightweight POD class that can be placed in a set
		class shapeFileStatsRotated
		{
		public:
			shapeFileStatsRotated(double beta, double theta, double phi);
			shapeFileStatsRotated();
			~shapeFileStatsRotated();
			double beta;
			double theta;
			double phi;
			// Derived stats quantities
			// PE is a potential energy-like function.
			// PE = sum_i(x_i) (Note: masses assumed to be constant. use scaling factor)
			double PE;
			//void calc();
			bool operator<(const shapeFileStatsRotated &rhs) const;
		private:
			//bool _valid;
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & BOOST_SERIALIZATION_NVP(PE);
					//ar & boost::serialization::make_nvp("Calculated", _valid);
				}
		};

		class shapeFileStatsBase
		{
		public:
			inline size_t N() const {return _N;}
			// Set rotation matrix, with each value in degrees
			//void setRot(double beta, double theta, double phi);
			void calcStatsBase();
			// calcStatsRot calculates the stats RELATIVE to the shapefile default rot.
			void calcStatsRot(double beta, double theta, double phi);

			// rot is the effective rotation designated by the choice of a1 and a2
			matrixop rot, invrot;
			double beta, theta, phi;

			// Before normalization and rotation
			matrixop b_min, b_max, b_mean;
			// After normalization
			matrixop min, max, sum, skewness, kurtosis;
			// Moments
			matrixop mom1, mom2, mominert;

			std::set<shapeFileStatsRotated> rotations;
		protected:
			shapeFileStatsBase();
			virtual ~shapeFileStatsBase();
			
			size_t _N;// Number of dipoles
			matrixop _a1, _a2;
			// Inertia tensor

			// The object
			boost::shared_ptr<const shapefile> _shp;
			bool _valid;
		private:
			friend class boost::serialization::access;
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("shapefile", _shp);
					ar & boost::serialization::make_nvp("N", _N);
					ar & boost::serialization::make_nvp("a1", _a1);
					ar & boost::serialization::make_nvp("a2", _a2);
					ar & BOOST_SERIALIZATION_NVP(beta);
					ar & BOOST_SERIALIZATION_NVP(theta);
					ar & BOOST_SERIALIZATION_NVP(phi);
					ar & boost::serialization::make_nvp("Effective_Rotation", rot);
					ar & boost::serialization::make_nvp("Inverse_Effective_Rotation", invrot);
					ar & BOOST_SERIALIZATION_NVP(b_min);
					ar & BOOST_SERIALIZATION_NVP(b_max);
					ar & BOOST_SERIALIZATION_NVP(b_mean);
					ar & BOOST_SERIALIZATION_NVP(min);
					ar & BOOST_SERIALIZATION_NVP(max);
					ar & BOOST_SERIALIZATION_NVP(sum);
					ar & BOOST_SERIALIZATION_NVP(skewness);
					ar & BOOST_SERIALIZATION_NVP(kurtosis);
					ar & BOOST_SERIALIZATION_NVP(mom1);
					ar & BOOST_SERIALIZATION_NVP(mom2);
					ar & boost::serialization::make_nvp("Moment_Inertia", mominert);
					ar & boost::serialization::make_nvp("Rotation-Dependent", rotations);
				}
		};

		class shapeFileStats : public shapeFileStatsBase
		{
		public:
			shapeFileStats();
			shapeFileStats(const shapefile &shp);
			shapeFileStats(const boost::shared_ptr<const shapefile> &shp);

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
