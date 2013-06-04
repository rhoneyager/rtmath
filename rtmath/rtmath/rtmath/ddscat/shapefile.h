#pragma once
//#include "../Serialization/serialization_macros.h"
//#include <boost/serialization/export.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <Eigen/Core>
#include <Eigen/Dense>

// Forward declaration for boost::serialization below
namespace rtmath {
	namespace ddscat {
		class shapefile;
	}
}

// Need these so the template friends can work
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive &, rtmath::ddscat::shapefile &, const unsigned int);
		//EXPORT(serialize, rtmath::ddscat::shapefile);
	}
}

namespace rtmath {
	namespace ddscat {

		class shapefile
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
			shapefile();
		private:
			void _init();
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			std::string filename;
			//std::vector<Eigen::Vector3f> 
			Eigen::Matrix<float, Eigen::Dynamic, 3>
				latticePts, // Untransformed points
				latticePtsStd, // Points with coord translation based on file properties
				latticePtsNorm, // Points with coord transform to mean center of shape
				latticePtsRi; // Dielectric information
			size_t numPoints;
			std::set<size_t> Dielectrics;
			std::string desc;
			// Specified in shape.dat
			// a1 and a2 are the INITIAL vectors (before rotation!)
			// usually a1 = x_lf, a2 = y_lf
			// choice of a1 and a2 can reorient the shape (useful for KE, PE constraints)
			Eigen::Array3f a1, a2, a3, d, x0, xd;

			// These are RAW values (no mean or d scaling)
			Eigen::Array3f mins, maxs, means;

			//boost::shared_ptr< rtmath::Garrett::pointContainer > _pclObj;
			
			friend class shapeFileStatsBase;
			friend class shapeFileStats;
			friend class convexHull;
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, shapefile &, const unsigned int);
		};


	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);


//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
