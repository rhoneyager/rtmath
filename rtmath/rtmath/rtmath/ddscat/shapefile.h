#pragma once
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
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

			std::vector<Eigen::Vector3f> 
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
