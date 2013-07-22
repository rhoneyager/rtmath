#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../Serialization/serialization_macros.h"
#include "../Serialization/eigen_serialization.h"
#include "../hash.h"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

namespace rtmath {
	namespace ddscat {

		class shapefile
		{
		public:
			shapefile(const std::string &filename);
			shapefile(std::istream &in);
			~shapefile();
			void print(std::ostream &out) const;
			/** \brief Read in a shapefile (compression allowed)
			 * 
			 * If a standard (uncompressed) file cannot be found, also search for 
			 * a compressed file.
			**/
			void read(const std::string &filename = "");
			/// Read a shape from an istream (no compression)
			void read(std::istream &in);
			/// Write a shapefile (compression allowed)
			/// \param autoCompress determines whether any output should be 
			/// automatically compressed. Specifying a compressed output filename 
			/// always forces compression.
			void write(const std::string &fname, bool autoCompress = false) const;
			/// Write a shapefile to a stream (no compression)
			void write(std::ostream &out) const;
			/// Export a shapefile to vtk output
			void writeVTK(const std::string &fname) const;
			shapefile();
		private:
			void _init();
			void readHeader(std::istream &in);
			mutable HASH_t _localhash;
			friend class ::boost::serialization::access;
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version);
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
			/// Calculates the hash of the given shapefile. Used as a reference when 
			/// serializing the shape. The hash table allows for smaller stats files.
			HASH_t hash() const;
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
			
		};
	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);


//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile);
BOOST_CLASS_VERSION(rtmath::ddscat::shapefile, 1);


