#pragma once
#include "../defs.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../Serialization/serialization_macros.h"
#include "../Serialization/eigen_serialization.h"
#include "../hash.h"
#include <boost/shared_ptr.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/string.hpp>

namespace rtmath {
	namespace ddscat {

		class DLEXPORT_rtmath_ddscat shapefile
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
			/// Read a shape from a memory buffer
			void readContents(const char *in);
			/// Write a shapefile (compression allowed)
			/// \param autoCompress determines whether any output should be 
			/// automatically compressed. Specifying a compressed output filename 
			/// always forces compression.
			void write(const std::string &fname, bool autoCompress = false,
				const std::string &type = "") const;
			/// Write shape to the hash directory (convenience function)
			void writeToHash() const;
			/// Write a shapefile to a stream (no compression)
			void write(std::ostream &out) const;
			/// Export a shapefile to vtk output
			void writeVTK(const std::string &fname) const;
			/// Export a shapefile to bov output
			void writeBOV(const std::string &prefix) const;

			/** \brief Decimate a shapefile
			* This version of the function examines the number of dipoles in a given degree^3
			* unit cube, and then constructs a smaller shapefile object with the matching parameters.
			*
			* The refractive indices can be externally manipulated to produce threshold values.
			*
			* \note Only works correctly when decimating a shape with one dielectric.
			**/
			boost::shared_ptr<shapefile> decimate(size_t degree = 2) const;

			shapefile();
		private:
			void _init();
			void readHeader(const char *in, size_t &headerEnd);
			/// Resizes arrays to hold the desired number of points
			void resize(size_t num);
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
			/// Force a hash to be recalculated
			HASH_t rehash() const;
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
			
			/// Convenience functions to load shape based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed shape is not found
			static boost::shared_ptr<shapefile> loadHash(
				const HASH_t &hash);
			/// Convenience functions to load shape based on hash
			/// \throws rtmath::debug::xMissingFile if the hashed shape is not found
			static boost::shared_ptr<shapefile> loadHash(
				const std::string &hash);
		};
	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
//std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);


//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile);
BOOST_CLASS_VERSION(rtmath::ddscat::shapefile, 1);


