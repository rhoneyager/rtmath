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
#include "../matrixop.h"
//#include "../coords.h"
//#include "shapes.h"

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
			friend class convexHull;
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("filename", _filename);
					ar & boost::serialization::make_nvp("N", _numPoints);
					ar & boost::serialization::make_nvp("a1", _a1);
					ar & boost::serialization::make_nvp("a2", _a2);
					ar & boost::serialization::make_nvp("a3", _a3);
					ar & boost::serialization::make_nvp("d", _d);
					ar & boost::serialization::make_nvp("x0", _x0);
					ar & boost::serialization::make_nvp("xd", _xd);
				}
		};


	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);

//BOOST_CLASS_EXPORT_KEY(rtmath::ddscat::shapefile)
