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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

namespace rtmath {
	class matrixop;

	namespace Garrett {
		class pointContainer;
	}

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
			std::vector<matrixop> _latticePts; // Untransformed points
			std::vector<matrixop> _latticePtsRi; // Dielectric information
			std::vector<matrixop> _latticePtsStd; // Normalized coord transform
			size_t _numPoints;
			std::set<size_t> _Dielectrics;
			std::string _desc;
			// Specified in shape.dat
			// a1 and a2 are the INITIAL vectors (before rotation!)
			// usually a1 = x_lf, a2 = y_lf
			// choice of a1 and a2 can reorient the shape (useful for KE, PE constraints)
			matrixop _a1, _a2, _a3; // a3 = a1 x a2
			matrixop _d;
			matrixop _x0, _xd;

			boost::shared_ptr< rtmath::Garrett::pointContainer > _pclObj;
			
			friend class shapeFileStatsBase;
			friend class shapeFileStats;
			friend class convexHull;
			friend class boost::serialization::access;
		};


	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);
