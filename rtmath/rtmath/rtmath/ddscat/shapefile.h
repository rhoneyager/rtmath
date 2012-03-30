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
#include "../matrixop.h"
#include "../coords.h"

namespace rtmath {
	namespace ddscat {

		class shapefile
		{
		public:
			shapefile(const std::string &filename);
			shapefile(std::istream &in);
			~shapefile();
			void print(std::ostream &out) const;
			void loadFile(const std::string &filename);
			void loadFile(std::istream &in);
		private:
			shapefile();
			void _init();
			std::string _filename;
			std::map<size_t, std::shared_ptr<const matrixop> >
				_moments;
			std::shared_ptr<const matrixop> _lattice;
			std::map<size_t, coords::cyclic<double> > _latticePts;
			double _mass;
			size_t _numPoints;
			std::string _desc;
			std::shared_ptr<const coords::cyclic<double> > _a1, _a2;
			std::shared_ptr<const coords::cyclic<double> > _d;
			std::shared_ptr<const coords::cyclic<double> > _x0;
		};

	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);

