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

		class shapeFileStats;

		class shapefile : public std::enable_shared_from_this<shapefile>
		{
		public:
			shapefile(const std::string &filename);
			shapefile(std::istream &in);
			~shapefile();
			void print(std::ostream &out) const;
			void loadFile(const std::string &filename);
			void loadFile(std::istream &in);
			std::shared_ptr<shapefile> getPtr() const;
		private:
			shapefile();
			void _init();
			std::string _filename;
			std::map<size_t, std::shared_ptr<const matrixop> >
				_moments;
			std::shared_ptr<const matrixop> _lattice;
			std::map<size_t, coords::cyclic<double> > _latticePts;
			std::map<size_t, coords::cyclic<double> > _latticePtsStd;
			double _mass;
			size_t _numPoints;
			std::string _desc;
			std::shared_ptr<const coords::cyclic<double> > _a1, _a2, _a3;
			std::shared_ptr<const coords::cyclic<double> > _d;
			std::shared_ptr<const coords::cyclic<double> > _x0, _xd;
			std::shared_ptr<matrixop> _I; // Moments of inertia (not counting mass) in xyz coords
			friend class shapeFileStats;
		};

		class shapeFileStats
		{
		public:
			shapeFileStats(const shapefile &shp);
			shapeFileStats(const std::shared_ptr<const shapefile> &shp);
			inline double d() const {return _d;}
			inline double density() const {return _density;}
			inline double T() const {return _T;}
			inline double V() const {return _V;}
			inline double reff() const {return _reff;}
			inline size_t N() const {return _N;}
			inline double mass() const {return _mass;}
			inline bool isValid() const { return _valid; }

		private:
			size_t _N;// Number of dipoles
			double _d; // Interdipole spacing, in um
			double _V; // Volume, in um^3
			double _reff; // reff in um
			double _mass; // mass in kg
			double _density; // kg / um^3
			double _T; // Temperature, K, used in water density calculations
			bool _valid;
			// Densities of different anisotropic materials. Used in mass and inertia calculations.
			std::map<size_t, double> _densities;

			// Center of mass

			// Inertia tensor

			// The object
			std::shared_ptr<const shapefile> _shp;
			// Functions
			void _init();
			void _calcFromD();
			void _calcFromV();
			void _calcFromReff();
			void _calcFromMass();
			void _calcDensity();
			void _calcDensities();
			void _calcOtherStats();
		};

	}
}

std::ostream & operator<<(std::ostream &stream, const rtmath::ddscat::shapefile &ob);
std::istream & operator>>(std::istream &stream, rtmath::ddscat::shapefile &ob);

