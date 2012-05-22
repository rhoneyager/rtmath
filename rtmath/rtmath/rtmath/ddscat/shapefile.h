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
#include "../matrixop.h"
#include "../coords.h"
#include "shapes.h"

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
			//std::map<size_t, std::shared_ptr<const matrixop> >
			//	_moments;
			std::shared_ptr<const matrixop> _lattice;
			std::map<size_t, matrixop > _latticePts;
			std::map<size_t, matrixop > _latticePtsRi;
			std::map<size_t, matrixop > _latticePtsStd;
			size_t _numPoints;
			std::string _desc;
			std::shared_ptr<matrixop> _a1, _a2, _a3;
			std::shared_ptr<const matrixop > _d;
			std::shared_ptr<const matrixop > _x0, _xd;
			//std::shared_ptr<matrixop> _I; // Moments of inertia (not counting mass) in xyz coords
			friend class shapeFileStats;
		};



		class shapeFileStats : public shapeModifiable
		{
		public:
			shapeFileStats(const shapefile &shp, double beta = 0, double theta = 0, double phi = 0);
			shapeFileStats(const std::shared_ptr<const shapefile> &shp, double beta = 0, double theta = 0, double phi = 0);
			virtual double d() const {return _d;}
			virtual double density() const {return _density;}
			virtual double T() const {return _T;}
			virtual double V() const {return _V;}
			virtual double reff() const {return _reff;}
			inline size_t N() const {return _N;}
			virtual double mass() const {return _mass;}
			inline bool isValid() const { return _valid; }

			virtual void d(double newD) const;
			virtual void T(double newT) const;
			virtual void V(double newV) const;
			virtual void reff(double newReff) const;
			virtual void mass(double newMass) const;

			// Set rotation matrix, with each value in degrees
			void setRot(double beta, double theta, double phi);
		private:
			size_t _N;// Number of dipoles

			// Center of mass

			// Inertia tensor

			// The object
			std::shared_ptr<const shapefile> _shp;
			// The rotations
			std::shared_ptr<matrixop> _rot;
			double _beta, _theta, _phi;
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

