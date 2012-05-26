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
#include <boost/bimap.hpp>
#include "../matrixop.h"
#include "../coords.h"
#include "../depGraph.h"
#include "../error/error.h"

namespace rtmath {
	namespace ddscat {

		namespace MANIPULATED_QUANTITY
		{
			enum MANIPULATED_QUANTITY
			{
				NONE,
				D,
				DENS,
				TEMP,
				VOL,
				REFF,
				MASS,
				NUM_MANIPULATED_QUANTITY
			};

			enum CONVERTERS
			{
				DENS_T = NUM_MANIPULATED_QUANTITY,
				T_DENS,
				REFF_V,
				V_REFF,
				MASS_V__DENS,
				MASS_DENS__V,
				DENS_V__MASS,
				NUM_ALL
			};
		}

		class shape : public std::enable_shared_from_this<shape>
		{
		public:
			shape() :
			  _d(0), _V(0), _reff(0), _mass(0), _density(0), _T(0), _valid(false) {}
			virtual ~shape();
			virtual double d() const { return _d; }
			virtual double density() const { return _density; }
			virtual double T() const { return _T; }
			virtual double V() const { return _V; }
			virtual double reff() const { return _reff; }
			virtual double mass() const { return _mass; }
		protected:
			double _d; // Interdipole spacing, in um
			double _V; // Volume, in um^3
			double _reff; // reff in um
			double _mass; // mass in kg
			double _density; // kg / um^3
			double _T; // Temperature, K, used in water density calculations
			bool _valid;
			// Densities of different anisotropic materials. Used in mass and inertia calculations.
			std::map<size_t, double> _densities;
		};

		class shapeModifiable : public shape
		{
		public:
			shapeModifiable();
			virtual ~shapeModifiable();
			virtual void d(double newD);
			virtual void density(double newDensity);
			virtual void T(double newT);
			virtual void V(double newV);
			virtual void reff(double newReff);
			virtual void mass(double newMass);
		protected:
			// updater function tracks which value(s) to keep constant 
			// and recalculates the rest
			virtual void _update(
				const rtmath::graphs::setWeakVertex &fixed);
			void _constructGraph();
			std::set< std::shared_ptr<rtmath::graphs::vertex> > _vertices;
			boost::bimap< size_t, 
				std::shared_ptr<rtmath::graphs::vertex> > _vertexMap;
			std::shared_ptr<rtmath::graphs::graph> _graph;
		};

		class shapeSphere : public shapeModifiable
		{
		public:
			shapeSphere();
			virtual ~shapeSphere();
		};


	}
}
