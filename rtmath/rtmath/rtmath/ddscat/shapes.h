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
#include "../error/error.h"

namespace rtmath {
	namespace ddscat {

		namespace MANIPULATED_QUANTITY
		{
			enum MANIPULATED_QUANTITY
			{
				NONE,
				D,
				DENSITY,
				T,
				V,
				REFF,
				MASS,
				NUM_MANIPULATED_QUANTITY
			};
		}

		class shape : public std::enable_shared_from_this<shape>, debug::defective
		{
		public:
			shape();
			virtual ~shape();
			virtual double d() const { return _d; }
			virtual double density() const { return _V; }
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

		class shapeModifiable : public shape, debug::defective
		{
		public:
			shapeModifiable();
			virtual ~shapeModifiable();
			virtual void d(double newD) const = 0;
			virtual void T(double newT) const = 0;
			virtual void V(double newV) const = 0;
			virtual void reff(double newReff) const = 0;
			virtual void mass(double newMass) const = 0;
		protected:
			// TODO: add updater function that tracks which
			// value(s) to keep constant and recalculate the rest
			//virtual void _updater() = 0;
		};

		class shapeSphere : public shapeModifiable, debug::defective
		{
		public:
			shapeSphere();
			virtual ~shapeSphere();
		};


	}
}
