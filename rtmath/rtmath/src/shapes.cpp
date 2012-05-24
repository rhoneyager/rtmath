#include "../rtmath/Stdafx.h"
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
#include <boost/filesystem.hpp>
#include <boost/tokenizer.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include "../rtmath/ddscat/shapes.h"

namespace rtmath {
	namespace ddscat {

		shape::~shape()
		{
		}


		shapeModifiable::shapeModifiable()
		{
		}

		shapeModifiable::~shapeModifiable()
		{
		}

		void shapeModifiable::d(double newD)
		{
			_d = newD;
			_update(MANIPULATED_QUANTITY::D);
		}

		void shapeModifiable::density(double newDensity)
		{
			_density = newDensity;
			_update(MANIPULATED_QUANTITY::DENSITY);
		}

		void shapeModifiable::T(double newT)
		{
			_T = newT;
			_update(MANIPULATED_QUANTITY::T);
		}

		void shapeModifiable::V(double newV)
		{
			_V = newV;
			_update(MANIPULATED_QUANTITY::V);
		}

		void shapeModifiable::reff(double newReff)
		{
			_reff = newReff;
			_update(MANIPULATED_QUANTITY::REFF);
		}

		void shapeModifiable::mass(double newMass)
		{
			_mass = newMass;
			_update(MANIPULATED_QUANTITY::MASS);
		}

		void shapeModifiable::_update(MANIPULATED_QUANTITY::MANIPULATED_QUANTITY fixed)
		{
			std::set<MANIPULATED_QUANTITY::MANIPULATED_QUANTITY> a;
			a.insert(fixed);
			_update(a);
		}

		void shapeModifiable::_update(const std::set<MANIPULATED_QUANTITY::MANIPULATED_QUANTITY> &fixed)
		{
			// This is ONE UGLY FUNCTION! IN FAR FUTURE, REIMPLEMENT WITH RECURSION!
			// A sphere has just one important parameter: reff
			// So, take the fixed quantity and convert based on the related ones
			
			// T <-> refractive index
			// T <-> density = mass / vol = mass / reff^3
			// V <-> reff
			// d (num dipoles is independent)
			const double pi = boost::math::constants::pi<double>();
			std::set<MANIPULATED_QUANTITY::MANIPULATED_QUANTITY> updated;

			if (fixed.count(MANIPULATED_QUANTITY::V) && fixed.count(MANIPULATED_QUANTITY::REFF))
				throw rtmath::debug::xBadInput("Cannot fix both V and REFF simultaneously and remain consistent");
			if (fixed.count(MANIPULATED_QUANTITY::DENSITY) && fixed.count(MANIPULATED_QUANTITY::T))
				throw rtmath::debug::xBadInput("Cannot fix both Density and Temp simultaneously and remain consistent");

			if (fixed.count(MANIPULATED_QUANTITY::REFF) || fixed.count(MANIPULATED_QUANTITY::V))
			{
				// They're exclusive by this point, so update the other one!
				if (fixed.count(MANIPULATED_QUANTITY::REFF))
				{
					_V = 4./3. * pi * _reff * _reff * _reff;
					updated.insert(MANIPULATED_QUANTITY::V);
				} else {
					_reff = pow(3.*_V/4./pi,1./3.);
					updated.insert(MANIPULATED_QUANTITY::REFF);
				}

				// Try to update density
				if (fixed.count(MANIPULATED_QUANTITY::DENSITY) || fixed.count(MANIPULATED_QUANTITY::T))
				{
					// Density must remain fixed, as must temperature
					// So, see if mass can be changed
					if (fixed.count(MANIPULATED_QUANTITY::MASS))
						throw rtmath::debug::xBadInput("Cannot fix both (V/REFF) and (DENSITY/T) and MASS simultaneously and remain consistent");

					// Change mass
					_mass = _density * _V;
					updated.insert(MANIPULATED_QUANTITY::MASS);
				} else {
					// Density and temperature can change (but prefer not to). Check if mass can change.
					// If mass is fixed, then Dens and T must change
					// If not, prefer change in mass
					if (fixed.count(MANIPULATED_QUANTITY::MASS))
					{
						// Change density and temperature
						_density = _mass / _V;
						_T = _convDT(_density);
						updated.insert(MANIPULATED_QUANTITY::DENSITY);
						updated.insert(MANIPULATED_QUANTITY::T);
					} else {
						// Change mass
						_mass = _density * _V;
						updated.insert(MANIPULATED_QUANTITY::MASS);
					}
				}
				// By this point, something must have changed
			}

			if (fixed.count(MANIPULATED_QUANTITY::MASS))
			{
				// try to change volume / reff
				if (!(fixed.count(MANIPULATED_QUANTITY::V) || fixed.count(MANIPULATED_QUANTITY::REFF))
					&&  !(updated.count(MANIPULATED_QUANTITY::V) || updated.count(MANIPULATED_QUANTITY::REFF)))
				{
					// So, hold density now fixed
					_V = _mass / _density;
					_reff = pow(3.*_V/4./pi,1./3.);
					updated.insert(MANIPULATED_QUANTITY::V);
					updated.insert(MANIPULATED_QUANTITY::REFF);
				} else if (!(fixed.count(MANIPULATED_QUANTITY::DENSITY) || fixed.count(MANIPULATED_QUANTITY::T))
					&& !(updated.count(MANIPULATED_QUANTITY::DENSITY) || updated.count(MANIPULATED_QUANTITY::T)))
				{
					// Hold volume fixed. Update density and T
					// Change density and temperature
					_density = _mass / _V;
					_T = _convDT(_density);
					updated.insert(MANIPULATED_QUANTITY::DENSITY);
					updated.insert(MANIPULATED_QUANTITY::T);
				} else
				{
					throw rtmath::debug::xBadInput("Cannot update from mass based on other constraints or previously updated quantities.");
				}
				// By this point, something must have changed
			}

			// Now try to change things based on density / temp
			if (fixed.count(MANIPULATED_QUANTITY::DENSITY) || fixed.count(MANIPULATED_QUANTITY::T))
			{
				if (fixed.count(MANIPULATED_QUANTITY::DENSITY))
				{
					_T = _convDT(_density);
					updated.insert(MANIPULATED_QUANTITY::T);
				} else {
					_density = _convTD(_T);
					updated.insert(MANIPULATED_QUANTITY::DENSITY);
				}

				// Try changing mass first. Then, change volume. 
				// NOTE: No option to scale mass and volume together. It's not a situation that I care to model.
				if (!(fixed.count(MANIPULATED_QUANTITY::MASS) || updated.count(MANIPULATED_QUANTITY::MASS)))
				{
					// Change mass
					_mass = _density * _V;
					updated.insert(MANIPULATED_QUANTITY::MASS);
				} else if (!(fixed.count(MANIPULATED_QUANTITY::V) || fixed.count(MANIPULATED_QUANTITY::REFF))
					&&  !(updated.count(MANIPULATED_QUANTITY::V) || updated.count(MANIPULATED_QUANTITY::REFF)))
				{
					// Change volume and reff
					_V = _mass / _density;
					_reff = pow(3.*_V/4./pi,1./3.);
					updated.insert(MANIPULATED_QUANTITY::V);
					updated.insert(MANIPULATED_QUANTITY::REFF);
				} else {
					throw rtmath::debug::xBadInput("Cannot update from density with other constraints.");
				}
				// Something must have changed
			}

			// And this overwieldy function is finally done!!!!!
		}

		// Dummy converters for water T and Density
		double shapeModifiable::_convDT(double density)
		{
			throw rtmath::debug::xUnimplementedFunction();
			return 0;
		}

		double shapeModifiable::_convTD(double temp)
		{
			throw rtmath::debug::xUnimplementedFunction();
			return 0;
		}
		
	}
}


