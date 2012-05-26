#include "../rtmath/Stdafx.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <algorithm>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <complex>
#include <boost/bimap.hpp>
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
			_constructGraph();
		}

		shapeModifiable::~shapeModifiable()
		{
		}

		void shapeModifiable::_constructGraph()
		{
			// Establish the vertices and construct the graph used in the shape 
			// parameter update process
			using namespace std;
			using namespace rtmath::graphs;
			using namespace MANIPULATED_QUANTITY;
			_vertices.clear();
			_vertexMap.clear();
			_graph = nullptr;

			typedef boost::bimap< size_t, std::shared_ptr<rtmath::graphs::vertex> > vidMap;

			// Create vertices for end variables
			for (size_t i = 1; i < NUM_MANIPULATED_QUANTITY; i++)
			{
				shared_ptr<vertex> w = shared_ptr<vertex>(new vertex(true));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(i, w) ) ; //[i] = w;
			}

			// Create vertices representing function relationships
			{
				// I can't make these in batch anywhere, since I need some rather specific relationships.
				shared_ptr<vertex> w;

				w = vertex::connect(_vertexMap.left.at(TEMP), 1, _vertexMap.left.at(DENS));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(DENS_T, w));

				w = vertex::connect(_vertexMap.left.at(DENS), 1, _vertexMap.left.at(TEMP));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(T_DENS, w));

				w = vertex::connect(_vertexMap.left.at(VOL), 1, _vertexMap.left.at(REFF));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(REFF_V, w));

				w = vertex::connect(_vertexMap.left.at(REFF), 1, _vertexMap.left.at(VOL));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(V_REFF, w));

				w = vertex::connect(_vertexMap.left.at(DENS), 
					2, _vertexMap.left.at(MASS), _vertexMap.left.at(VOL));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(MASS_V__DENS, w));

				w = vertex::connect(_vertexMap.left.at(VOL), 
					2, _vertexMap.left.at(MASS), _vertexMap.left.at(DENS));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(MASS_DENS__V, w));

				w = vertex::connect(_vertexMap.left.at(MASS), 
					2, _vertexMap.left.at(DENS), _vertexMap.left.at(VOL));
				_vertices.insert(w);
				_vertexMap.insert( vidMap::value_type(DENS_V__MASS, w));
			}
			

			// Create the graph from the vertices
			_graph = std::shared_ptr<graph>(new graph(_vertices));
		}

		void shapeModifiable::d(double newD)
		{
			_d = newD;
		}

		void shapeModifiable::density(double newDensity)
		{
			_density = newDensity;
		}

		void shapeModifiable::T(double newT)
		{
			_T = newT;
		}

		void shapeModifiable::V(double newV)
		{
			_V = newV;
		}

		void shapeModifiable::reff(double newReff)
		{
			_reff = newReff;
		}

		void shapeModifiable::mass(double newMass)
		{
			_mass = newMass;
		}

		void shapeModifiable::_update(const rtmath::graphs::setWeakVertex &fixed)
		{
			const double pi = boost::math::constants::pi<double>();
			rtmath::graphs::setWeakVertex remaining, ignored;
			rtmath::graphs::listWeakVertex order;
			_graph->generate(fixed, order, remaining, ignored);

			// Now, make sure that all variables are solved for (check that order contains all base vertices)
			{
				using namespace MANIPULATED_QUANTITY;
				using namespace std;
				using namespace rtmath::graphs;
				
				// For ease of intersection calculation, promote weak_ptrs to shared_ptr. It's hell otherwise.

				setShrdVertex baseVertices, shrRemaining;
				for (auto it = remaining.begin(); it != remaining.end(); it++)
					shrRemaining.insert(*it);

				for (size_t i = 1; i < NUM_MANIPULATED_QUANTITY; i++)
					baseVertices.insert(_vertexMap.left.at(i));

				// Use std algorithms to ensure that intersection of remaining and baseVertices is null
				setShrdVertex intersection;

				std::set_intersection(
					baseVertices.begin(), baseVertices.end(),
					shrRemaining.begin(), shrRemaining.end(),
					std::inserter(intersection,intersection.begin())
					);
					
				if (intersection.size())
				{
					// We have a problem.
					throw rtmath::debug::xBadInput("Need to fix more input variables");
				}
			}

			// All variables are solved for. Now, go through ordering and perform operations in 
			// specified order to fill in the rest of the variables.
			for (auto it = order.begin(); it != order.end(); it++)
			{
				// For each step, consult against the reverse map (see bidirectional map)
				// and get the type of operation. Then, based on operation type, execute 
				// the appropriate calculation.
				size_t id = _vertexMap.right.at(*it);

				/* DENS_T
				T_DENS
				REFF_V
				V_REFF
				MASS_V__DENS
				MASS_DENS__V
				DENS_V__MASS
				*/
			}

			/*
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
			*/
		}
		
	}
}


