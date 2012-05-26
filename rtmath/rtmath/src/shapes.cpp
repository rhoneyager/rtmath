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

		namespace MANIPULATED_QUANTITY
		{

			const char* qnames[] = 
			{
				"NONE",
				"DIPOLE SPACING",
				"DENSITY",
				"TEMPERATURE",
				"VOLUME",
				"REFF",
				"MASS",
				"DENS_T",
				"T_DENS",
				"REFF_V",
				"V_REFF",
				"MASS_V__DENS",
				"MASS_DENS__V",
				"DENS_V__MASS",
				"INVALID"
			};

		}

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
					std::ostringstream out;
					out << "Input variable set needs more input variables." << endl;
					out << "Still need: ";
					for (auto it = intersection.begin(); it != intersection.end(); it++)
					{
						std::shared_ptr<graphs::vertex> UT = std::const_pointer_cast< graphs::vertex >(*it);
						out << qnames[_vertexMap.right.at(UT)] << " ";
					}
					out << std::endl;
					throw rtmath::debug::xBadInput(out.str().c_str());
				}
			}

			// All variables are solved for. Now, go through ordering and perform operations in 
			// specified order to fill in the rest of the variables.
			for (auto it = order.begin(); it != order.end(); it++)
			{
				using namespace MANIPULATED_QUANTITY;
				// For each step, consult against the reverse map (see bidirectional map)
				// and get the type of operation. Then, based on operation type, execute 
				// the appropriate calculation.
				std::shared_ptr<const graphs::vertex> IT = it->lock();
				std::shared_ptr<graphs::vertex> UT = std::const_pointer_cast< graphs::vertex >(IT);
				size_t id = _vertexMap.right.at(UT);
				//size_t id = _vertexMap.right.at(it->lock());
				
				if (id == DENS_T)
				{
					throw rtmath::debug::xUnimplementedFunction(); // TODO
				} else if (id == T_DENS)
				{
					throw rtmath::debug::xUnimplementedFunction(); // TODO
				} else if (id == REFF_V)
				{
					_V = 4./3. * pi * _reff * _reff * _reff;
				} else if (id == V_REFF)
				{
					_reff = pow(3.*_V/4./pi,1./3.);
				} else if (id == MASS_V__DENS)
				{
					_density = _mass / _V;
				} else if (id == MASS_DENS__V)
				{
					_V = _mass / _density;
				} else if (id == DENS_V__MASS)
				{
					_mass = _density * _V;
				}
				
			}
			// And we've updated!

		}
		
	}
}


