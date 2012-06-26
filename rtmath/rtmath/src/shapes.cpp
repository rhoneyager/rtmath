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
#include <boost/serialization/map.hpp>
#include <boost/assign.hpp>
#include <cmath>
#include "../rtmath/refract.h"
#include "../rtmath/units.h"
#include "../rtmath/ddscat/shapes.h"

namespace rtmath {
	namespace ddscat {

		namespace MANIPULATED_QUANTITY
		{

			const char* qnames[] = 
			{
				"NONE",
				"Dipole Spacing",
				"Density",
				"Temperature",
				"Volume",
				"Effective Radius",
				"Mass",
				"Refractive Index (Real)",
				"Refractive Index (Imag)",
				"Frequency",
				"Size Parameter",
				"SHPAR1",
				"SHPAR2",
				"SHPAR3",
				"DENS_T",
				"T_DENS",
				"REFF_V",
				"V_REFF",
				"MASS_V__DENS",
				"MASS_DENS__V",
				"DENS_V__MASS",
				"INVALID"
			};

			void shapeBasicManip::run()
			{
				// This is the trivial case of converter manipulation.
				// The appropriate vertex has its run() method called, and it 
				// eventually makes its way to this bit of code. This is the default
				// catch-all converter, designed to handle several possible basic tasks. 
				// It's all really just in the name of extensibility.
				const double pi = boost::math::constants::pi<double>();

				if (_id == DENS_T)
				{
					// Using http://www.ptb.de/cms/fileadmin/internet/
					// publikationen/buecher/Kohlrausch/Tabellen/Kohlrausch_3_Tabellen_und_Diagramme_Waerme.pdf
					// Gives a few pts for ice I-h, and I'm using a polynomial interp,
					// with R^2 = 1 to 4 digits of precision
					// Formula takes dens in kg/m^3 and returns temp in celsius
					// So, I need to convert density from kg/um^3 to kg/m^3
					double dens = _base->get(DENS);
					dens *= 1.e18;
					double t = -6e-5*pow(dens,6.) + 0.3271*pow(dens,5.)
						- 754.04*pow(dens,4.) + 927139.*pow(dens,3)
						- 6e8*pow(dens,2.) + 2.e11*dens - 4.e13;
					_base->set(TEMP, t + 273.15); // Save temp in Kelvin
				} else if (_id == T_DENS)
				{
					// Using same source as above
					// R^2 = 0.9998, and is near-perfect along the subdomain -50 - 0 C.
					double t = _base->get(TEMP) - 273.15; // need temp in K
					double dens = 1.e-11*pow(t,6.) + 6.e-9*pow(t,5.) + 1.e-6*pow(t,4.)
						+ 0.0001*pow(t,3.) + 0.034*pow(t,2.) -0.1177*t + 916.99;
					// density now in kg/m^3. Want kg/um^3
					dens *= 1.e-18;
					_base->set(DENS, dens);
				} else if (_id == REFF_V)
				{
					_base->set(VOL,4./3. * pi * pow(_base->get(REFF),3.0));
				} else if (_id == V_REFF)
				{
					_base->set(REFF, pow(3.*_base->get(VOL)/4./pi,1./3.));
				} else if (_id == MASS_V__DENS)
				{
					_base->set(DENS, _base->get(MASS) / _base->get(VOL));
				} else if (_id == MASS_DENS__V)
				{
					_base->set(VOL, _base->get(MASS) / _base->get(DENS));
				} else if (_id == DENS_V__MASS)
				{
					_base->set(MASS, _base->get(DENS) * _base->get(VOL));
				} else if (_id == FREQ_TEMP__IREFR_R || _id == FREQ_TEMP__IREFR_I)
				{
					std::complex<double> m;
					refract::mice(_base->get(FREQ), _base->get(TEMP), m);
					_base->set(IREFR_R, m.real());
					_base->set(IREFR_IM, m.imag());
				} else 
				{
					throw rtmath::debug::xUnimplementedFunction();
				}
			}

			void shapeEllipsoidManip::run()
			{
				// This is the trivial case of converter manipulation.
				// The appropriate vertex has its run() method called, and it 
				// eventually makes its way to this bit of code. This is the default
				// catch-all converter, designed to handle several possible basic tasks. 
				// It's all really just in the name of extensibility.
				const double pi = boost::math::constants::pi<double>();
				// Need knowledge of aspect ratios or ellipse components
				if (_id == "GENSHPAR1")
				{
					throw rtmath::debug::xUnimplementedFunction(); // TODO
				} else if (_id == "GENSHPAR2")
				{
					throw rtmath::debug::xUnimplementedFunction(); // TODO
				} else if (_id == "GENSHPAR3")
				{
					throw rtmath::debug::xUnimplementedFunction(); // TODO
				} else
				{
					throw rtmath::debug::xUnimplementedFunction();
				}
			}
		}

		shapeConstraint::shapeConstraint()
		{
		}

		shapeConstraint::~shapeConstraint()
		{
		}

		shapeConstraint::shapeConstraint(
			const std::string &varname,
			const std::string &psetShorthand,
			const std::string &units)
			:
				varname(varname),
				pset(psetShorthand),
				units(units)
		{
		}

		bool shapeConstraint::operator==(const shapeConstraint &rhs) const
		{
			if (varname != rhs.varname) return false;
			if (units != rhs.units) return false;
			if (pset != rhs.pset) return false;
			return true;
		}

		bool shapeConstraint::operator!=(const shapeConstraint &rhs) const
		{
			return !(operator==(rhs));
		}

		bool shapeConstraint::operator<(const shapeConstraint &rhs) const
		{
			int lt;
			if (lt = varname.compare(rhs.varname))
				return (lt>0) ? false : true;

			if (lt = units.compare(rhs.units))
				return (lt>0) ? false : true;

			if (pset != rhs.pset) return (pset < rhs.pset);
			return false;
		}

		typename shapeConstraint::const_iterator shapeConstraint::begin() const
		{
			return pset.begin();
		}

		typename shapeConstraint::const_iterator shapeConstraint::end() const
		{
			return pset.end();
		}

		typename shapeConstraint::const_iterator shapeConstraint::rbegin() const
		{
			return pset.rbegin();
		}

		typename shapeConstraint::const_iterator shapeConstraint::rend() const
		{
			return pset.rend();
		}

		shape::~shape()
		{
		}

		shape::shape()
		{
		}

		double shape::get(const std::string & var) const
		{
			if (_vars.count(var))
				return _vars.at(var);
			return 0;
		}

		void shape::write(const std::string &fname) const
		{
			using namespace ::boost::filesystem;
			using namespace ::boost;
			path p(fname);
			if (exists(p))
			{
				if (is_directory(p))
					throw rtmath::debug::xPathExistsWrongType(fname.c_str());
			}

			// Okay, now to serialize and output...
			std::ofstream out(p.string().c_str());
			//boost::archive::text_oarchive oa(out);
			// oa << *this;
			::boost::archive::xml_oarchive oa(out);
			oa << BOOST_SERIALIZATION_NVP(*this);
		}

		void shapeModifiable::set(const std::string & var, double val)
		{
			_vars[var] = val;
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

			// Create vertices for end variables
			for (size_t i = 1; i < NUM_MANIPULATED_QUANTITY; i++)
			{
				shared_ptr<vertex> w = shared_ptr<vertex>(new vertex(true));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(i, w) ) ; //[i] = w;
				_vertexIdMap.insert( 
					vertexIdMap::value_type(ddscat::MANIPULATED_QUANTITY::qnames[i], i) );
			}

			for (size_t i=1; i<NUM_ALL;i++)
			{
				_vertexIdMap.insert( 
					vertexIdMap::value_type(ddscat::MANIPULATED_QUANTITY::qnames[i], i) );
			}

			// Create vertices representing function relationships
			{
				// I can't make these in batch anywhere, since I need some rather specific relationships.
				shared_ptr<vertex> w;
				std::set<std::shared_ptr<vertex> > ptrs;

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(DENS));
				w = vertex::connect(_vertexMap.left.at(TEMP), ptrs);
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, DENS_T)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(DENS_T, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(TEMP));
				w = vertex::connect(_vertexMap.left.at(DENS), ptrs );
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, T_DENS)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(T_DENS, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(REFF));
				w = vertex::connect(_vertexMap.left.at(VOL), ptrs);
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, REFF_V)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(REFF_V, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(VOL));
				w = vertex::connect(_vertexMap.left.at(REFF), ptrs);
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, V_REFF)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(V_REFF, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(MASS)); ptrs.insert(_vertexMap.left.at(VOL));
				w = vertex::connect(_vertexMap.left.at(DENS), ptrs);
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, MASS_V__DENS)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(MASS_V__DENS, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(MASS)); ptrs.insert(_vertexMap.left.at(DENS));
				w = vertex::connect(_vertexMap.left.at(VOL), ptrs);
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, MASS_DENS__V)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(MASS_DENS__V, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(DENS)); ptrs.insert(_vertexMap.left.at(VOL));
				w = vertex::connect(_vertexMap.left.at(MASS), ptrs);
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, DENS_V__MASS)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(DENS_V__MASS, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(TEMP)); ptrs.insert(_vertexMap.left.at(FREQ));
				w = vertex::connect(_vertexMap.left.at(IREFR_R), ptrs );
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, FREQ_TEMP__IREFR_R)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(FREQ_TEMP__IREFR_R, w));

				ptrs.clear(); ptrs.insert(_vertexMap.left.at(TEMP)); ptrs.insert(_vertexMap.left.at(FREQ));
				w = vertex::connect(_vertexMap.left.at(IREFR_IM), ptrs );
				w->setVertexRunnableCode(shared_ptr<shapeBasicManip>(new shapeBasicManip(this, FREQ_TEMP__IREFR_I)));
				_vertices.insert(w);
				_vertexMap.insert( vertexMap::value_type(FREQ_TEMP__IREFR_I, w));
                                
			}
			

			// Create the graph from the vertices
			_graph = std::shared_ptr<graph>(new graph(_vertices));
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
					shrRemaining.insert(it->lock());

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
				std::shared_ptr<const graphs::vertex> IT = it->lock();
				std::shared_ptr<graphs::vertex> UT = std::const_pointer_cast< graphs::vertex >(IT);
				UT->run();
			}
			// And we've updated!

		}
		
		bool shapeModifiable::mapVertex(const std::string &idstr, size_t &id, std::shared_ptr<rtmath::graphs::vertex> &vertex)
		{
			if (_vertexIdMap.left.count(idstr))
			{
				id = _vertexIdMap.left.at(idstr);
				vertex = _vertexMap.left.at(id);
				return true;
			} else {
				return false;
			}
		}

		void shapeModifiable::getVertices(vertexMap &mappings)
		{
			mappings = _vertexMap;
		}

	}
}


