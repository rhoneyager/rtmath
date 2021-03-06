#include "Stdafx-ddscat.h"
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
#include <boost/assign.hpp>
#include <cmath>
#include "../rtmath/refract.h"
#include "../rtmath/units.h"
#include "../rtmath/ddscat/shapes.h"
#include "../rtmath/ddscat/ddpar.h"
#include "../rtmath/ddscat/ddparGenerator.h"
#include "../rtmath/ddscat/runScripts.h"
#include "../rtmath/error/debug.h"
#include "../rtmath/error/error.h"

namespace rtmath {
	namespace ddscat {

		void addConstraint(shapeConstraintContainer &container, shapeConstraintPtr constraint)
		{
			//constraintsVaried.insert(pair<const string,units::hasUnits>(name, units::hasUnits(val,units)));
			//container.
			container.insert(std::pair<const std::string, shapeConstraintPtr >
				(constraint->varname, constraint));
		}

		bool shapeModifiable::runSupported(const std::string &id)
		{
			if (id == "DENS_T") return true;
			if (id == "T_DENS") return true;
			if (id == "AEFF_V") return true;
			if (id == "V_AEFF") return true;
			if (id == "MASS_V__DENS") return true;
			if (id == "MASS_DENS__V") return true;
			if (id == "DENS_V__MASS") return true;
			if (id == "FREQ_TEMP__IREFR_R") return true;
			if (id == "FREQ_TEMP__IREFR_I") return true;
			/*
				"AEFF_ASxy__Az",			"az",				"aeff,aspect_xy",
				"AEFF_ASxz__Ay",			"ay",				"aeff,aspect_xz",
				"AEFF_ASyz__Ax",			"ax",				"aeff,aspect_yz"
				*/
			if (id == "ASxz_ASxy__ASyz") return true;
			if (id == "ASxz_ASyz__ASxy") return true;
			if (id == "ASxy_ASyz__ASxz") return true;
			if (id == "Ax_Ay__ASxy") return true;
			if (id == "Ax_Az__ASxz") return true;
			if (id == "Ay_Az__ASyz") return true;
			if (id == "AEFF_ASxy__Az") return true;//
			if (id == "AEFF_ASxz__Ay") return true;
			if (id == "AEFF_ASyz__Ax") return true;
			return false;
		}

		void shapeModifiable::run(const std::string &id)
		{
			using namespace std;
			using namespace rtmath::units;
			// This is the trivial case of converter manipulation.
			// The appropriate vertex has its run() method called, and it 
			// eventually makes its way to this bit of code. This is the default
			// catch-all converter, designed to handle several possible basic tasks. 
			// It's all really just in the name of extensibility.
			const double pi = boost::math::constants::pi<double>();
			double raw;
			string units;
			/*
			{
				set<boost::tuple<string,string> > permutations;
				permutations.insert(boost::make_tuple<string,string>("xy","z"));
				permutations.insert(boost::make_tuple<string,string>("xz","y"));
				permutations.insert(boost::make_tuple<string,string>("yz","x"));
				for (auto it = permutations.begin(); it != permutations.end(); it++)
				{
					ostringstream sid;
					sid << "A" << it->get<0>() << "_A" << it->get<1>() << "__AS" << it->get<2>();
					ostringstream sa1, sa2, sa3;
					sa1 << "a" << it->get<0>();
					sa2 << "a" << it->get<1>();
					sa3 << "aspect_" << it->get<2>();

					if (id == sid.str())
					{
						double as_a, as_b, as_c;
						_get(sa1.str(), as_a, units);
						_get(sa2.str(), as_b, units);
						as_c = as_a / as_b;

						_set(sa3.str(), as_c , "");
						return;
					}
				}
			}

			{
				set<boost::tuple<string,string,string> > permutations;
				permutations.insert(boost::make_tuple<string,string,string>("x","y","xy"));
				permutations.insert(boost::make_tuple<string,string,string>("x","z","xz"));
				permutations.insert(boost::make_tuple<string,string,string>("y","z","xy"));
				for (auto it = permutations.begin(); it != permutations.end(); it++)
				{
					ostringstream sid;
					sid << "A" << it->get<0>() << "_A" << it->get<1>() << "__AS" << it->get<2>();
					ostringstream sa1, sa2, sa3;
					sa1 << "a" << it->get<0>();
					sa2 << "a" << it->get<1>();
					sa3 << "aspect_" << it->get<2>();

					if (id == sid.str())
					{
						double as_a, as_b, as_c;
						_get(sa1.str(), as_a, units);
						_get(sa2.str(), as_b, units);
						as_c = as_a / as_b;

						_set(sa3.str(), as_c , "");
						return;
					}
				}
			}

			{
				// Carefully constructed permutations, such that
				// AS1 / AS2 = AS3
				set<boost::tuple<string,string,string> > permutations;
				permutations.insert(boost::make_tuple<string,string,string>("xz","xy","yz"));
				permutations.insert(boost::make_tuple<string,string,string>("xz","yz","xy"));
				permutations.insert(boost::make_tuple<string,string,string>("xy","yz","xz"));
				for (auto it = permutations.begin(); it != permutations.end(); it++)
				{
					ostringstream sid;
					sid << "AS" << it->get<0>() << "_AS" << it->get<1>() << "__AS" << it->get<2>();
					ostringstream sa1, sa2, sa3;
					sa1 << "aspect_" << it->get<0>();
					sa2 << "aspect_" << it->get<1>();
					sa3 << "aspect_" << it->get<2>();

					if (id == sid.str())
					{
						double as_a, as_b, as_c;
						_get(sa1.str(), as_a, units);
						_get(sa2.str(), as_b, units);
						as_c = as_a / as_b;

						_set(sa3.str(), as_c , "");
						return;
					}
				}
			}
			*/
			if (id == "DENS_T")
			{
				// Using http://www.ptb.de/cms/fileadmin/internet/
				// publikationen/buecher/Kohlrausch/Tabellen/Kohlrausch_3_Tabellen_und_Diagramme_Waerme.pdf
				// Gives a few pts for ice I-h, and I'm using a polynomial interp,
				// with R^2 = 1 to 4 digits of precision
				// Formula takes dens in kg/m^3 and returns temp in celsius
				// So, I need to convert density from kg/um^3 to kg/m^3
				
				// TODO: implement this converter
				//GETOBJKEY();

				_get("density",raw,units);
				double dens = raw;
				dens *= 1.e18;
				double t = -6e-5*pow(dens,6.) + 0.3271*pow(dens,5.)
					- 754.04*pow(dens,4.) + 927139.*pow(dens,3)
					- 6e8*pow(dens,2.) + 2.e11*dens - 4.e13;
				_set("temp", t + 273.15, "K"); // Save temp in Kelvin
			} else if (id == "T_DENS")
			{
				// Using same source as above
				// R^2 = 0.9998, and is near-perfect along the subdomain -50 - 0 C.
				_get("temp", raw, units);
				double t = conv_temp(units,"K").convert(raw);
				double dens = 1.e-11*pow(t,6.) + 6.e-9*pow(t,5.) + 1.e-6*pow(t,4.)
					+ 0.0001*pow(t,3.) + 0.034*pow(t,2.) -0.1177*t + 916.99;
				// density now in kg/m^3. Want kg/um^3
				dens *= 1.e-18;
				_set("density", dens, "kg/um^3");
			} else if (id == "AEFF_V")
			{
				_get("aeff",raw,units);
				double aeff_um = conv_alt(units,"um").convert(raw);
				_set("volume", 4./3. * pi * pow(aeff_um,3.0), "um^3");
			} else if (id == "V_AEFF")
			{
				_get("volume",raw,units);
				double v_umt = conv_vol(units,"um^3").convert(raw);
				_set("aeff", pow(3.*v_umt/4./pi,1./3.), "um");
			} else if (id == "MASS_V__DENS")
			{
				double mass;
				_get("mass",raw,units);
				mass = conv_mass(units,"kg").convert(raw);
				_get("volume",raw,units);
				double v_umt = conv_vol(units,"um^3").convert(raw);
				_set("density", mass / v_umt, "kg/um^3");
			} else if (id == "MASS_DENS__V")
			{
				double mass, density;
				_get("mass",raw,units);
				mass = conv_mass(units,"kg").convert(raw);
				_get("density",raw,units);
				density = raw; //conv_vol(units,"um^3").convert(raw);
				// TODO: implement this converter
				//GETOBJKEY();

				_set("volume", mass / density, "um^3");
			} else if (id == "DENS_V__MASS")
			{
				double density, volume;
				// TODO: implement density converter
				//GETOBJKEY();

				_get("density", density, units);

				_get("volume", raw, units);
				volume = conv_vol(units,"um^3").convert(raw);

				_set("mass", density * volume, "kg");
			} else if (id == "FREQ_TEMP__IREFR_R" || id == "FREQ_TEMP__IREFR_I")
			{
				std::complex<double> m;
				double freq, temp;

				_get("freq", raw, units); freq = conv_spec(units,"GHz").convert(raw);
				_get("temp", raw, units); temp = conv_temp(units, "K").convert(raw);

				refract::mIce(freq, temp, m);
				_set("IREFR_R", m.real(), "");
				_set("IREFR_IM", m.imag(), "");
			} else 
			{
				throw rtmath::debug::xUnimplementedFunction();
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

		shapeConstraint::shapeConstraint(
			const std::string &varname,
			double psetShorthand,
			const std::string &units)
			:
				varname(varname),
				pset(psetShorthand),
				units(units)
		{
		}

		shapeConstraintPtr shapeConstraint::create(
				const std::string &varname,
				const std::string &psetShorthand,
				const std::string &units)
		{
			shapeConstraintPtr res(
				new shapeConstraint(varname, psetShorthand, units));
			return res;
		}

		shapeConstraintPtr shapeConstraint::create(
				const std::string &varname,
				double psetShorthand,
				const std::string &units)
		{
			shapeConstraintPtr res(
				new shapeConstraint(varname, psetShorthand, units));
			return res;
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
			// Yes, I know what I'm doing with setting the variable!
			if (lt = varname.compare(rhs.varname))
				return (lt>0) ? false : true;

			if (lt = units.compare(rhs.units))
				return (lt>0) ? false : true;

			if (pset != rhs.pset) return (pset < rhs.pset);
			return false;
		}

		shapeConstraint::const_iterator shapeConstraint::begin() const
		{
			return pset.begin();
		}

		shapeConstraint::const_iterator shapeConstraint::end() const
		{
			return pset.end();
		}

		shapeConstraint::const_reverse_iterator shapeConstraint::rbegin() const
		{
			return pset.rbegin();
		}

		shapeConstraint::const_reverse_iterator shapeConstraint::rend() const
		{
			return pset.rend();
		}

		size_t shapeConstraint::size() const
		{
			return pset.size();
		}

		shape::~shape()
		{
		}

		shape::shape()
		{
		}

		void constrainable::addConstraint(shapeConstraintPtr rhs)
		{
			rtmath::ddscat::addConstraint(shapeConstraints, rhs);
		}

		bool shape::_get(const std::string &id, double &val, std::string &units) const
		{
			auto it = shapeConstraints.find(id);
			if (it == shapeConstraints.end()) return false;
			val = *(it->second->begin());
			units = it->second->units;
			return true;
		}

		void shape::_set(const std::string &id, double val, const std::string &units)
		{
			// Erase existing ranges
			auto it = shapeConstraints.equal_range(id);
			shapeConstraints.erase(it.first, it.second);

			boost::shared_ptr<shapeConstraint> sc
				(new shapeConstraint(id, boost::lexical_cast<std::string>(val), units));

			// Insert value
			shapeConstraints.insert(std::pair<std::string, boost::shared_ptr<shapeConstraint> >
				(id,sc));

		}

		void shape::write(const std::string &fname, const ddPar &ddbase) const
		{
			// This member just prevents a pure virtual shape. It does nothing, as at this level 
			// there is no knowledge of how to write the shape. Execution should never reach here in a 
			// proper implementation. canWrite prevents most cases, and if the derived class is writable,
			// it should override this.
			throw rtmath::debug::xUnimplementedFunction();
		}

		shapeModifiable::shapeModifiable()
		{
			_constructGraph();
		}

		shapeModifiable::~shapeModifiable()
		{
		}

		runScriptIndiv shapeModifiable::prepRunScript(const std::string &name, const ddParGenerator &gen) const
		{
			// Should be overridden, but I don't want a pure virtual class
			throw rtmath::debug::xUnimplementedFunction();
			return runScriptIndiv("", ddParGenerator()); // To prevent errors
		}

		rtmath::graphs::vertex*
			shapeModifiable::_createVertex(const std::string &name, bool OR)
		{
			using namespace rtmath::graphs;
			vertex* connector = new vertex(OR);

			return _createVertex(name, connector);
		}

		rtmath::graphs::vertex*
			shapeModifiable::_createVertex(const std::string &name, 
			const std::string &target, const std::string &depends)
		{
			// Tokenize the dependency string, look up vertices, and place in set.
			// Then, create vertex per output of vertex::connect.
			using namespace std;
			using namespace rtmath::graphs;
			typedef boost::tokenizer<boost::char_separator<char> >
				tokenizer;
			boost::char_separator<char> sep(",");
			std::set<vertex* > setDepends;
			vertex* res = nullptr, *ptarget = nullptr;

			if (!(_vertexMap.left.count(target)))
				throw rtmath::debug::xBadInput(target.c_str());
			ptarget = _vertexMap.left.at(target);

			tokenizer tcom(depends,sep);
			for (auto it = tcom.begin(); it != tcom.end(); ++it)
			{
				if (!(_vertexMap.left.count(*it))) 
					throw rtmath::debug::xBadInput(it->c_str());

				setDepends.insert(_vertexMap.left.at(*it));
			}

			res = vertex::connect(ptarget, setDepends );
			return _createVertex(name, res);
		}

		rtmath::graphs::vertex*
			shapeModifiable::_createVertex(const std::string &name, rtmath::graphs::vertex* vert)
		{
			using namespace rtmath::graphs;

			if (!_vertices.count(vert))
				_vertices.insert(vert);

			if (!_vertexMap.left.count(name))
				_vertexMap.insert(vertexMap::value_type(name, vert));

			return vert;
		}

		void shapeModifiable::_constructGraph(bool makegraph)
		{
			// Establish the vertices and construct the graph used in the shape 
			// parameter update process
			using namespace std;
			using namespace rtmath::graphs;
			_vertices.clear();
			_vertexMap.clear();

			// Most basic level of var names. Interdipole spacing is in a derived class, as are others.
			const size_t varnames_size = 14;
			const std::string rawvarnames[varnames_size] = {
				"density",
				"temp",
				"aeff",
				"volume",
				"mass",
				"freq",
				"irefr_r",
				"irefr_i",
				"aspect_xy",
				"aspect_yz",
				"aspect_xz",
				"ax",
				"ay",
				"az"
			};
			std::set<std::string> varnames( rawvarnames, rawvarnames + varnames_size );

			// Create vertices for end variables
			for (auto it = varnames.begin(); it != varnames.end(); ++it)
				_createVertex(*it, true);

			// Create vertices representing function relationships
			// This is a table that lists the name of the new node, the variable being calculated, 
			// and the necessary dependencies. This is much cleaner than repeated copying / pasting, 
			// and is much easier to read.
			const size_t varmapnames_size = 54;
			const std::string rawvarmapnames[varmapnames_size] = {
				// name,					target,				dependencies
				"DENS_T",					"temp",				"density",
				"T_DENS",					"density",			"temp",
				"AEFF_V",					"volume",			"aeff",
				"V_AEFF",					"aeff",				"volume",
				"MASS_V__DENS",				"density",			"mass,volume",
				"MASS_DENS__V",				"volume",			"mass,density",
				"DENS_V__MASS",				"mass",				"density,volume",
				"FREQ_TEMP__IREFR_R",		"irefr_r",			"freq,temp",
				"FREQ_TEMP__IREFR_I",		"irefr_i",			"freq,temp",
				// Start from here - TODO: check dep correctness
				"ASxz_ASxy__ASyz",			"aspect_yz",		"aspect_xz,aspect_xy",
				"ASxz_ASyz__ASxy",			"aspect_xy",		"aspect_xz,aspect_yz",
				"ASxy_ASyz__ASxz",			"aspect_xz",		"aspect_xy,aspect_yz",
				"Ax_Ay__ASxy",				"aspect_xy",		"ax,ay",
				"Ax_Az__ASxz",				"aspect_xz",		"ax,az",
				"Ay_Az__ASyz",				"aspect_yz",		"ay,az",
				"AEFF_ASxy__Az",			"az",				"aeff,aspect_xy",
				"AEFF_ASxz__Ay",			"ay",				"aeff,aspect_xz",
				"AEFF_ASyz__Ax",			"ax",				"aeff,aspect_yz"
			};

			for (size_t i=0; i< varmapnames_size; i = i + 3)
			{
				const string &name = rawvarmapnames[i];
				const string &starget = rawvarmapnames[i+1];
				const string &deps = rawvarmapnames[i+2];

				_createVertex(name, starget, deps);
			}
			

			// Create the graph from the vertices
			if (makegraph)
				_graph = boost::shared_ptr<graph>(new graph(_vertices));
		}

		void shapeModifiable::update()
		{
			// Create a setWeakVertex based on the known mappings in shapeConstraints
			rtmath::graphs::setWeakVertex known;
			// Basically, if there exists a shapeConstraint name matching a vertex map name, 
			// then it is already known, so it should be added.

			for (auto it = shapeConstraints.begin(); it != shapeConstraints.end(); ++it)
			{
				if (_vertexMap.left.count(it->first))
				{
					known.insert(_vertexMap.left.at(it->first));
				}
			}

			update(known);
		}

		void shapeModifiable::update(const rtmath::graphs::setWeakVertex &fixed)
		{
			const double pi = boost::math::constants::pi<double>();
			rtmath::graphs::setWeakVertex remaining, ignored;
			rtmath::graphs::listWeakVertex order;

			_graph->generate(fixed, order, remaining, ignored);

			// Now, make sure that all variables are solved for (check that order contains all base vertices)
			{
				using namespace std;
				using namespace rtmath::graphs;
				
				// For ease of intersection calculation, promote weak_ptrs to shared_ptr. It's hell otherwise.

				setShrdVertex baseVertices, shrRemaining;
				for (auto it = remaining.begin(); it != remaining.end(); it++)
					shrRemaining.insert(*it);

				// baseVertices are those which are not connectors and have no associated run code
				// use bool vertex::isOR() to determine this
				for (auto it = _vertices.begin(); it != _vertices.end(); ++it)
				{
					if ((*it)->isOR() == true)
						baseVertices.insert(*it);
				}

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
						graphs::vertex* UT = (*it);
						out << _vertexMap.right.at(UT) << ", ";
					}
					out << std::endl;
					throw rtmath::debug::xBadInput(out.str().c_str());
				}
			}

			// All variables are solved for. Now, go through ordering and perform operations in 
			// specified order to fill in the rest of the variables.
			for (auto it = order.begin(); it != order.end(); it++)
			{
				graphs::vertex* IT = it->first; //.lock();
				//boost::shared_ptr<graphs::vertex> UT = boost::const_pointer_cast< graphs::vertex >(IT);
				IT->run();
			}
			// And we've updated!

		}
		
		bool shapeModifiable::mapVertex(const std::string &idstr, rtmath::graphs::vertex* vertex)
		{
			if (_vertexMap.left.count(idstr))
			{
				vertex = _vertexMap.left.at(idstr);
				return true;
			} else {
				return false;
			}
		}

		void shapeModifiable::getVertices(vertexMap &mappings)
		{
			mappings = _vertexMap;
		}

		void shapeModifiable::setRots(boost::shared_ptr<rotations> rots)
		{
			_rots = rots;
		}
	}
}
