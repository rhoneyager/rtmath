#include <boost/shared_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <sstream>

#include "relGraph.h"
#include "../../rtmath/rtmath/zeros.h"
#include "../../rtmath/rtmath/error/debug.h"

using namespace rtmath;

namespace shape_hexplate
{
	hexRelns::hexRelns() :
		aeff(0),
		diam(0),
		thick(0),
		ar(0),
		v(0),
		scaleAR(1.0),
		calcAeff(false),
		calcDiam(false),
		calcThick(false),
		calcAR(false),
		calcV(false)
	{
		_constructGraph();
	}

	hexRelns::~hexRelns()
	{
	}

	boost::shared_ptr<rtmath::graphs::vertex> 
		hexRelns::_createVertex(const std::string &name, bool OR)
	{
		using namespace rtmath::graphs;
		boost::shared_ptr<vertex> connector;
		connector = boost::shared_ptr<vertex>(new vertex(OR) );

		return _createVertex(name, connector);
	}

	boost::shared_ptr<rtmath::graphs::vertex> 
		hexRelns::_createVertex(const std::string &name, 
		const std::string &target, const std::string &depends)
	{
		// Tokenize the dependency string, look up vertices, and place in set.
		// Then, create vertex per output of vertex::connect.
		using namespace std;
		using namespace rtmath::graphs;
		typedef boost::tokenizer<boost::char_separator<char> >
			tokenizer;
		boost::char_separator<char> sep(",");
		std::set<boost::shared_ptr<vertex> > setDepends;
		boost::shared_ptr<vertex> res, ptarget;

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

	boost::shared_ptr<rtmath::graphs::vertex> 
		hexRelns::_createVertex(const std::string &name, boost::shared_ptr<rtmath::graphs::vertex> vert)
	{
		using namespace rtmath::graphs;

		if (!_vertices.count(vert))
			_vertices.insert(vert);

		if (!_vertexMap.left.count(name))
			_vertexMap.insert(vertexMap::value_type(name, vert));

		return vert;
	}

	void hexRelns::_constructGraph(bool makegraph)
	{
		// Establish the vertices and construct the graph used in the shape 
		// parameter update process
		using namespace std;
		using namespace rtmath::graphs;
		_vertices.clear();
		_vertexMap.clear();

		// Most basic level of var names. Interdipole spacing is in a derived class, as are others.
		const size_t varnames_size = 7;
		const std::string rawvarnames[varnames_size] = {
			"dspacing",
			"diam",
			"aeff",
			"thick",
			"ar",
			"v",
			"scalear"
		};
		std::set<std::string> varnames( rawvarnames, rawvarnames + varnames_size );

		// Create vertices for end variables
		for (auto it = varnames.begin(); it != varnames.end(); ++it)
		{
			auto vert = _createVertex(*it, true);
			//std::cerr << *it << " - " << vert.get() << "\n";
		}

		// Create vertices representing function relationships
		// This is a table that lists the name of the new node, the variable being calculated, 
		// and the necessary dependencies. This is much cleaner than repeated copying / pasting, 
		// and is much easier to read.
		const size_t varmapnames_size = 30;
		const std::string rawvarmapnames[varmapnames_size] = {
			// name,					target,				dependencies
			"DIAM_THICK__AR",			"ar",				"diam,thick",
			"AR_THICK__DIAM",			"diam",				"ar,thick",
			"DIAM_AR__THICK",			"thick",			"diam,ar",
			"DIAM_THICK__V",			"v",				"diam,thick",
			"V_THICK__DIAM",			"diam",				"v,thick",
			"V_DIAM__THICK",			"thick",			"v,diam",
			"AEFF__V",					"v",				"aeff",
			"V__AEFF",					"aeff",				"v",
			"AR_V__THICK",				"thick",			"ar,v",
			"AEFF_SCALE__AR",			"ar",				"aeff,scalear"
// TODO: fix and add the final two conversions
//			"THICK_SCALE__AR",			"ar",				"thick,scalear",
//			"DIAM_SCALE__AR",			"ar",				"diam,scalear"
		};

		for (size_t i=0; i< varmapnames_size; i = i + 3)
		{
			const string &name = rawvarmapnames[i];
			const string &starget = rawvarmapnames[i+1];
			const string &deps = rawvarmapnames[i+2];

			auto vert = _createVertex(name, starget, deps);
			vert->setVertexRunnableCode(this);
			//std::cerr << name << " - " << vert.get() << "\n";
		}


		// Create the graph from the vertices
		if (makegraph)
			_graph = boost::shared_ptr<graph>(new graph(_vertices));
	}

	void hexRelns::update(size_t level)
	{
		// Create a setWeakVertex based on the known mappings in shapeConstraints
		rtmath::graphs::setWeakVertex known;
		// Basically, if there exists a shapeConstraint name matching a vertex map name, 
		// then it is already known, so it should be added.

		auto setKnown = [&](const std::string &id)
		{
			if (_vertexMap.left.count(id))
			{
				known.insert(_vertexMap.left.at(id));
			}
		};

		setKnown("dspacing");
		if (!calcAeff) setKnown("aeff");
		if (!calcDiam) setKnown("diam");
		if (!calcThick) setKnown("thick");
		if (!calcAR) setKnown("ar");
		if (level) setKnown("scalear");

		update(known);
	}

	void hexRelns::update(const rtmath::graphs::setWeakVertex &fixed)
	{
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
				shrRemaining.insert(it->lock());

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
				bool err = true;
				// Ignore the scalear vertex --- eventually, designate some vertices as unimportant in the depGraph code
				if (intersection.size() == 1)
				{
					if ( _vertexMap.right.at(boost::const_pointer_cast< graphs::vertex >
						( *(intersection.begin()) )) == "scalear") err = false;
				}

				// We have a problem.
				if (err)
				{
					std::ostringstream out;
					out << "Input variable set needs more input variables." << endl;
					out << "Still need: ";
					for (auto it = intersection.begin(); it != intersection.end(); it++)
					{
						boost::shared_ptr<graphs::vertex> UT = boost::const_pointer_cast< graphs::vertex >(*it);
						out << _vertexMap.right.at(UT) << ", ";
					}
					out << std::endl;
					throw rtmath::debug::xBadInput(out.str().c_str());
				}
			}
		}

		{
			using std::cerr;
			using std::endl;
			cerr << "Calculation order:\n";
			for (auto it = order.begin(); it != order.end(); ++it)
			{
				boost::shared_ptr<graphs::vertex> UT = boost::const_pointer_cast< graphs::vertex >(it->lock());
				cerr << _vertexMap.right.at(UT) << ", ";
			}
			cerr << endl;
		}

		// All variables are solved for. Now, go through ordering and perform operations in 
		// specified order to fill in the rest of the variables.
		for (auto it = order.begin(); it != order.end(); ++it)
		{
			boost::shared_ptr<const graphs::vertex> IT = it->lock();
			boost::shared_ptr<graphs::vertex> UT = boost::const_pointer_cast< graphs::vertex >(IT);
			UT->run(_vertexMap.right.at(UT));
		}
		// And we've updated!

	}

	bool hexRelns::mapVertex(const std::string &idstr, boost::shared_ptr<rtmath::graphs::vertex> &vertex)
	{
		if (_vertexMap.left.count(idstr))
		{
			vertex = _vertexMap.left.at(idstr);
			return true;
		} else {
			return false;
		}
	}

	void hexRelns::getVertices(vertexMap &mappings)
	{
		mappings = _vertexMap;
	}

	bool hexRelns::runSupported(const std::string &id)
	{
		if (id == "DIAM_THICK__AR") return true;
		if (id == "AR_THICK__DIAM") return true;
		if (id == "DIAM_AR__THICK") return true;
		if (id == "DIAM_THICK__V") return true;
		if (id == "V_THICK__DIAM") return true;
		if (id == "V_DIAM__THICK") return true;
		if (id == "AEFF__V") return true;
		if (id == "V__AEFF") return true;
		if (id == "AR_V__THICK") return true;
		if (id == "AEFF_SCALE__AR") return true;
		if (id == "THICK_SCALE__AR") return true;
		if (id == "DIAM_SCALE__AR") return true;
		//if (id == "V_SCALE__AUTOTHICK") return true;
		return false;
	}

	void hexRelns::run(const std::string &id)
	{
		using namespace std;
		// This is the trivial case of converter manipulation.
		// The appropriate vertex has its run() method called, and it 
		// eventually makes its way to this bit of code. This is the default
		// catch-all converter, designed to handle several possible basic tasks. 
		// It's all really just in the name of extensibility.
		const double pi = boost::math::constants::pi<double>();

		auto disp = [&]()
		{
			cerr << "Scale-AR:\t" << scaleAR << endl;
			cerr << "Diameter:\t" << diam << endl;
			cerr << "Thickness:\t" << thick << endl;
			cerr << "AR:\t\t" << ar << endl;
			cerr << "Aeff:\t\t" << aeff << endl;
			cerr << "V:\t\t" << v << endl;
		};

		std::cerr << "Running " << id << std::endl;

		if (id == "DIAM_THICK__AR")
		{
			ar = diam / thick;
		} else if (id == "AR_THICK__DIAM")
		{
			diam = ar * thick;
		} else if (id == "DIAM_AR__THICK")
		{
			thick = diam / ar;
		} else if (id == "DIAM_THICK__V")
		{
			v = thick * diam * diam * 3. * sqrt(3.) / 8.;
			//v = 1.5 * sqrt(3.) * thick * pow(diam / 2.,2.);
		} else if (id == "V_THICK__DIAM")
		{
			//diam = 2. * ( sqrt( 2./(3.*sqrt(3.)*thick) ) );
			diam = sqrt( 8.*v / (3.*thick*sqrt(3.)));
		} else if (id == "V_DIAM__THICK")
		{
			//thick = v * 2. / ( 3. * sqrt(3.) * pow(diam/2.,2.0) );
			thick = 8. * v / (3. * sqrt(3.) * diam * diam);
		} else if (id == "AEFF__V")
		{
			v = 4./3. * pi * pow(aeff,3.0);
		} else if (id == "V__AEFF")
		{
			aeff = pow(3.*v/4./pi,1./3.);
		} else if (id == "AR_V__THICK")
		{
			// thick = v * 2. / ( 3. * sqrt(3.) * pow(diam/2.,2.0) );
			// diam = ar * thick
			//
			// Use a solver to solve for thick
			// thick - v * 2. / (3. * sqrt(3.) ( pow(ar*thick/2.,2.0) ) = 0;
			/*
			auto func = [&](double val) -> double
			{
				return val - ( v * 2. / (3. * sqrt(3.) * pow(ar*v/2.,2.0) ) );
			};

			thick = rtmath::zeros::secantMethod(func, 1., 2.);
			*/
			double expr = 8. * v / (ar*ar*3.*sqrt(3.));
			thick = pow(expr,1./3.);
		} else if (id == "AEFF_SCALE__AR")
		{
			// Derived in my notebook
			double expr = 32. * pi / (9.*sqrt(3.)) * pow(2.,2./0.449) * pow(aeff,3.);
			double resa = pow(expr, (-1.0 + (1./0.449)) / (1.0 + (2./0.449)) );
			double resb = resa * scaleAR * pow(2.,-1.0/0.449);
			ar = resb;
		} else if (id == "DIAM_SCALE__AR")
		{
			// Derived in my notebook
			ar = scaleAR / 2.02 * pow(diam,0.451);
		} else if (id == "THICK_SCALE__AR")
		{
			ar = scaleAR * pow(2.02,-1./0.449) * pow(thick,-1.0+1./0.449);
			// Derived in my notebook
			/*
		} else if (id == "V_SCALE__AUTOTHICK")
		{
			// diam = (thick/2.02)^(1/0.449)
			// ar = (thick/2.02)^(1/0.449) / thick
			// ar *= scaleAR, then rederive to get width and thickness, as v should be known

			double expr = v * 8. * pow(2.02,2./0.449) / (scaleAR*scaleAR * 3. * sqrt(3.));
			thick = pow(expr,1./(1.+(2./0.449)));
			double al = pow(thick/2.02,1./0.449);
			cerr << "AL: " << al << endl;
			double aar = al / thick;
			cerr << "AAR: " << aar << endl;
			
			*/
		} else if (id == "")
		{
			throw rtmath::debug::xBadInput("relGraph::run had no id specified");
		} else 
		{
			throw rtmath::debug::xUnimplementedFunction();
		}
	}

}

