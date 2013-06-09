#pragma once

#include <boost/bimap.hpp>
#include "../../rtmath/rtmath/depGraph.h"

namespace shape_hexplate
{
	class hexRelns : protected rtmath::graphs::vertexRunnable
	{
	public:
		hexRelns();
		virtual ~hexRelns();

		bool calcAeff, calcDiam, calcThick, calcAR, calcV;
		double aeff, diam, thick, ar, v;

		double scaleAR;

		typedef boost::bimap< std::string, boost::shared_ptr<rtmath::graphs::vertex> > vertexMap;
		typedef rtmath::graphs::setWeakVertex vertexSet;

		// create ordering and apply vertex actions to shapeConstraints
		virtual void update(const rtmath::graphs::setWeakVertex &fixed);
		// create ordering and apply vertex actions based on known shapeConstraints
		virtual void update(size_t level = 0);
		// Return vertex mappings
		void getVertices(vertexMap &mappings);
		// Search for vertex by name
		virtual bool mapVertex(const std::string &idstr, boost::shared_ptr<rtmath::graphs::vertex> &vertex);
	protected:
		// Establish basic vertices and construct graph. Will be overridden by shape specializations,
		// but they will still call this function as a base.
		virtual void _constructGraph(bool makegraph = true);
		// Convenient functions to create a vertex and assign a name in _vertexMap
		virtual boost::shared_ptr<rtmath::graphs::vertex> 
			_createVertex(const std::string &name, bool OR = false);
		// Create vertex like with connect, but by parsing string
		virtual boost::shared_ptr<rtmath::graphs::vertex>
			_createVertex(const std::string &name, 
			const std::string &target, const std::string &depends);
		// - Name and add existing vertex (used with vertex::connect)
		virtual boost::shared_ptr<rtmath::graphs::vertex> 
			_createVertex(const std::string &name, boost::shared_ptr<rtmath::graphs::vertex> vert);

		std::set< boost::shared_ptr<rtmath::graphs::vertex> > _vertices;
		vertexMap _vertexMap;
		boost::shared_ptr<rtmath::graphs::graph> _graph;
	protected:
		// The vertexRunnable overrides
		virtual void run(const std::string &id = "");
		virtual bool runSupported(const std::string &id = "");
	};
}
