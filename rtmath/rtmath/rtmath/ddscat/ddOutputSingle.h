#pragma once
#include <string>
#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <complex>
#include <boost/shared_ptr.hpp>
#include "../matrixop.h"
#include "../interpolatable.h"
#include "../phaseFunc.h"
#include "ddScattMatrix.h"
#include "shapefile.h"
#include "../da/daStatic.h"
#include "../coords.h"
#include "../depGraph.h"

namespace rtmath {
	
	namespace ddscat {

		class ddOutputSingleObj;

		class ddOutputSingle
		{
			// Class contains the output of a single ddscat fml / sca or avg file
			// Doesn't quite inherit from daStatic.
			// Note: ensemble providers inherit from this!
		public:
			ddOutputSingle(); //
			virtual ~ddOutputSingle(); //

			// Direct reading and writing of ddscat-formatted files (avg, sca and fml)
			void readFile(const std::string &filename);
			void writeFile(const std::string &filename) const; //

			void writeFML(std::ostream &out) const; //
			void writeSCA(std::ostream &out) const; //
			void writeAVG(std::ostream &out) const; //
			void writeStatTable(std::ostream &out) const; //

			void readFML(std::istream &in);
			void readSCA(std::istream &in);
			void readAVG(std::istream &in);
			void readStatTable(std::istream &in);

			size_t version() const;
			void version(size_t);

			bool operator<(const ddOutputSingle &rhs) const;
		protected:
			size_t _version;
			void _init(); //
			std::map< std::string, boost::shared_ptr<ddOutputSingleObj> >
				_objMap;
			std::vector<double> _statTable;
			std::map< rtmath::coords::cyclic<double> , 
				boost::shared_ptr<const ddscat::ddScattMatrix> >
				_scattMatricesRaw;

			typedef boost::bimap< std::string, 
				boost::shared_ptr<rtmath::graphs::vertex> > vertexMap;
			typedef rtmath::graphs::setWeakVertex vertexSet;
			std::set<boost::shared_ptr<rtmath::graphs::vertex> >
				_depsAVG, _depsFML, _depsSCA, _vertices;
			vertexMap _vertexMap;
			boost::shared_ptr<rtmath::graphs::graph> _graph;
			// Return vertex mappings
			void getVertices(vertexMap &mappings);
			// Search for vertex by name
			bool mapVertex(const std::string &idstr, boost::shared_ptr<rtmath::graphs::vertex> &vertex);
			// create ordering and apply vertex actions to shapeConstraints
			void update(const rtmath::graphs::setWeakVertex &fixed);
			// create ordering and apply vertex actions based on known shapeConstraints
			void update();
			// Establish basic vertices and construct graph.
			void _constructGraph();
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
		};

		class ddOutputSingleObj
		{
		private:
			friend class ddOutputSingle;
			ddOutputSingleObj();
			virtual ~ddOutputSingleObj();
			virtual void write(std::ostream &out) const = 0;
			virtual void read(std::istream &in) = 0;
		};

		enum stat_entries
		{
			QEXT1,QABS1,QSCA1,G11,G21,QBK1,QPHA1,
			QEXT2,QABS2,QSCA2,G12,G22,QBK2,QPHA2,
			QEXTM,QABSM,QSCAM,G1M,G2M,QBKM,QPHAM,
			QPOL,DQPHA,
			QSCAG11,QSCAG21,GSCAG31,ITER1,MXITER1,NSCA1,
			QSCAG12,QSCAG22,GSCAG32,ITER2,MXITER2,NSCA2,
			QSCAG1M,QSCAG2M,QSCAG3M,
			NUM_STAT_ENTRIES
		};
	}

}

