#pragma once
/* This code enables my ddOutputSingle code to take the incomplete 
 * Mueller relations that are provided by ddscat and fill them 
 * in using symmetry relations. Only 7 of the 16 mueller entries 
 * are independent. These routines will attempt to fill in as 
 * many of these as possible.
 */

#include <boost/bimap.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_io.hpp>
#include <boost/weak_ptr.hpp>
#include <functional>
#include <string>
#include "../depGraph.h"
#include "../matrixop.h"

namespace rtmath
{
	namespace ddscat
	{
		namespace muellerRelations
		{
			class muellerProvider
			{
			public:
				muellerProvider(const std::string &knownIndices = "");

				// Give the pf mask of the known and calculated values
				matrixop fillMask() const;
				// Take a pf and fill in everything possible
				matrixop fill(const matrixop &in) const;

				// Known indices range from 1 to 4, so 11,12,41 are acceptable
				void addKnown(size_t index);
				void addKnown(const std::string &indices);
				
				// Is the index known?
				bool isKnown(const std::string& index) const;
				// Is the vertex completely unknown and unknowable
				bool isUnknown(const std::string& index) const;
				// Can the index be calculated from other known or calculated values?
				bool isCalculable(const std::string& index) const;
			private:
				typedef boost::bimap< std::string, 
					boost::shared_ptr<rtmath::graphs::vertex> > vertexMap;
				typedef rtmath::graphs::setWeakVertex vertexSet;
				typedef std::map<boost::shared_ptr<rtmath::graphs::vertex>, 
					std::function<void(rtmath::matrixop&)> > vertexFuncMap;
				// Flag that indicates if the maps need to be recalculated
				mutable bool recalcmaps;
				// Create ordering and apply vertex actions
				void update();
				// Establish basic vertices and construct the graph
				void constructGraph();
				// Convenience function to create a vertex and attach a function
				// that operates on the matrixop when the node is actively traversed
				void createVertex(const std::string &name, const std::string &target, 
					const std::string &deps, std::function<void(rtmath::matrixop&)> &);

				// These contain the data on the known and unknown sets
				// It does not contain the link vertices
				mutable vertexSet known, unknown, calculable;
				vertexMap vertices;
				//std::set<boost::shared_ptr<rtmath::graphs::vertex> > vertices;
				vertexFuncMap funcMap;
			};
		}
	}
}

