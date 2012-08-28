#pragma once
#pragma warning(disable:4503) // decorated name length exceeded. with boost bimap mpl

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
#include <boost/bimap.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/weak_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include "../matrixop.h"
#include "../coords.h"
#include "../common_templates.h"
#include "../depGraph.h"
#include "rotations.h"

//double _d; // Interdipole spacing, in um
//double _V; // Volume, in um^3
//double _reff; // reff in um
//double _mass; // mass in kg
//double _density; // kg / um^3
//double _T; // Temperature, K, used in water density calculations

namespace rtmath {
	namespace ddscat {

		class ddPar;
		class ddParGenerator;
		class runScriptIndiv;
		class runScriptGlobal;

		class shapeConstraint;
		typedef boost::shared_ptr<const shapeConstraint> shapeConstraintPtr;

		class shapeConstraint
		{
		public:
			shapeConstraint();
			shapeConstraint(
				const std::string &varname,
				const std::string &psetShorthand,
				const std::string &units = "");
			shapeConstraint(
				const std::string &varname,
				double psetShorthand,
				const std::string &units = "");
			static shapeConstraintPtr create(
				const std::string &varname,
				const std::string &psetShorthand,
				const std::string &units = "");
			static shapeConstraintPtr create(
				const std::string &varname,
				double psetShorthand,
				const std::string &units = "");
			virtual ~shapeConstraint();
			bool operator< (const shapeConstraint &rhs) const;
			bool operator==(const shapeConstraint &rhs) const;
			bool operator!=(const shapeConstraint &rhs) const;
			typedef paramSet<double>::const_iterator const_iterator;
			typedef paramSet<double>::const_reverse_iterator const_reverse_iterator;
			const_iterator begin() const;
			const_iterator end() const;
			const_reverse_iterator rbegin() const;
			const_reverse_iterator rend() const;
			size_t size() const;
			paramSet<double> pset;
			std::string varname;
			std::string units;
		};

		// Using a map because it provides easy access to the constraint var name,
		// really making it an indexed set
		typedef std::multimap< std::string, shapeConstraintPtr > shapeConstraintContainer;

		void addConstraint(shapeConstraintContainer &container, shapeConstraintPtr constraint);

		class constrainable
		{
		protected:
			constrainable(){}
		public:
			virtual ~constrainable() {}
			void addConstraint(shapeConstraintPtr);
			shapeConstraintContainer shapeConstraints;
			friend class boost::serialization::access;
		};

		class shape : public constrainable, public std::enable_shared_from_this<shape>
		{
		public:
			shape();
			virtual ~shape();
			virtual shape* clone() const { shape* ns = new shape(*this); return ns; }
			virtual bool canWrite() const { return false; }
			virtual void write(const std::string &fname, const ddPar &ddbase) const;
			// Nothing done with these until iterator evaluation. They are split, and THEN
			// the mappings and vertexRunnable code is executed
			//shapeConstraintContainer &shapeConstraints;
		protected:
			// Densities of different anisotropic materials. Used in mass and inertia calculations.
			std::map<size_t, double> _densities;
			// Convenient aliases to avoid repeated multimap searching
			bool _get(const std::string &id, double &val, std::string &units) const;
			void _set(const std::string &id, double val, const std::string &units);
		};

		class shapeModifiable : public shape, protected rtmath::graphs::vertexRunnable
		{
		public:
			typedef boost::bimap< std::string, boost::shared_ptr<rtmath::graphs::vertex> > vertexMap;
			typedef rtmath::graphs::setWeakVertex vertexSet;

			shapeModifiable();
			virtual ~shapeModifiable();

			virtual shapeModifiable* clone() const { shapeModifiable *ns = new shapeModifiable(*this); return ns; }
			// create ordering and apply vertex actions to shapeConstraints
			virtual void update(const rtmath::graphs::setWeakVertex &fixed);
			// create ordering and apply vertex actions based on known shapeConstraints
			virtual void update();
			// Return vertex mappings
			void getVertices(vertexMap &mappings);
			// Search for vertex by name
			virtual bool mapVertex(const std::string &idstr, boost::shared_ptr<rtmath::graphs::vertex> &vertex);
			// Set rotation information
			void setRots(boost::shared_ptr<rotations> rots);
			// Prepare an individual run script, given the constraints of the individual shape
			virtual runScriptIndiv prepRunScript(const std::string &name, const ddParGenerator &gen) const;
			// Take shape and cast upward to get the appropriate target object (based on CSHAPE)
			boost::shared_ptr<shapeModifiable> promote() const;
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
			boost::shared_ptr<rotations> _rots;
		protected:
			// The vertexRunnable overrides
			virtual void run(const std::string &id = "");
			virtual bool runSupported(const std::string &id = "");
		};

		namespace shapes
		{
			// _ddscat is a class that provides some of the writing functions for file output.
			class from_ddscat : public shapeModifiable
			{
			protected:
				from_ddscat();
				virtual ~from_ddscat();
			public:
				virtual bool canWrite() const { return true; }
				// This write function is not the same as serialization
				// It will write the necessary ddscat files into the directory.
				// It is implemented to replace ddParIterator's routines, as my code can 
				// also work with tmatrix.
				virtual void write(const std::string &base, const ddPar &ddbase) const;

				virtual runScriptIndiv prepRunScript(const std::string &name, const ddParGenerator &gen) const;

				// The component writing functions
				void exportDiel(const std::string &filename) const;
				//void exportShape(const std::string &filename) const;
				void exportDDPAR(ddPar &out) const;
				void exportDDPAR(const std::string &filename, const ddPar &ddbase) const;
				friend class boost::serialization::access;
			};

			// from_file provides a target for ddscat shape.dat file processing.
			// TODO: allow for stretching/squeezing and aspect ratio manipulation
			//       needs knowledge of shape file statistics (like length, width, depth)
			//       Should this be in another class?
			class from_file : public from_ddscat
			{
			public:
				from_file();
				virtual ~from_file();
				virtual from_file* clone() const { from_file *ns = new from_file(*this); return ns; }
				// Function to write shape.dat
				void exportShape(const std::string &filename) const;
				// General 'write everything' function
				virtual void write(const std::string &base, const ddPar &ddbase) const;

				virtual runScriptIndiv prepRunScript(const std::string &name, const ddParGenerator &gen) const;
			protected:
				virtual void _constructGraph(bool makegraph = true);
				virtual void run(const std::string &id = "");
				virtual bool runSupported(const std::string &id = "");
			};

			
			class ellipsoid : public from_ddscat
			{
			public:
				ellipsoid();
				virtual ~ellipsoid();
				virtual ellipsoid* clone() const { ellipsoid *ns = new ellipsoid(*this); return ns; }
			protected:
				virtual void _constructGraph(bool makegraph = true);
				virtual void run(const std::string &id = "");
				virtual bool runSupported(const std::string &id = "");
			};
			
		}

	}
}
