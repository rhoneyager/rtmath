#pragma once
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
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/tuple/tuple_comparison.hpp>
#include "../matrixop.h"
#include "../coords.h"
#include "../common_templates.h"
#include "../depGraph.h"
#include "../error/error.h"

//double _d; // Interdipole spacing, in um
//double _V; // Volume, in um^3
//double _reff; // reff in um
//double _mass; // mass in kg
//double _density; // kg / um^3
//double _T; // Temperature, K, used in water density calculations

namespace rtmath {
	namespace ddscat {

		class ddPar;

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
			static boost::shared_ptr<shapeConstraint> create(
				const std::string &varname,
				const std::string &psetShorthand,
				const std::string &units = "");
			static boost::shared_ptr<shapeConstraint> create(
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
			friend class boost::serialization::access;
		private:
			template<class Archive>
			void serialize(Archive & ar, const unsigned int version)
			{
				ar & boost::serialization::make_nvp("variable", varname);
				ar & boost::serialization::make_nvp("paramSet",pset);
				ar & boost::serialization::make_nvp("units", units);
			}
		};

		// Using a map because it provides easy access to the constraint var name,
		// really making it an indexed set
		typedef std::multimap< std::string, boost::shared_ptr<shapeConstraint> > shapeConstraintContainer;

		void addConstraint(shapeConstraintContainer &container, boost::shared_ptr<shapeConstraint> constraint);

		class constrainable
		{
		protected:
			constrainable(){}
			shapeConstraintContainer ___sbase;
		public:
			virtual ~constrainable() {}
			void addConstraint(boost::shared_ptr<shapeConstraint>);
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & boost::serialization::make_nvp("shapeConstraints", ___sbase);
				}
		};

		class shape : public constrainable, public std::enable_shared_from_this<shape>
		{
		public:
			shape();
			virtual ~shape();
			virtual shape* clone() const { shape* ns = new shape(*this); return ns; }
			virtual bool canWrite() const { return false; }
			virtual void write(const std::string &fname) const;
			virtual bool useDDPAR() const;
			virtual void setDDPAR(ddPar &out) const;
			// Nothing done with these until iterator evaluation. They are split, and THEN
			// the mappings and vertexRunnable code is executed
			shapeConstraintContainer &shapeConstraints;
		protected:
			// Densities of different anisotropic materials. Used in mass and inertia calculations.
			std::map<size_t, double> _densities;
			// Convenient aliases to avoid repeated multimap searching
			bool _get(const std::string &id, double &val, std::string &units) const;
			void _set(const std::string &id, double val, const std::string &units);
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(constrainable);
					ar & BOOST_SERIALIZATION_NVP(shapeConstraints);
					ar & boost::serialization::make_nvp("densities", _densities);
				}
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
		protected:
			// Establish basic vertices and construct graph. Will be overridden by shape specializations,
			// but they will still call this function as a base.
			virtual void _constructGraph();
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
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					// TODO: save the vertex maps!!!
					// saving these implies need to save lambda function linkage.....?
					GETOBJKEY();
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(shape);
				}
		protected:
			// The vertexRunnable overrides
			virtual void run(const std::string &id = "");
			virtual bool runSupported(const std::string &id = "");
		};
		
		// In ellipsoid, 1/4 = (x/d*shp1)^2 + (y/d*shp2)^2 + (z/d*shp3)^2
		// In cylinder, shp1 = length/d, shp2 = diam/d,
		//		shp3 = 1 for a1 || x, 2 for a1|| y, 3 for a1 || z
		// In hex_prism, shp1 = length of prism / d == dist betw hex faces / d
		//				shp2 = dist betw opp vertices on one hex face / d = 2 * side length / d
		//				shp3 = 1 for a1 || x and a2 || y, ..... (see documentation)
		// shapeModifiable class specializations allow for shape param setting and provide other 
		// vars, like mean radius.		

		namespace shapes
		{
			// from_file provides a target for ddscat shape.dat file processing. provides its 
			// own dipole spacing vertex, which basically goes unused.
			// TODO: allow for stretching/squeezing and aspect ratio manipulation
			//       needs knowledge of shape file statistics (like length, width, depth)
			//       Should this be in another class?
			class from_file : public shapeModifiable
			{
			public:
				from_file();
				virtual ~from_file();
				virtual from_file* clone() const { from_file *ns = new from_file(*this); return ns; }
				virtual bool canWrite() const { return true; }
				// Function to write shape.dat, not to be confused with static xml write
				virtual void write(const std::string &fname) const;
				// shape.dat file source is stored in shapeConstraints, again.
				// It is hidden in a units field. This allows splitting of multiple files and 
				// the associated generation.
				// field name is "source_filename"
			protected:
				virtual void _constructGraph();
				virtual void run(const std::string &id = "");
				virtual bool runSupported(const std::string &id = "");
			};

			/*
			// ddscat_dep provides the interdipole spacing vertex. Not used in tmatrix-derivations.
			class ddscat_dep : public shapeModifiable
			{
			protected:
				ddscat_dep();
				virtual ~ddscat_dep();
				virtual shape* clone() const { ddscat_dep *ns = new ddscat_dep(*this); return ns; }
				virtual void _constructGraph();
			protected:
				virtual void run(const std::string &id = "");
				virtual bool runSupported(const std::string &id = "");
			};

			class ellipsoid : public ddscat_dep
			{
			public:
				ellipsoid();
				virtual ~ellipsoid();
				virtual shape* clone() const { ellipsoid *ns = new ellipsoid(*this); return ns; }
				virtual void _constructGraph();
			protected:
				virtual void run(const std::string &id = "");
				virtual bool runSupported(const std::string &id = "");
			};
			*/
		}

	}
}



