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
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/map.hpp>
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

		class shapeConstraint
		{
		public:
			shapeConstraint();
			shapeConstraint(
				const std::string &varname,
				const std::string &psetShorthand,
				const std::string &units);
			virtual ~shapeConstraint();
			bool operator< (const shapeConstraint &rhs) const;
			bool operator==(const shapeConstraint &rhs) const;
			bool operator!=(const shapeConstraint &rhs) const;
			typedef paramSet<double>::const_iterator const_iterator;
			typename const_iterator begin() const;
			typename const_iterator end() const;
			typename const_iterator rbegin() const;
			typename const_iterator rend() const;
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

		namespace MANIPULATED_QUANTITY
		{
			enum MANIPULATED_QUANTITY
			{
				NONE,
				D,
				DENS,
				TEMP,
				VOL,
				REFF,
				MASS,
				// Size parameter relations
				IREFR_R,
				IREFR_IM,
				FREQ,
				SIZEP,
				NUM_MANIPULATED_QUANTITY
			};

			enum CONVERTERS
			{
				DENS_T = NUM_MANIPULATED_QUANTITY,
				T_DENS,
				REFF_V,
				V_REFF,
				MASS_V__DENS,
				MASS_DENS__V,
				DENS_V__MASS,
				FREQ_TEMP__IREFR_R,
				FREQ_TEMP__IREFR_I,
				NUM_ALL
			};

			extern const char* qnames[NUM_ALL];
		}

		class shape : public std::enable_shared_from_this<shape>
		{
		public:
			shape();
			virtual ~shape();
			virtual double get(const std::string &var) const;
			virtual shape* clone() const { shape* ns = new shape(*this); return ns; }
			virtual bool canWrite() const { return false; }
			virtual void write(const std::string &fname) const;
		protected:
			// Used to take a MANIPULATED_QUANTITY until switched over to string
			std::map<std::string, double> _vars;
			// Densities of different anisotropic materials. Used in mass and inertia calculations.
			std::map<size_t, double> _densities;
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					ar & BOOST_SERIALIZATION_NVP(_vars);
					ar & BOOST_SERIALIZATION_NVP(_densities);
				}
		};

		class shapeModifiable : public shape
		{
		public:
			typedef boost::bimap< size_t, std::shared_ptr<rtmath::graphs::vertex> > vertexMap;
			typedef boost::bimap< std::string, size_t > vertexIdMap;
			typedef rtmath::graphs::setWeakVertex vertexSet;
			shapeModifiable();
			virtual ~shapeModifiable();
			virtual void set(const std:string &var, double val);
			virtual shape* clone() const { shapeModifiable *ns = new shapeModifiable(*this); return ns; }
			inline void update(const rtmath::graphs::setWeakVertex &fixed) { _update(fixed); }
			void getVertices(vertexMap &mappings);
			virtual bool mapVertex(const std::string &idstr, size_t &id, std::shared_ptr<rtmath::graphs::vertex> &vertex);
		protected:
			virtual void _update(
				const rtmath::graphs::setWeakVertex &fixed);
			virtual void _constructGraph();
			std::set< std::shared_ptr<rtmath::graphs::vertex> > _vertices;
			vertexMap _vertexMap;
			vertexIdMap _vertexIdMap;
			std::shared_ptr<rtmath::graphs::graph> _graph;
			friend class boost::serialization::access;
		private:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
					// TODO: save the vertex maps!!!
					GETOBJKEY();
					ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(shape);
				}
		};
		
		// In ellipsoid, 1/4 = (x/d*shp1)^2 + (y/d*shp2)^2 + (z/d*shp3)^2
		// In cylinder, shp1 = length/d, shp2 = diam/d,
		//		shp3 = 1 for a1 || x, 2 for a1|| y, 3 for a1 || z
		// In hex_prism, shp1 = length of prism / d == dist betw hex faces / d
		//				shp2 = dist betw opp vertices on one hex face / d = 2 * side length / d
		//				shp3 = 1 for a1 || x and a2 || y, ..... (see documentation)
		// shapeModifiable class specializations allow for shape param setting and provide other 
		// vars, like mean radius.		

		namespace MANIPULATED_QUANTITY
		{
			class shapeBasicManip : public rtmath::graphs::vertexRunnable
			{
			public:
				shapeBasicManip(shapeModifiable *base, CONVERTERS varconv) : _base(base), _id(varconv) {}
				virtual ~shapeBasicManip() {}
				virtual void run();
			private:
				shapeModifiable *_base;
				CONVERTERS _id;
			};

			// Provide shape parameters here
			class shapeEllipsoidManip : public rtmath::graphs::vertexRunnable
			{
			public:
				shapeEllipsoidManip(shapeModifiable *base, const std::string &varconv) : _base(base), _id(varconv) {}
				virtual ~shapeEllipsoidManip() {}
				virtual void run();
			private:
				shapeModifiable *_base;
				std::string _id;
			};
		}

	}
}
