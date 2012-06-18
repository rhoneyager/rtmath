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
//#include <boost/archive/
#include "../matrixop.h"
#include "../coords.h"
#include "../depGraph.h"
#include "../error/error.h"

namespace rtmath {
	namespace ddscat {

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
				// Shape parameter relations
				SHPAR1,
				SHPAR2,
				SHPAR3,
				NUM_MANIPULATED_QUANTITY
			};

			// TODO: add in converters for SHPAR[1-3]
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

		namespace shape_types
		{
			enum SHAPE_TYPES
			{
				UNKNOWN,
				FROM_FILE,
				ELLIPSOID,
				HEX_PLATE,
				CYLINDER1,
				MULTISPHERES,
				NUM_SHAPE_TYPES
			};
		}

		typedef ::rtmath::ddscat::MANIPULATED_QUANTITY::MANIPULATED_QUANTITY MQ;

		class shape : public std::enable_shared_from_this<shape>
		{
		public:
			shape();
			virtual ~shape();
			double d() const { return get(MANIPULATED_QUANTITY::D); }
			double density() const { return get(MANIPULATED_QUANTITY::DENS); }
			double T() const { return get(MANIPULATED_QUANTITY::TEMP); }
			double V() const { return get(MANIPULATED_QUANTITY::VOL); }
			double reff() const { return get(MANIPULATED_QUANTITY::REFF); }
			double mass() const { return get(MANIPULATED_QUANTITY::MASS); }
			virtual double get(MQ var) const;
			virtual shape* clone() const { shape* ns = new shape(*this); return ns; }
			virtual bool canWrite() const { return false; }
			virtual void write(const std::string &fname) const;
		protected:
			std::map<MQ, double> _vars;
			//double _d; // Interdipole spacing, in um
			//double _V; // Volume, in um^3
			//double _reff; // reff in um
			//double _mass; // mass in kg
			//double _density; // kg / um^3
			//double _T; // Temperature, K, used in water density calculations
			// Densities of different anisotropic materials. Used in mass and inertia calculations.
			std::map<size_t, double> _densities;
			friend class boost::serialization::access;
		public:
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
			virtual shape_types::SHAPE_TYPES type() const { return shape_types::UNKNOWN; }
			void d(double newD) { set(MANIPULATED_QUANTITY::D, newD); }
			void density(double newDensity) { set(MANIPULATED_QUANTITY::DENS, newDensity); }
			void T(double newT) { set(MANIPULATED_QUANTITY::TEMP, newT); }
			void V(double newV) { set(MANIPULATED_QUANTITY::VOL, newV); }
			void reff(double newReff) { set(MANIPULATED_QUANTITY::REFF, newReff); }
			void mass(double newMass) { set(MANIPULATED_QUANTITY::MASS, newMass); }
			virtual void set(MQ var, double val);
			virtual shape* clone() const { shapeModifiable *ns = new shapeModifiable(*this); return ns; }
			inline void update(const rtmath::graphs::setWeakVertex &fixed) { _update(fixed); }
			void getVertices(vertexMap &mappings);
			bool mapVertex(const std::string &idstr, size_t &id, std::shared_ptr<rtmath::graphs::vertex> &vertex);

			// TODO: allow function to change aspect ratio / dipole spacing prototype
		protected:
			// updater function tracks which value(s) to keep constant 
			// and recalculates the rest
			virtual void _update(
				const rtmath::graphs::setWeakVertex &fixed);
			virtual void _constructGraph();
			std::set< std::shared_ptr<rtmath::graphs::vertex> > _vertices;
			vertexMap _vertexMap;
			vertexIdMap _vertexIdMap;
			std::shared_ptr<rtmath::graphs::graph> _graph;
			friend class boost::serialization::access;
		public:
			template<class Archive>
				void serialize(Archive & ar, const unsigned int version)
				{
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
		class shapeEllipsoid : public shapeModifiable
		{
		public:
			shapeEllipsoid();
			virtual ~shapeEllipsoid();
			virtual shape_types::SHAPE_TYPES type() const { return shape_types::ELLIPSOID; }
			virtual shape* clone() const { shapeEllipsoid *ns = new shapeEllipsoid(*this); return ns; }
		protected:
			// Provide overload for shape parameters
			virtual void _constructGraph(); // Override function to add connectors to shape params
		};
		

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
