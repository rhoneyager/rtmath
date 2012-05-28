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
				NUM_ALL
			};

			const char* qnames[];

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
			virtual shape* clone() const 
			{ shape* ns = new shape(this); // TODO: fix odd error here!!!
			return ns; }
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
		};

		class shapeModifiable : public shape
		{
		public:
			shapeModifiable();
			virtual ~shapeModifiable();
			void d(double newD) { set(MANIPULATED_QUANTITY::D, newD); }
			void density(double newDensity) { set(MANIPULATED_QUANTITY::DENS, newDensity); }
			void T(double newT) { set(MANIPULATED_QUANTITY::TEMP, newT); }
			void V(double newV) { set(MANIPULATED_QUANTITY::VOL, newV); }
			void reff(double newReff) { set(MANIPULATED_QUANTITY::REFF, newReff); }
			void mass(double newMass) { set(MANIPULATED_QUANTITY::MASS, newMass); }
			virtual void set(MQ var, double val);
			virtual shape* clone() const { shapeModifiable *ns = new shapeModifiable(this); return ns; }
		protected:
			// updater function tracks which value(s) to keep constant 
			// and recalculates the rest
			virtual void _update(
				const rtmath::graphs::setWeakVertex &fixed);
			void _constructGraph();
			std::set< std::shared_ptr<rtmath::graphs::vertex> > _vertices;
			boost::bimap< size_t, 
				std::shared_ptr<rtmath::graphs::vertex> > _vertexMap;
			std::shared_ptr<rtmath::graphs::graph> _graph;
		};
		
		class shapeSphere : public shapeModifiable
		{
		public:
			shapeSphere();
			virtual ~shapeSphere();
			virtual shape* clone() const { shapeSphere *ns = new shapeSphere(this); return ns; }
		protected:
			// Provide overload for shape parameters
		};
		

		namespace MANIPULATED_QUANTITY
		{
			class shapeBasicManip : public rtmath::graphs::vertexRunnable
			{
			public:
				shapeBasicManip(shapeModifiable *base, CONVERTERS varconv);
				virtual ~shapeBasicManip() {}
				virtual void run();
			private:
				shapeModifiable *_base;
				CONVERTERS _id;
			};
		}

	}
}
