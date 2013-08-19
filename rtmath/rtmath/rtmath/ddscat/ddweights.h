#pragma once
#include "../defs.h"
#include "rotations.h"
#include <map>
#include <set>

/* ddweights - The ddscat code was becoming too large, so the weighting functions
 * are now located here */


namespace rtmath {

	namespace ddscat {
		class rotations;

		/// Base class that handles independent weighting in one direction
		class DLEXPORT_rtmath_ddscat ddWeights
		{
		public:
			virtual ~ddWeights() {}
			/// Base weight
			virtual double weightBase(double point) const;
			/// Weight (accounting for degeneracy)
			virtual double weightDegen(double point) const;
			void getWeights(std::map<double,double> &weights) const;
			void getFreqs(std::map<double,size_t> &freqs) const;
		protected:
			ddWeights() {}
			typedef std::map<double,double> IndependentWeights;
			/// Weight at each coordinate
			IndependentWeights weights;
			/// Degeneracy table (may be used in derived classes)
			std::map<double,size_t> freqs;
		};

		/// Generate linearly-spaced weights
		class DLEXPORT_rtmath_ddscat ddWeightsLinInt
			: public ddWeights
		{
		public:
			ddWeightsLinInt(double start, double end, size_t n);
			virtual ~ddWeightsLinInt() {};
		};

		/// Generate cosine-spaced weighting points, following 
		/// DDSCAT's convention about weights for odd or even 
		/// interval numbers.
		class DLEXPORT_rtmath_ddscat ddWeightsCosInt
			: public ddWeights
		{
		public:
			ddWeightsCosInt(double start, double end, size_t n);
			virtual ~ddWeightsCosInt() {};
		};

		class DLEXPORT_rtmath_ddscat ddWeightsDDSCAT
		{
		public:
			/// Constructor that takes raw rotation values.
			ddWeightsDDSCAT(
				double bMin, double bMax, size_t nB,
				double tMin, double tMax, size_t nT,
				double pMin, double pMax, size_t nP);
			/// Constructor that extracts the rotations.
			ddWeightsDDSCAT(const rotations&);
			virtual ~ddWeightsDDSCAT() {}

			double getWeight(double beta, double thata, double phi) const;
		private:
			ddWeightsCosInt wThetas;
			ddWeightsLinInt wPhis;
			ddWeightsLinInt wBetas;
		};

		/*
		/// Gaussian weights where the point is always guaranteed to be positive. 
		/// It's really double gaussian on an interval of [0,infinity)
		class DLEXPORT_rtmath_ddscat gaussianPosWeights : public ddWeights
		{
		public:
			gaussianPosWeights(double sigma, const std::multiset<double> &points);
			virtual ~gaussianPosWeights();
			//virtual double weight(double point) const;
			inline double mu() const {return 0;}
			inline double sigma() const {return _sigma;}
		protected:
			double _mu;
			double _sigma;
		};

		/// Isotropic weighting scheme. 
		/// Useful for comparing with ddscat ensemble result
		class DLEXPORT_rtmath_ddscat isoPosWeights : public ddWeights
		{
		public:
			isoPosWeights(const std::multiset<double> &points);
			virtual ~isoPosWeights();
			//virtual double weight(double point) const;
		};
		*/

	}

}

