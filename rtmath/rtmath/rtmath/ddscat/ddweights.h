#pragma once
#include "../defs.h"
#include "rotations.h"
#include <functional>
#include <map>
#include <set>

namespace rtmath {

	namespace ddscat {
		class rotations;

		/**
		* \brief Contains weighting code for integration intervals in 
		* DDSCAT and environment-frame rt modeling.
		**/
		namespace weights {

			/// Weights (without degeneracy): (point, weight)
			typedef std::map<double,double> IndependentWeights;
			/// Records degeneracy: (point, degeneracy)
			typedef std::map<double,size_t> DegeneracyTable;
			/// Records interval bounds: (point, (min, max) )
			typedef std::map<double, std::pair<double, double> > IntervalTable;

			/// Base class that handles independent weighting in one direction
			class DLEXPORT_rtmath_ddscat ddWeights
			{
			public:
				virtual ~ddWeights() {}
				/// Base weight
				virtual double weightBase(double point) const;
				/// Weight (accounting for degeneracy)
				virtual double weightDegen(double point) const;
				/// Find the interval that encompasses the provided point
				virtual bool interval(double point,
					double &start, double &end, double &pivot) const;

				/// Provide a copy of the weighting table
				void getWeights(IndependentWeights &weights) const;
				/// Provide a copy of the degeneracy table
				void getFreqs(DegeneracyTable &freqs) const;
				/// Provide a copy of the interval table
				void getIntervals(IntervalTable &intervals) const;
			protected:
				ddWeights() {}
				/// Weight at each coordinate
				IndependentWeights weights;
				/// Degeneracy table (may be used in derived classes)
				DegeneracyTable freqs;
				/// Interval table (for each subinterval midpoint, contains the 
				/// subinterval bounds). Useful in CDF calculations.
				IntervalTable intervals;

				/// Fill the interval table
				void fillIntervalTable(double start, double end, 
					const std::function<double(double,double)> &midpointFunction);
			};

			/// Generate linearly-spaced weights
			class DLEXPORT_rtmath_ddscat ddWeightsLinInt
				: public ddWeights
			{
			public:
				ddWeightsLinInt(double start, double end, size_t n);
				virtual ~ddWeightsLinInt() {};
			};

			/// \brief Generate cosine-spaced weighting points, following 
			/// DDSCAT's convention about weights for odd or even 
			/// interval numbers.
			class DLEXPORT_rtmath_ddscat ddWeightsCosInt
				: public ddWeights
			{
			public:
				ddWeightsCosInt(double start, double end, size_t n);
				virtual ~ddWeightsCosInt() {};
			};

			/// Generates weights for imported DDSCAT results
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

				/// Gets the final weight for a given orientation
				double getWeight(double beta, double theta, double phi) const;
				/// Gets the weight of a1 (theta, phi) for a given orientation
				double getWeight(double theta, double phi) const;
				/// Gets the weight of a2 (beta) for a given orientation
				double getWeight(double beta) const;

				typedef std::pair<double, double> dp;
				/**
				* \brief Gets the interval 'size' for a given orientation.
				*
				* Used in mapping the orientation distribution function 
				* based on the interval CDF.
				**/
				void getIntervalBounds(double beta, double theta, double phi,
					dp &intBeta, dp &intTheta, dp &intPhi) const;
			private:
				ddWeightsCosInt wThetas;
				ddWeightsLinInt wPhis;
				ddWeightsLinInt wBetas;
			};


			/// Base class for weighting distributions in 1d space
			class DLEXPORT_rtmath_ddscat OrientationWeights1d
			{
			public:
				/// weightTable stores weights based on min interval point and the cdf weight.
				typedef std::map<double, double> weightTable;
			protected:
				/// Is the 1d space cyclic?
				const bool cyclic;
				/// Minimum and maximum bounds
				double min, max, span;
				/// The weighting table
				weightTable weights;
				OrientationWeights1d(bool cyclic);
				/// Calculate overall span
				void calcSpan(const ddWeights&, double &min, double &max, double &span) const;
			public:
				virtual ~OrientationWeights1d();
				/// Provide a copy of the weighting table
				void getWeights(weightTable &weights) const;
			};

			/// Uniform 1d distribution
			class DLEXPORT_rtmath_ddscat Uniform1dWeights
				: public OrientationWeights1d
			{
			public:
				Uniform1dWeights(const ddWeights&);
			};

			/// Von Mises Distribution
			class DLEXPORT_rtmath_ddscat VonMisesWeights
				: public OrientationWeights1d
			{
			public:
				/** \brief Construct Von Mises Weights for a ddWeights interval.
				*
				* Here, mean and kappa are in degrees, for DDSCAT consistency.
				**/
				VonMisesWeights(const ddWeights&, double mean, double kappa);
				/** \brief Calculate Von Mises PDF.
				*
				* x and mu should be in radians.
				**/
				static double VonMisesPDF(double x, double mu, double kappa);
				/** \brief Calculate Von Mises CDF
				*
				* x and mu should be in radians.
				**/
				static double VonMisesCDF(double x, double mu, double kappa);
			protected:
				double mean;
				double kappa;
			};

			/*
			/// Base class for weighting distributions in 3d space
			class DLEXPORT_rtmath_ddscat OrientationWeights3d
			{
			protected:
				OrientationWeights3d();
			public:
				virtual ~OrientationWeights3d();
			};

			/// Provides weights for a Von Mises-Fisher distribution
			class DLEXPORT_rtmath_ddscat VonMisesFischerWeights
				: public OrientationWeights3d
			{
			};

			/// Provides weights for a Kent Distribution
			class DLEXPORT_rtmath_ddscat KentWeights
				: public OrientationWeights3d
			{
			};

			*/

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

}

