#pragma once
#include "../defs.h"
#include "rotations.h"
#include <array>
#include <functional>
#include <map>
#include <set>
#include <vector>

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

			/// 1d interval table and weights definition
			namespace IntervalTable1dDefs
			{
				enum IntervalTable1dDefs
				{
					MIN,
					MAX,
					PIVOT,
					WEIGHT_RAW,
					DEGENERACY,
					WEIGHT_DEGEN,
					NUM_ENTRIES_IntervalTable1dDefs
				};
			}
			/// The newer 1d table definition
			typedef std::array<double, IntervalTable1dDefs::NUM_ENTRIES_IntervalTable1dDefs> IntervalTable1dEntry;
			/// Container for storing data of intervals, degeneracies and weights
			typedef std::vector<IntervalTable1dEntry> IntervalTable1d;

			/// 3d interval table definitions
			namespace IntervalTable3dDefs
			{
				enum IntervalTable3dDefs
				{
					BETA_MIN,
					BETA_MAX,
					BETA_PIVOT,
					THETA_MIN,
					THETA_MAX,
					THETA_PIVOT,
					PHI_MIN,
					PHI_MAX,
					PHI_PIVOT,
					WEIGHT,
					NUM_ENTRIES_IntervalTable3dDefs
				};
			}
			/// The newer 3d table definition
			typedef std::array<double, IntervalTable3dDefs::NUM_ENTRIES_IntervalTable3dDefs> IntervalTable3dEntry;
			/// Container for storing data of intervals, degeneracies and weights
			typedef std::vector<IntervalTable3dEntry> IntervalTable3d;

			/// Base class that handles independent weighting in one direction
			class DLEXPORT_rtmath_ddscat ddWeights
			{
			public:
				virtual ~ddWeights() {}
				/// Number of points
				virtual size_t size() const { return weights.size(); }
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
				/// Provide a copy of the newer interval table
				void getIntervals(IntervalTable1d &intervals1d) const;
				// Get a random point in the interval range
				//virtual double getRandomPoint() const = 0;
				// Get random points in the interval range
				//virtual void getRandomPoints(size_t n, std::vector<double> &) const = 0;
			protected:
				ddWeights(double start = 0, double end = 0, size_t n = 0);
				/// Start point
				double start;
				/// End point
				double end;
				/// Number of points
				size_t n;
				/// Weight at each coordinate
				IndependentWeights weights;
				/// Degeneracy table (may be used in derived classes)
				DegeneracyTable freqs;
				/// Interval table (for each subinterval midpoint, contains the 
				/// subinterval bounds). Useful in CDF calculations.
				IntervalTable intervals;
				/// Replacement interval table
				IntervalTable1d intervals1d;

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
				//virtual double getRandomPoint() const override;
				//virtual void getRandomPoints(size_t n, std::vector<double> &) const override;
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
				//virtual double getRandomPoint() const override;
				//virtual void getRandomPoints(size_t n, std::vector<double> &) const override;
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

				/// Provide a copy of the weighting table
				void getIntervalTable(IntervalTable3d&) const;

				/// Get number of beta orientations
				size_t numBetas() const { return wBetas.size(); }
				/// Get number of theta orientations
				size_t numThetas() const { return wThetas.size(); }
				/// Get number of phi orientations
				size_t numPhis() const { return wPhis.size(); }
				/// Get number of orientations
				size_t size() const { return wBetas.size() * wThetas.size() * wPhis.size(); }
			private:
				void _init();
				ddWeightsCosInt wThetas;
				ddWeightsLinInt wPhis;
				ddWeightsLinInt wBetas;
				IntervalTable3d IntervalWeights;
			};

			/// Generates weights for imported DDSCAT results with a statistically random orientation distribution
			/*
			class DLEXPORT_rtmath_ddscat ddWeightsDDSCATrandom
			{
			public:
				/// Constructor that takes the number of rotations considered
				ddWeightsDDSCATrandom(size_t n);
				virtual ~ddWeightsDDSCATrandom() {}
				/// Gets the weight for a given orientation
				double getWeight(double beta, double theta, double phi) const;
				/// Get the number of orientations
				size_t size() const {return n;}
			protected:
				size_t n;
			};
			*/

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
				virtual ~Uniform1dWeights() {};
			};

			/// Von Mises Distribution
			class DLEXPORT_rtmath_ddscat VonMisesWeights
				: public OrientationWeights1d
			{
			public:
				virtual ~VonMisesWeights() {};
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

			
			/// Base class for weighting distributions in 3d space
			class DLEXPORT_rtmath_ddscat OrientationWeights3d
			{
			public:
				typedef IntervalTable3d weightTable;
				virtual ~OrientationWeights3d();
				/// Provide a copy of the weighting table
				void getWeights(IntervalTable3d &weights) const;
			protected:
				OrientationWeights3d();
				IntervalTable3d weights;
			};

			/** \brief Provides weights for a Von Mises-Fisher distribution
			*
			* \note The static methods of this class are quite capable of handling 
			* higher dimensions. This class will eventually inherit from 
			* a vMF base class that will provide these methods.
			*
			* \todo Extend class to lower and higher dimensions.
			**/
			class DLEXPORT_rtmath_ddscat VonMisesFisherWeights
				: public OrientationWeights3d
			{
			public:
				virtual ~VonMisesFisherWeights() {};
				/** \brief Construct Von Mises-Fisher Weights for a ddWeightsDDSCAT interval.
				*
				* Here, mean and kappa are in degrees, for DDSCAT consistency.
				**/
				VonMisesFisherWeights(const ddWeightsDDSCAT&, double muT, double muP, double kappa);
				/** \brief Calculate Von Mises-Fisher PDF.
				*
				* x and mus should be in cylindrical coordinates (see Sra 2007 and 2012).
				* \see degToSph
				**/
				static double VonMisesFisherPDF(size_t degree, const double *x, 
					const double *mu, double kappa);
				/** \brief Calculate Von Mises-Fisher CDF
				*
				* x and mus should be in cylindrical coordinates (see Sra 2007 and Sra 2012).
				* \see degToSph
				**/
				static double VonMisesFisherCDF(size_t degree, const double *x1, 
					const double *x2, const double *mu, double kappa);
				/** \brief Convert from angles (radians) to Cartesian coordinates, 
				* needed for the PDF and CDF functions.
				*
				* \param n is the vector space dimension
				* \param in is the input point of the form u(r,theta), theta = (t1, ..., t_n-1), and r = mag(x)
				* \param out is the output point in Cartesian coordinates
				**/
				static void radSphToCrt(size_t n, const double *in, double *out);
			protected:
				double meanTheta;
				double meanPhi;
				double kappa;
			};

			/// Provides a bimodal vMF distribution, which better matches flake results
			class DLEXPORT_rtmath_ddscat BimodalVonMisesFisherWeights
				: public OrientationWeights3d
			{
			public:
				virtual ~BimodalVonMisesFisherWeights() {};
				/** \brief Construct Bimodal Von Mises-Fisher Weights for a ddWeightsDDSCAT interval.
				*
				* Here, mean and kappa are in degrees, for DDSCAT consistency.
				**/
				BimodalVonMisesFisherWeights(const ddWeightsDDSCAT&, double muT, double muP, double kappa);
			protected:
				double meanTheta;
				double meanPhi;
				double kappa;
			};

			/// Provides weighting based on the base DDSCAT weights
			class DLEXPORT_rtmath_ddscat DDSCAT3dWeights
				: public OrientationWeights3d
			{
			public:
				virtual ~DDSCAT3dWeights() {};
				/** \brief Construct DDSCAT Weights based on a ddWeightsDDSCAT interval.
				*
				* This is a trivial adapter to allow 3d weights to be computed easily 
				* using shared pointers to OrientationWeights3d objects.
				**/
				DDSCAT3dWeights(const ddWeightsDDSCAT&);
			};

			/**
			 * \brief Adds weighting function support options to a program
			 *
			 * \item cmdline provides options only allowed on the command line
			 * \item config provides options available on the command line and in a config file
			 * \item hidden provides options allowed anywhere, but are not displayed to the user
			 **/
			//void add_options(
			//	boost::program_options::options_description &cmdline,
			//	boost::program_options::options_description &config,
			//	boost::program_options::options_description &hidden);
			/// Processes options defined in add_options
			//void process_static_options(
			//	boost::program_options::variables_map &vm);

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

