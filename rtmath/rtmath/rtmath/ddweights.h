#pragma once
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <bitset>
#include <cstdio>
#include <cstring>
#include <complex>
#include "matrixop.h"
#include "phaseFunc.h"
#include "cdf-ddscat.h"

// Needs extensive use of filesystem
// (for reading whole directories, manipulating paths, ...)
#include <boost/filesystem.hpp>

/* ddweights - The ddscat code was becoming too large, so the weighting functions
 * are now located here */


namespace rtmath {

	namespace ddscat {

		class weights
		{
		public:
			weights() {}
			virtual ~weights() {}
			virtual double weight(double point) const = 0;
			void getWeights(std::map<double,double> &weights) const;
		protected:
			std::map<double,double> _pointWeights;
			
		};

		// Gaussian weights where the point is always guaranteed to be positive
		// It's really double gaussian on an interval of [0,infinity)
		class gaussianPosWeights : public weights
		{
		public:
			gaussianPosWeights(double sigma, const std::set<double> &points);
			virtual ~gaussianPosWeights();
			virtual double weight(double point) const;
			inline double mu() const {return 0;}
			inline double sigma() const {return _sigma;}
		protected:
			double _mu;
			double _sigma;
		};

	};

};

