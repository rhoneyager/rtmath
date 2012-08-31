#pragma once
/* This is the container for tmatrix raw data, used by the tmatrix-convert-from-ddscat program.
 * It represents the serialized run data. For further comparisons, it provides several outputs 
 * to make the data manipulable in the same manner as ddscat data.
 *
 * tmatrix data easily provides scattering information but gives no absorption calculations.
 * Absorptive calculations are required for a full rt solution.
 */

#include <boost/shared_ptr.hpp>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <complex>
#include <boost/cstdint.hpp>
#include "shapestats.h"

// Forward declaration for boost::serialization below
namespace rtmath {
	namespace tmatrix {
		struct tmRun;
		class tmData;
	}
}

// Need these so the template friends can work
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive &, rtmath::tmatrix::tmData &, const unsigned int);
	}
}

namespace rtmath
{
	namespace tmatrix
	{
		// I am providing structures equivalent to the 
		// tmatrix-provided ones. This is because I 
		// do not want rtmath to have a direct dependence 
		// on the tmatrix library.

		// A conversion library will be provided for 
		// programs, but it will be at a higher level.
		struct tmIn
		{
			tmIn();
			double axi, rat, lam, mrr, mri,
			       eps, ddelt, alpha, beta,
			       thet0, thet, phi0, phi;
			boost::int32_t np, ndgs;
		};

		struct tmOut
		{
			tmOut();
			std::complex<double> S[4];
			double P[4][4];
		};

		struct tmRun
		{
			tmIn invars;
			tmOut res;
		};

		class tmData
		{
		public:
			std::string ddparpath;
			double dipoleSpacing, T, freq, nu;
			std::complex<double> reff;
			std::string volMeth, dielMeth, shapeMeth, angleMeth;

			boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;

			std::vector<boost::shared_ptr< tmRun > > data;
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, tmData &, const unsigned int);
		};
	}
}
