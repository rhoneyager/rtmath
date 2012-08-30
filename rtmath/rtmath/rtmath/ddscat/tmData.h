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
#include "shapestats.h"

// Forward declaration for boost::serialization below
namespace rtmath {
	namespace tmatrix {
		class tmData;
	}
}

namespace tmatrix
{
	class tmatrixSet;
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
		class tmData
		{
		public:
			std::string ddparpath;
			double dipoleSpacing, T, freq, nu;
			std::complex<double> reff;
			std::string volMeth, dielMeth, shapeMeth, angleMeth;

			boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;

			std::vector<boost::shared_ptr<::tmatrix::tmatrixSet> > data;
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, tmData &, const unsigned int);
		};
	}
}
