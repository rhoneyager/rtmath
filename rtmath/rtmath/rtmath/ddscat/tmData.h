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
#include <tmatrix/tmatrix.h>
#include "../mie/mie.h"
#include "shapestats.h"

// Forward declaration for boost::serialization below
namespace rtmath {
	namespace tmatrix {
		class tmData;
		class tmStats;
	}
}

// Need these so the template friends can work
namespace boost
{
	namespace serialization
	{
		template <class Archive>
		void serialize(Archive &, rtmath::tmatrix::tmStats &, const unsigned int);
		template <class Archive>
		void serialize(Archive &, rtmath::tmatrix::tmData &, const unsigned int);
	}
}

namespace rtmath
{
	namespace tmatrix
	{
		// rtmath can have a dependency on the tmatrix 
		// serialization code, since the tmatrix library can 
		// be built without fortran code. However, the 
		// tmatrix library must still be built.

		// Stats class
		class tmStats
		{
		public:
			tmStats();
			// Has Qsca, Qabs, Qbk for different phi
			//std::map<double, std::map<std::string, double> > stats;
			std::map<std::string, double> stats;

			// And include a nested tmStats on a per-rotation basis.
			//std::set<boost::shared_ptr<tmStats> > nested;

			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, tmStats &, const unsigned int);
		};

		class tmData
		{
		public:
			tmData();
			std::string ddparpath;
			std::string dielpath;
			double dipoleSpacing, T, freq, nu, sizep;
			std::complex<double> reff;
			std::string volMeth, dielMeth, shapeMeth, angleMeth;

			boost::shared_ptr<rtmath::ddscat::shapeFileStats> stats;

			std::vector<boost::shared_ptr< const ::tmatrix::OriAngleRes > > data;
			std::vector<boost::shared_ptr< const rtmath::mie::mieAngleRes> > miedata;
			boost::shared_ptr<tmStats> tstats;
			template<class Archive> 
			friend void ::boost::serialization::serialize(
				Archive &, tmData &, const unsigned int);
		};
	}
}
