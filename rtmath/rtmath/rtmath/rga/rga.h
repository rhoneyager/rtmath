#pragma once
#include "../defs.h"
#include <complex>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>

/// This header implements the Rayleigh-Gans approximations.

namespace rtmath {
	namespace ddscat {
		class rotations;
		namespace shapefile {
			class shapefile;
		}
	}
	namespace rga {

		// The RG code can directly compute the RG approximation for a
		// shape at a specified orientation. This is easy to do since
		// it is a direct numerical evaluation.
		// Just need the refractive index of ice, the frequency and
		// the structural terms.
		struct DLEXPORT_rtmath_ddscat cross_sections {
			double Cbk, Cext, Csca, Cabs, g;
			std::string Cunits;
			bool valid;
			boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> shp;
			double dSpacing;
			cross_sections();
		};

		/// This is a very simple implementation of Rayleigh-Gans theory
		/// It calculates only backscatter.
		class DLEXPORT_rtmath_ddscat rgSimple {
			private:
				rgSimple();
				double dSpacing;
				boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile> shp;
				//boost::shared_ptr<const ::rtmath::ddscat::rotations> rots;
			public:
				virtual ~rgSimple();
				static boost::shared_ptr<const rgSimple> generate(
					boost::shared_ptr<const ::rtmath::ddscat::shapefile::shapefile>,
					double dSpacing = 0);

				/// Calculate backscatter and average over a set or rotations
				//boost::shared_ptr<const cross_sections>
				//	run(double freqGHz, double tempK,
				//		boost::shared_ptr<const ::rtmath::ddscat::rotations>) const;
				/// Calculate backscatter for a particular orientation
				boost::shared_ptr<const cross_sections>
					run(double freqGHz, double tempK, double betad,
						double thetad, double phid) const;


		};

		// Can also attempt to perform this using Robin Hogan's methods
	}
}

