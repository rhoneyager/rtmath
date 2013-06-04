#pragma once
/* atmos.h - the class definitions for an atmosphere
   An atmosphere is a collection of layers from which doubling-adding
   calculations may give properties such as intensity, flux, 
   reflectance, transmittance, heating rate, ...

   A vector holds the atmospheric layers in order. Each layer is generated 
   at a specified wavelength (sets tau and single scattering albedo), and 
   the layers are added together using the adding method. See Liou for 
   several descriptions of the adding method (he repeats himself constantly)
   */

#include <vector>
#include <map>
//#include "error/debug.h"
#include "da/daLayer.h"
#include "matrixop.h"
#include "da/damatrix.h"
#include <string>
#include <memory>
#include "atmoslayer.h"

namespace rtmath {
	//class daLayer; // List it here for reference
	//namespace lbl { class lbllayer; }; // Listed for reference

	namespace atmos {

		class atmoslayer;
		class absorber;

		class atmos {
		public:
			atmos();
			~atmos();
			atmos(const std::string &filename);
			atmos(const std::string &filename, const atmos &base);
			std::string name; // Name of the atmosphere

			// File loading and saving to various formats
			// Formats are my own text, netcdf, Liu's text format and Evans' text format
			// Overlays are extra files containing the same layers but additional information
			// The additional information could include more gases, phase functions, or
			// really anything else that loadProfile-based functions would process
			void loadProfile(const std::string &filename);
			void saveProfile(const std::string &filename) const;
			//void loadProfileRyan(const std::string &filename);
			void loadProfileRyanB(const std::string &filename);
			//void saveProfileRyan(const std::string &filename);
			void loadProfileLiu(const std::string &filename);
			//void saveProfileLiu(const std::string &filename);
			//void loadProfileCDF(const std::string &filename);
			//void saveProfileCDF(const std::string &filename);

			// Calculation of optical depth of atmosphere or parts thereof
			double tau(double f) const;
			double tau(double f, size_t layernum) const;
			// Careful: calcs from layerLow (inclusive) to high (exclusive)
			double tau(double f, size_t layerLow, size_t layerHigh) const;
		public: // Public for testing... Should change.
			std::vector<atmoslayer > _layers;
		private:
			void _init();
		};

	}; // end namespace atmos

}; // end namespace rtmath
