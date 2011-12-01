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
#include "debug.h"
#include "daLayer.h"
#include "matrixop.h"
#include "damatrix.h"
#include "lbl.h"
#include <string>
#include <memory>
//#include <netcdfcpp.h>

namespace rtmath {
	class daLayer; // List it here for reference
	namespace lbl { class lbllayer; }; // Listed for reference

	class atmos {
	public:
		atmos();
		~atmos();
		std::string name;						// Name of this atmosphere
		// Containers
		std::vector<std::shared_ptr<daLayer> > daLayers;			// The doubling-adding layers in the atmosphere
		std::vector<lbl::lbllayer> lblLayers;	// The line-by-line layers
		// Functions to set and get the target wavelength
		inline void nu(double wvnum) { _wvnum = wvnum; _taus.clear(); }
		inline double nu() { return _wvnum; }
		// Calculation of the optical depth in the atmosphere or in parts thereof
		double tau();
		double tau(unsigned int layernum);
		double tau(unsigned int layerlow, unsigned int layerhigh);

		// Smart function to load a profile with the corresponding gas
		// concentrations, scattering information, phase functions, ..., or any 
		// subset of the above.
		// For absorption-only, only the gas concentrations are really needed.
		// For merging profile data, a better function must be implemented.
		void loadProfile(const char* filename); // Only handles absorption information only.
		/* Load in a profile from netCDF
		 * Profile contains the name of the atmosphere, the lbl layers,
		 * the lbl layer gas concentrations, the corresponding daLayers, 
		 * their albedos and phase function information.
		 * The phase functions may be simple, but are typically a complex angle-dependent 
		 * construct. There are several ways of storing the phase functions, but they really 
		 * should contain information on the particle scattering schemes and any necessary parameters 
		 * to reproduce the phase matrix. This will use identifiers, and will call the appropriate loading functions.
		 * It should rely on both attributes and variables to work properly.
		 */
		/*
		void loadProfile(NcFile * nfile, const char* profileName); 
		inline void saveProfile(NcFile * nfile) { saveProfile(nfile, name.c_str()); }
		void saveProfile(NcFile * nfile, const char* profileName);
		*/

		// Functions that perform adding-doubling on entire layer
		std::shared_ptr<damatrix> R();
		std::shared_ptr<damatrix> T();
	protected:
		std::shared_ptr<damatrix> _R, _T;		// These hold the pointers to any preconstructed R and T for the atmosphere
	private:
		void _calcProps();
		double _wvnum;
		std::vector<double> _taus;

	};

}; // end namespace rtmath
