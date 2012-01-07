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
#include "layer.h"
#include "matrixop.h"
#include "damatrix.h"
#include "lbl.h"
#include <string>

namespace rtmath {
	class dalayer;
	namespace lbl { class lbllayer; };

	class atmos {
	public:
		atmos();
		~atmos();
		// The doubling-adding layers
		//  for scattering
		std::vector<dalayer> dalayers;
		// The lbl layers
		//  for calculation of tau
		std::vector<lbl::lbllayer> lbllayers;

		std::string name;
		// For now, calculate at just one wavelength, until dalayer set 
		// has support for wavenumber cacheing (different tau, ssa yield 
		// different results at different wavenumbers)
		inline void nu(double wvnum) { _wvnum = wvnum; _taus.clear(); }
		inline double nu() { return _wvnum; }
		
		// Calculation of optical depth of the entire atmosphere
		double tau();

		// Smart function to load in a profile with corresponding gas 
		// concentrations, scattering information, phase functions, ...
		//  or any subset of the above
		// For absorption-only, only the gas concentrations are really needed
		// For merging profile data, consider using a better function
		void loadProfile(const char* filename);

	private:
		void _calcProps();
		double _wvnum;
		std::vector<double> _taus;
		// R and T for each layer are provided for by dalayers.
	public:
		// The functions that return the model calculations
		/*
		double Fup(std::vector<dalayer>::const_iterator low, 
			std::vector<dalayer>::const_iterator high);
		double Fdown(std::vector<dalayer>::const_iterator low, 
			std::vector<dalayer>::const_iterator high);
		double Fnet(std::vector<dalayer>::const_iterator low, 
			std::vector<dalayer>::const_iterator high);
			*/
		void RTd(size_t low, size_t high, damatrix *Rres, damatrix *Tres);
		/*
		void Ru(size_t low, size_t high, std::shared_ptr<damatrix> &res);
		void Td(size_t low, size_t high, std::shared_ptr<damatrix> &res);
		void Tu(size_t low, size_t high, std::shared_ptr<damatrix> &res);
		*/
	};

}; // end namespace rtmath


