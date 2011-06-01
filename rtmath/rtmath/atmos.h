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
#include "layer.h"
#include "../rtmath-base/matrixop.h"
#include "damatrix.h"

namespace rtmath {

	class atmos {
	public:
		atmos();
		~atmos();
		std::vector<dalayer> layers;
		// For now, calculate at just one wavelength, until dalayer set 
		// has support for wavenumber cacheing (different tau, ssa yield 
		// different results at different wavenumbers)
		void spec(double wvnum);
		double spec();
	private:
		void _calcProps();
		double _wvnum;
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
	};

}; // end namespace rtmath


