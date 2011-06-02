#include "Stdafx.h"
#include "atmos.h"
#include <vector>
#include <map>
#include "layer.h"
#include "../rtmath-base/matrixop.h"
#include "damatrix.h"


namespace rtmath {

	atmos::atmos()
	{
	}

	atmos::~atmos()
	{
	}

	double atmos::spec()
	{
		return _wvnum;
	}

	void atmos::spec(double wvnum)
	{
		_wvnum = wvnum;
	}

	void atmos::_calcProps()
	{
		throw;
		// Have each layer generate its base properties
		// Then, add downward, then upward to get appropriate R and T
		// Next, calculate intensities and fluxes
		// Convert to get brightness temperatures
	}


};

