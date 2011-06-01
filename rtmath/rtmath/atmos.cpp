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
	}


};

