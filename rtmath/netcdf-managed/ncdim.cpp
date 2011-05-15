#include "ncdim.h"

namespace netcdf_managed {
	ncDim::ncDim(ncGroup* parent)
	{
		_parent = parent;
		_dimid = -1;
	}

	ncDim::~ncDim()
	{
	}
};

