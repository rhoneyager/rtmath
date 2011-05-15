#include "ncvar.h"

namespace netcdf_managed {
	ncVar::ncVar(ncGroup *parent)
	{
		_parent = parent;
		varEndian = NATIVE;
		varType = NULL;
		data = NULL;
		_varid = -1;
	}

	ncVar::~ncVar()
	{
	}

};


