#include "nctype.h"

namespace netcdf_managed {
	ncType::ncType(ncGroup *parent)
	{
		_parent = parent;
		typesize = 0;
		nctypeclass = 0;
		basetype = false;
		castfunction = NULL;
		_typeid = -1;
	}

	ncType::~ncType()
	{
	}

};


