#include "ncgroup.h"

namespace netcdf_managed {
	ncGroup::ncGroup(ncGroup* parent)
	{
		_parent = parent;
		_ncid = -1;
	}

	ncGroup::~ncGroup()
	{
	}

};