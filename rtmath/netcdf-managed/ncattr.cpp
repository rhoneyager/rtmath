#include "ncattr.h"

namespace netcdf_managed {
	ncAttr::ncAttr(ncGroup *parent)
	{
		_parentGroup = parent;
		_parentVar = NULL;
		_data = NULL;
		len = 0;
		attType = NULL;
	}

	ncAttr::ncAttr(ncVar *parent)
	{
		_parentVar = parent;
		_parentGroup = parent->_parent;
		_data = NULL;
		len = 0;
		attType = NULL;
	}

	ncAttr::~ncAttr()
	{
	}

};

