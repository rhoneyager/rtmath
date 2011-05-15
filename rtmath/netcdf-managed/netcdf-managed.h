#pragma once
/* The general catch-all header for including netcdf-managed into a project */

#include <netcdf.h>

// Test to see if version 4 is enabled. If not, disable those features.
//TODO

namespace netcdf_managed {
	class ncFile;
	class ncGroup;
	class ncDim;
	class ncAttr;
	class ncVar;
	class ncType;
};

#include "ncfile.h"
#include "ncgroup.h"
#include "ncdim.h"
#include "ncattr.h"
#include "ncvar.h"
#include "nctype.h"
