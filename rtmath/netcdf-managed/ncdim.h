#pragma once

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {
	class ncDim {
	public:
		ncDim(ncGroup* parent);
		~ncDim();
		std::string name;
		int length;
		bool unlimdim;
	private:
		int _dimid;
		ncGroup *_parent;
		friend class ncFile;
	};
};


