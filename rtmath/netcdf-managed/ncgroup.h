#pragma once

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {

	class ncGroup {
	public:
		ncGroup(ncGroup* parent);
		~ncGroup();
		std::string name;
		std::set<ncGroup*> children;
		std::set<ncDim*> dimensions;
		std::set<ncAttr*> attributes;
		std::set<ncVar*> variables;
		std::set<ncType*> types;
	private:
		int _ncid; // Used only when writing / reading
		ncGroup *_parent; // for structure consistency
		friend class ncFile;
	};
};


