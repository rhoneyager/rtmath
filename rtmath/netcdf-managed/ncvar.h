#pragma once

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {
	enum endianness {
		NATIVE,
		LITTLE,
		BIG
	};

	class ncVar {
		// base class, overloaded by different types
		friend class ncAttr;
		friend class ncFile;
	public:
		ncVar(ncGroup *parent);
		virtual ~ncVar();
		std::string name;
		std::set<ncAttr*> attributes;
		std::set<ncDim*> dimensions;
		endianness varEndian;
		ncType *varType;
	protected:
		void *data;
	private:
		int _varid;
		ncGroup *_parent;
	};
};


