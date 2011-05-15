#pragma once

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {
	class ncAttr {
		// base class, to support different types
	public:
		ncAttr(ncGroup* parent);
		ncAttr(ncVar* parent);
		virtual ~ncAttr();
		std::string name;
		ncType *attType;
		size_t len;
	private:
		int _attid;
		ncGroup *_parentGroup;
		ncVar *_parentVar;
		friend class ncFile;
		friend class ncVar;
	protected:
		void *_data;
	};
};