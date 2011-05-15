#pragma once

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {
	class ncType {
		// This is a base class
	public:
		ncType(ncGroup* parent);
		virtual ~ncType();
		std::string name;
		size_t typesize;
		int nctypeclass; // Type class - ensures correct upward casting
		bool basetype; // Careful here - find a better way
		void *castfunction; // Pointer to casting function
	private:
		ncGroup *_parent;
		int _typeid;
		friend class ncFile;
	};

	class ncTypeCompound : public ncType
	{
	};

	class ncTypeOpaque : public ncType
	{
	};

	class ncTypeEnum : public ncType
	{
	};

	class ncTypeVlen : public ncType
	{
	};

};


