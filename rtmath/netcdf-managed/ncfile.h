#pragma once
// ncfile.h - The top-level structure of a netCDF version 4 file

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {

	class ncFile {
	public:
		ncFile();
		~ncFile();
		void load(const char* filename);
		void write(const char* filename);
		void create(const char* filename);
		void close();
		ncGroup* groupTop;
		//std::set<ncGroup*> groups;
		struct options {
			options();
			bool writeable;
			bool noclobber;
			bool offset64;
			bool netcdf4;
			bool classic_model;
		};
	private:
		bool _valid;
		bool _fileopen;
	};


};


