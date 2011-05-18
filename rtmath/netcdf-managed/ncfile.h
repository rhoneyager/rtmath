#pragma once
// ncfile.h - The top-level structure of a netCDF version 4 file

#include "netcdf-managed.h"
#include <string>
#include <vector>
#include <map>
#include <set>

namespace netcdf_managed {

	struct fileOpt {
		fileOpt();
		bool writeable;
		bool clobber;
		bool offset64;
		bool netcdf4;
		bool classic_model;
		bool parallel;
	};

	class ncFile {
	public:
		ncFile();
		~ncFile();
		void load(const char* filename);
		void write();
		void create(const char* filename);
		void close();
		ncGroup* groupTop;
		//std::set<ncGroup*> groups;
		struct fileOpt options;
	private:
		void _iter_load(ncGroup *grp);
		bool _valid;
		bool _fileopen;
		int _fileid;
	};


};


