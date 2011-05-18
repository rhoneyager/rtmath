#include "ncfile.h"
#include <netcdf.h>
#include <set>
#include <vector>


namespace netcdf_managed {
	ncFile::ncFile()
	{
		// The class really isn't initialized until load, write or create is called
		// After all, this class is for manipulating netcdf files
		groupTop = NULL;
		_valid = false;
		_fileopen = false;
		_fileid = -1;
	}

	ncFile::~ncFile()
	{
		// Free stuff and close files
		if (_fileopen) close();
		if (groupTop) delete groupTop;
	}

	void ncFile::load(const char* filename)
	{
		if (_fileopen) throw;
		// Map the options to nc_open options
		/* omode A zero value (or NC NOWRITE) specifies the default behavior: open the
		dataset with read-only access, buffering and caching accesses for efficiency
		NC WRITE flag opens the dataset with read-write access.
		NC SHARE flag is only used for netCDF classic and 64-bit offset files. It
		is appropriate when one process may be writing the dataset and one or more
		other processes reading the dataset concurrently */
		// So, options are writable and parallel
		int res = 0;
		try {
			// This block does error testing. If throw is encountered, handle it
			int omode = 0;
			if (options.writeable) omode += NC_WRITE;
			if (options.parallel)  omode += NC_SHARE;
			res = nc_open(filename, omode, &_fileid);
			if (res) throw(res);

			// The file is open. Now, for a bunch of inquiries
			// Groups must be handled in version 4 of the library...
			// the ncid is the id of the file and the root group of the file

			// Create the root group programatically
			ncGroup *root = new ncGroup(NULL);
			root->_ncid = _fileid;

			/* Now, iterate through the group tree to 
			populate each group's name, childres, dimensions,
			attributes, variables and types */
			groupTop = root;
			_iter_load(root);
		}
		catch(int ncreturn)
		{
			// Error handleing - ncreturn is the netcdf error code
			// Get string with nc_strerror(ncreturn) and 
			// send to the predefined handler in the ui
		}

	}

	void ncFile::_iter_load(ncGroup *grp)
	{
		// TODO: do all the error checks
		int res = 0;
		// Get the group name - _not_ the full name
		char gname[NC_MAX_NAME];
		res = nc_inq_grpname(grp->_ncid, gname);
		if (res) throw(res);
		grp->name.assign(gname);

		// Load in the group's dimensions
		int numdims;
		nc_inq_dimids(grp->_ncid, &numdims, NULL, 0);
		// Get the ncids
		int *dimids = new int[numdims];
		// Get the unlimited dimension ids
		int numulimdims;
		nc_inq_unlimdims(grp->_ncid, &numulimdims,NULL);
		int *uldimids = new int[numulimdims];
		nc_inq_unlimdims(grp->_ncid, &numulimdims,uldimids);

		nc_inq_dimids(grp->_ncid, &numdims, dimids, 0);
		// Do a loop to load each dimension
		for (int i=0; i<numdims; i++) {
			ncDim *newDim = new ncDim(grp);
			newDim->_dimid = dimids[i];
			// length, name, unlim dim
			size_t dlen;
			char dimname[NC_MAX_NAME];
			nc_inq_dim(grp->_ncid, dimids[i], dimname, &dlen);
			newDim->name.assign(dimname);
			newDim->length = dlen;
			// Check id against list to see if it is unlimited
			for (int j=0;j<numulimdims;j++)
				if (uldimids[j] == dimids[i]) newDim->unlimdim = true;
			// Link dimension in set
			grp->dimensions.insert(newDim);
		}
		delete[] dimids;
		delete[] uldimids;

		// Load in the types
		// TODO: implement later
		// First, find out how to look up the right nc_type
		/*
		int numtypes;
		nc_inq_typeids(grp->_ncid,&numtypes,NULL);
		// This includes atomic types within the listing
		int *typeids = new int[numtypes];
		nc_inq_typeids(grp->_ncid,&numtypes,typeids);

		for (int i=0;i<numtypes;i++)
		{
		// Create and load each type
		// First, find out name and typeid
		nc_type idtype;
		char nametype[NC_MAX_NAME];
		nc_inq_type(grp->_ncid,idtype, );

		}


		delete[] typeids;
		*/

		// Load variables (and their attributes)
		int numvars;
		nc_inq_varids(grp->_ncid,&numvars,NULL);
		int *varids = new int[numvars];
		nc_inq_varids(grp->_ncid,&numvars,varids);

		for (int i=0;i<numvars;i++)
		{
			ncVar *newVar = new ncVar(grp);
			char varname[NC_MAX_NAME];
			int ndims,natts,endian;
			nc_type vartype;
			nc_inq_varname(grp->_ncid,varids[i],varname);
			nc_inq_varndims(grp->_ncid,varids[i],&ndims);
			nc_inq_varnatts(grp->_ncid,varids[i],&natts);
			nc_inq_vartype(grp->_ncid,varids[i],&vartype);
			newVar->name.assign(varname);
			nc_get_var(grp->_ncid,varids[i], newVar->data);
			nc_inq_var_endian(grp->_ncid, varids[i],&endian);
			switch (endian)
			{
			case NC_ENDIAN_NATIVE:
				newVar->varEndian = NATIVE;
				break;
			case NC_ENDIAN_LITTLE:
				newVar->varEndian = LITTLE;
				break;
			case NC_ENDIAN_BIG:
				newVar->varEndian = BIG;
				break;
			}

			// Get dimension ids and map to the actual dimensions
			int *dimids = new int[ndims];
			nc_inq_vardimid(grp->_ncid,varids[i],dimids);
			// Search through the group's dimension list
			// This list gives all inherited and group-specific dimensions
			// The appropriate ids may be located here
			for (int j=0;j<ndims;j++)
			{
				std::set<ncDim*>::const_iterator it;
				for (it=grp->dimensions.begin();it!=grp->dimensions.end(); it++)
				{
					if (dimids[j] == (*it)->_dimid) newVar->dimensions.push_back(*it);
				}
			}
			// The dimensions are now loaded in the appropriate order
			// Free the dim. id list
			delete[] dimids;

			// Load the attributes for the variable
			// Attribute names are the key here - NOT the id number
			// This is an annoying shift from previous parts. 
			// So, iterate and get the name for each id and add the entry
			for (int j=0;j<natts;j++)
			{
				ncAttr* newAttr = new ncAttr(newVar); // set variable as the parent
				char attname[NC_MAX_NAME];
				nc_inq_attname(grp->_ncid, varids[i], j, attname);
				newAttr->name.assign(attname);
				nc_inq_attlen(grp->_ncid, varids[i], attname, &newAttr->len);
				newAttr->_attid = j; // Not very important
				nc_type atttype;
				nc_inq_atttype(grp->_ncid, varids[i], attname, &atttype);
				// Get the data pointer
				// Be lazy and don't do the cast just yet
				nc_get_att(grp->_ncid, varids[i],attname, newAttr->_data);
				newVar->attributes.insert(newAttr);
			}
		}

		// Load global attributes
		// TODO: recombine this code with above into a separate class function!!!!!
		int ngatts;
		nc_inq_varnatts(grp->_ncid,NC_GLOBAL,&ngatts);
		for (int j=0;j<ngatts;j++)
		{
			ncAttr* newAttr = new ncAttr(grp); // set variable as the parent
			char attname[NC_MAX_NAME];
			nc_inq_attname(grp->_ncid, NC_GLOBAL, j, attname);
			newAttr->name.assign(attname);
			nc_inq_attlen(grp->_ncid, NC_GLOBAL, attname, &newAttr->len);
			newAttr->_attid = j; // Not very important
			nc_type atttype;
			nc_inq_atttype(grp->_ncid, NC_GLOBAL, attname, &atttype);
			// Get the data pointer
			// Be lazy and don't do the cast just yet
			nc_get_att(grp->_ncid, NC_GLOBAL,attname, newAttr->_data);
			grp->attributes.insert(newAttr);
		}

		// Load in new groups and prepolulate the mappings with what the
		// parent knows (dim ids, types, ...)
		// nc_inq_grps
		int numgrps;
		nc_inq_grps(grp->_ncid,&numgrps,NULL);
		int *grpids = new int[numgrps];
		nc_inq_grps(grp->_ncid,&numgrps,grpids);
		// Loop through to construct each group
		// Give each group the dimensions and types of the parent
		// * These will just be relinked in - _parent is unchanged!
		for (int j=0;j<numgrps;j++)
		{
			ncGroup *child = new ncGroup(grp);
			std::set<ncDim*>::const_iterator dit;
			std::set<ncType*>::const_iterator tit;
			for (dit = grp->dimensions.begin(); dit != grp->dimensions.end(); dit++)
			{
				child->dimensions.insert( *dit );
			}
			for (tit = grp->types.begin(); tit != grp->types.end(); tit++)
			{
				child->types.insert( *tit );
			}
			// Recurse with this function to load the child!
			_iter_load(child);
		}
	}

	void ncFile::write()
	{
		if (!_valid) throw;
		if (!_fileopen) throw;
		// Assume file is already open, via create or load
		if (!options.writeable) throw;
		// See if nc_redef needs calling
		// nc_enddef

		// end with nc_sync to sync to disk
	}

	void ncFile::create(const char* filename)
	{
		/* cmode The creation mode flag. The following flags 
		are available: NC NOCLOBBER, NC SHARE, NC 64BIT OFFSET, 
		NC NETCDF4, NC CLASSIC MODEL. */
	}

	void ncFile::close()
	{
	}

	fileOpt::fileOpt()
	{
		writeable = false;
		clobber = false;
		offset64 = false;
		netcdf4 = true;
		classic_model = false;
		parallel = false;
	}
}; // end namespace

