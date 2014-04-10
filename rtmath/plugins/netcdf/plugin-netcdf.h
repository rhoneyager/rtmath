#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

#include <netcdf.h>


#define PLUGINID "B30B35AF-8F61-4DB2-B42D-BADA08928717"

//typedef int nc_type;

namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		/** \brief Namespace for the netCDF file readers.
		*
		* This plugin will probably only be used to read netcdf files. I already have HDF5 
		* write support and thus have no reason to write netcdf files. The plugin will 
		* provide reading abilities for the rtmath_data library for ARM program data 
		* for various operational products.
		**/
		namespace netcdf {
			//class netcdfFile;


			struct netcdf_handle : public rtmath::registry::IOhandler
			{
				netcdf_handle(const char* filename, IOtype t);
				virtual ~netcdf_handle();
				void open(const char* filename, IOtype t);
				int file;
				bool headerOpen;
				bool readable;
				bool writeable;
				static void handle_error(int status);
				//std::shared_ptr<netcdfFile> file;
				void openHeader();
				void closeHeader();
			};


			template <class DataType>
			bool AttrMatches(nc_type) { return false; }
			template<> bool AttrMatches<double>(nc_type t);
			template<> bool AttrMatches<float>(nc_type t);
			template<> bool AttrMatches<int>(nc_type t);
			template<> bool AttrMatches<short>(nc_type t);


			std::pair<bool, int> AttrExists(int ncid, int varid, const char* varname);

			std::pair<bool, int> VarExists(int ncid, const char* varname);

			template <class DataType>
			int getNCvar(int ncid, int varid, DataType *) { RTthrow debug::xUnimplementedFunction(); }

			template<> int getNCvar<double>(int ncid, int varid, double* res);
			template<> int getNCvar<float>(int ncid, int varid, float* res);
			template<> int getNCvar<int>(int ncid, int varid, int* res);
			template<> int getNCvar<short>(int ncid, int varid, short* res);
			//template<> int getNCvar<std::string>(int ncid, int varid, std::string *res);

			template <class DataType>
			Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>
				getMatrix(const char* name, std::shared_ptr<netcdf_handle> h)
			{
					int status = 0;
					int parentId = h->file;
					int varid = 0;
					if (!VarExists(parentId, name).first) RTthrow debug::xArrayOutOfBounds();
					status = nc_inq_varid(parentId, name, &varid);
					if (status) h->handle_error(status);
					nc_type vartype;
					status = nc_inq_vartype(parentId, varid, &vartype);
					if (status) h->handle_error(status);
					if (!AttrMatches<typename DataType>(vartype)) RTthrow debug::xBadInput(name);

					int ndims = 0;
					status = nc_inq_varndims(parentId, varid, &ndims);
					if (status) h->handle_error(status);
					if (ndims < 0 || ndims > 2) RTthrow debug::xArrayOutOfBounds();

					int dimids[2] = { 0, 0 };
					status = nc_inq_vardimid(parentId, varid, dimids);

					// Get dimension lengths
					size_t dimLens[2] = { 1, 1 };
					for (int i = 0; i < ndims; ++i)
					{
						status = nc_inq_dimlen(parentId, dimids[i], &(dimLens[i]));
						if (status) h->handle_error(status);
					}

					// Define the matrix
					/// \note Variable is read with last dimension varying fastest. No idea how this converts to Eigen. Will 
					/// determine when reading actual data.
					Eigen::Matrix<typename DataType, Eigen::Dynamic, Eigen::Dynamic> res(dimLens[0], dimLens[1]);
					status = getNCvar<typename DataType>(parentId, varid, res.data());
					if (status) h->handle_error(status);
					return res;
			}
		}
	}
}

