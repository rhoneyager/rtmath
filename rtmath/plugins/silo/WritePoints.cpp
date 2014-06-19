#include "WritePoints.h"

#define DB_USE_MODERN_DTPTR
#include <silo.h>

/// Namespace defining function mappings
namespace {
	template<class T>
	DBdatatype getSiloDatatype() { throw; }

	template<> DBdatatype getSiloDatatype<float>() { return DB_FLOAT; }
	template<> DBdatatype getSiloDatatype<double>() { return DB_DOUBLE; }
	template<> DBdatatype getSiloDatatype<int>() { return DB_INT; }
	template<> DBdatatype getSiloDatatype<char>() { return DB_CHAR; }
	template<> DBdatatype getSiloDatatype<long>() { return DB_LONG; }
	template<> DBdatatype getSiloDatatype<long long>() { return DB_LONG_LONG; }
}

namespace rtmath {
	namespace plugins {
		namespace silo {

			void WritePoints(DBfile *db, const char* name,
				const std::array<std::string, 3> &axislabels,
				const std::array<std::string, 3> &axisunits, 
				const Eigen::Matrix<float, Eigen::Dynamic, 3> &pts,
				std::vector<std::tuple<std::string, std::string, 
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > &vals)
			{
				// Write the point mesh
				DBoptlist *optlist = DBMakeOptlist(6);
				DBAddOption(optlist, DBOPT_XLABEL, (void*)axislabels[0].c_str());
				DBAddOption(optlist, DBOPT_YLABEL, (void*)axislabels[1].c_str());
				DBAddOption(optlist, DBOPT_ZLABEL, (void*)axislabels[2].c_str());
				DBAddOption(optlist, DBOPT_XUNITS, (void *)axisunits[0].c_str());
				DBAddOption(optlist, DBOPT_YUNITS, (void *)axisunits[0].c_str());
				DBAddOption(optlist, DBOPT_ZUNITS, (void *)axisunits[0].c_str());
				const int numPoints = (int)pts.rows();

				// Convert the points, by coordinate, into arrays
				const float *pcoords[3] = { pts.col(0).data(), 
					pts.col(1).data(), pts.col(2).data() };
				DBPutPointmesh(db, name, 3, pcoords, numPoints, DB_FLOAT, optlist);
				DBFreeOptlist(optlist);

				for (const auto &a : vals)
				{
					// Write the point var
					optlist = DBMakeOptlist(1);
					DBAddOption(optlist, DBOPT_UNITS, (void *)std::get<1>(a).c_str());

					const float **vals = new const float*[std::get<2>(a).cols()];
					for (size_t i = 0; i < (size_t) std::get<2>(a).cols(); ++i)
						vals[i] = std::get<2>(a).col(i).data();
					//float *vals[1] = { a.second.col(0).data() };
					DBPutPointvar(db, std::get<0>(a).c_str(), name, 
						(int) std::get<2>(a).cols(), 
						vals, 
						numPoints, 
						DB_FLOAT, 
						optlist);
					DBFreeOptlist(optlist);
					delete[] vals;
				}
			}



			siloFile::siloFile(const char* filename, const char* desc)
			{
				df = DBCreate(filename, DB_CLOBBER, DB_LOCAL,
					desc, // Optional string describing file
					//DB_HDF5
					DB_PDB
					);
				assert(df);
			}

			siloFile::~siloFile()
			{
				DBClose(df);
			}


			template <class T>
			std::shared_ptr<siloFile::pointMesh<T> > siloFile::pointMesh<T>::createMesh(
				std::shared_ptr<siloFile> parent,
				const char* name, 
				size_t ndims, size_t nCrds, const T* crdarray[],
				const char *dimLabels[], const char *dimUnits[])
			{
				std::shared_ptr<pointMesh<T> > res(new pointMesh<T>(parent));
				res->name = std::string(name);
				res->numPoints = nCrds;

				DBdatatype datatype = getSiloDatatype<T>();

				DBoptlist *optlist = DBMakeOptlist((int)ndims*2);
				if (dimLabels && dimUnits)
					for (size_t i=0; i<ndims && i < 3; ++i)
					{
						const char *dimLabel = dimLabels[i];
						const char *dimUnit = dimUnits[i];
						res->dimLabels.push_back(std::string(dimLabel));
						res->dimUnits.push_back(std::string(dimUnit));
						int labelVal = DBOPT_XLABEL;
						if (i==1) labelVal = DBOPT_YLABEL;
						if (i==2) labelVal = DBOPT_ZLABEL;
						int unitVal = DBOPT_XUNITS;
						if (i==1) unitVal = DBOPT_YUNITS;
						if (i==2) unitVal = DBOPT_ZUNITS;

						DBAddOption(optlist, labelVal, (void*)dimLabel);
						DBAddOption(optlist, unitVal, (void*)dimUnit);
					}

					//if (type == meshType::MESH_POINTS)
					DBPutPointmesh(parent->df, name, (int) ndims, crdarray, (int) nCrds, datatype, optlist);
					/*
					else if (type == meshType::MESH_CURVILINEAR)
					DBPutQuadmesh(df, name, NULL, crdarray, dims, 
					ndims, datatype, DB_COLLINEAR, optlist);
					*/
					DBFreeOptlist(optlist);

					return res;
			}

			template <class T>
			std::shared_ptr<siloFile::rectilinearMesh<T> > siloFile::rectilinearMesh<T>::createMesh(
				std::shared_ptr<siloFile> parent,
				const char* name, 
				size_t ndims, T* crdarray[], const int dimsizes[],
				const char *dimLabels[], const char *dimUnits[])
			{
				std::shared_ptr<rectilinearMesh<T> > res(new rectilinearMesh<T>(parent));
				res->name = std::string(name);
				res->dimsizes.resize(ndims);
				res->dimsizes.assign(dimsizes, dimsizes + ndims);
				res->numPoints = 1;
				for (const auto &it : res->dimsizes)
					res->numPoints *= it;
				//res->crdarray = crdarray;

				DBdatatype datatype = getSiloDatatype<T>();

				DBoptlist *optlist = DBMakeOptlist(3+(int)ndims*2);
				if (dimLabels && dimUnits)
					for (size_t i=0; i<ndims && i < 3; ++i)
					{
						const char *dimLabel = dimLabels[i];
						const char *dimUnit = dimUnits[i];
						res->dimLabels.push_back(std::string(dimLabel));
						res->dimUnits.push_back(std::string(dimUnit));
						int labelVal = DBOPT_XLABEL;
						if (i==1) labelVal = DBOPT_YLABEL;
						if (i==2) labelVal = DBOPT_ZLABEL;
						int unitVal = DBOPT_XUNITS;
						if (i==1) unitVal = DBOPT_YUNITS;
						if (i==2) unitVal = DBOPT_ZUNITS;

						DBAddOption(optlist, labelVal, (void*)dimLabel);
						DBAddOption(optlist, unitVal, (void*)dimUnit);
					}

					//DBAddOption(optlist, DBOPT_MAJORORDER, (void*)1); // Eigen matrices are in column-major ordering!

					DBPutQuadmesh(parent->df, // dbfile
						name, // mesh name
						NULL, // coordinate names (ignored, set to NULL)
						crdarray, // array of length ndims containing pointers to the coord arrays
						res->dimsizes.data(), // array of length ndims describing mesh dimensionality
						(int) ndims, // number of dimensions
						datatype, // datatype of the coord arrays
						DB_COLLINEAR, // rectilinear mesh
						optlist // option list
						);
					DBFreeOptlist(optlist);

					return res;
			}


			template <class U>
			void writeQuadData(const char* varname,
				const char* name, DBfile *df, 
				const U** data, size_t nDims, 
				const std::vector<int> &dimsizes,
				const char* varunits,
				const char** varnames
				)
			{
				DBdatatype datatype = getSiloDatatype<U>();

				DBoptlist *optlist = DBMakeOptlist(1);
				if (varunits)
					DBAddOption(optlist, DBOPT_UNITS, (void *)varunits);
				//DBAddOption(optlist, DBOPT_MAJORORDER, (void *)1); // Causes access violation

				char ** vn = const_cast<char**> (varnames);
				const char* noname = varname;
				std::vector<const char*> novarnames(nDims, noname);
				if (!vn)
					vn = const_cast<char**> (novarnames.data());

				std::vector<int> dimsizes2 = dimsizes;

				DBPutQuadvar(df, varname, name,
					(int) nDims,
					vn,
					(void*) data,
					//crdarray,
					dimsizes2.data(),
					(int) dimsizes2.size(),
					NULL,
					0,
					datatype,
					//DB_ZONECENT,
					DB_NODECENT,
					optlist
					);
				DBFreeOptlist(optlist);

			}

			
			template <class U>
			void writePointData(const char* varname, const char* name, DBfile *df, 
				size_t numPoints,
				const U** data, size_t nDims, const char* varUnits)
			{
				DBdatatype datatype = getSiloDatatype<U>();

				DBoptlist *optlist = DBMakeOptlist(1);
				if (varUnits)
					DBAddOption(optlist, DBOPT_UNITS, (void *)varUnits);
				DBPutPointvar(df, varname, name,
					(int) nDims,
					(void*) data,
					(int) numPoints,
					datatype,
					optlist);
				DBFreeOptlist(optlist);
			}


#define INST_MESHES(x) \
	template class siloFile::mesh<x>; \
	template class siloFile::pointMesh<x>; \
	template class siloFile::rectilinearMesh<x>; \
	template void writePointData<x>(const char*, const char*, DBfile*, \
		size_t, const x**, size_t, const char*); \
	template void writeQuadData<x>(const char*, const char*, DBfile*, \
		const x**, size_t, const std::vector<int>&, const char*, const char**);

			INST_MESHES(float);
			INST_MESHES(double);
			INST_MESHES(int);
			INST_MESHES(char);
			INST_MESHES(long long);
			INST_MESHES(long);

		}
	}
}

