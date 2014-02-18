#include "../../rtmath/rtmath/defs.h"
#include <array>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <Eigen/Dense>

struct DBfile;

namespace rtmath {
	namespace ddscat {
		namespace shapefile {
			class shapefile;
		}
	}
	namespace plugins {
		namespace silo {
			struct silo_handle;

			void writeShape(DBfile *f, const char* meshname, 
				const rtmath::ddscat::shapefile::shapefile *shp);

			/// \brief Old method of writing points to a silo file
			/// \deprecated
			void DEPRECATED WritePoints(DBfile *db, 
				const char* meshname,
				const std::array<std::string, 3> &axislabels,
				const std::array<std::string, 3> &axisunits, 
				const Eigen::Matrix<float, Eigen::Dynamic, 3> &pts,
				std::vector<std::tuple<std::string, std::string, 
				const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > &vals);

			
			template <class U>
			void writePointData(const char* varname, const char* name, DBfile *df, 
				size_t numPoints,
				const U** data, size_t nDims, const char* varUnits);

			template <class U>
			void writeQuadData(const char* varname,
				const char* name, DBfile *df, 
				const U** data, size_t nDims, 
				const std::vector<int> &dimsizes,
				const char* varunits,
				const char** varnames
				);

			/**
			* \brief Provides the ability to write various types of meshes and 
			* data to silo files.
			*
			* This class opens a silo file for writing, and closes the file 
			* upon destruction. Using the class members, various types of 
			* meshes may be constructed. Each mesh supports various writing 
			* methods for data storage.
			*
			* \todo Enable re-finding of meshes.
			**/
			class siloFile : public std::enable_shared_from_this<siloFile>
			{
			protected:
				friend struct silo_handle;
				class meshBase : protected std::enable_shared_from_this<meshBase> {protected: meshBase() {} virtual ~meshBase() {}};
				siloFile(const char* filename, const char* desc = nullptr);
			private:
				
				friend class meshBase;
				//std::vector<std::weak_ptr<meshBase> > meshes;
			public:
				/// \todo Make private after silo-ddOutput is ported.
				DBfile *df;
				static std::shared_ptr<siloFile> generate
					(const char* filename, const char* desc = nullptr)
				{
					return std::shared_ptr<siloFile>(new siloFile(filename, desc));
				}
				~siloFile();

				enum class meshType
				{
					MESH_RECTILINEAR,
					MESH_POINTS
				};

				template <class T>
				class mesh : public meshBase
				{
				protected:
					mesh(meshType type, std::shared_ptr<siloFile> parent) 
						: type(type), parent(parent), numPoints(0) {}
					meshType type;
					std::string name;
					std::vector<std::string> dimLabels;
					std::vector<std::string> dimUnits;
					std::shared_ptr<siloFile> parent;
					size_t numPoints;
				};
				template <class T>
				class pointMesh : public mesh<T>
				{
					friend class siloFile;
				protected:
					pointMesh(std::shared_ptr<siloFile> parent) 
						: mesh(meshType::MESH_POINTS, parent) {}
				public:
					virtual ~pointMesh() {}
					static std::shared_ptr<pointMesh<T> > createMesh(
						std::shared_ptr<siloFile> parent,
						const char* name, size_t ndims, size_t nCrds,
						const T* crdarray[],
						const char *dimLabels[] = nullptr,
						const char *dimUnits[] = nullptr);
					static std::shared_ptr<pointMesh<T> > createMesh(
						std::shared_ptr<siloFile> parent,
						const char* name,
						const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &crds,
						const char *dimLabels[] = nullptr, const char *dimUnits[] = nullptr
						)
					{
						std::vector<const T*> crdarray(crds.cols());
						//std::unique_ptr<T[]> crdarray(new T[crds.rows()]);
						for (size_t i=0; i<(size_t)crds.cols(); ++i)
							crdarray[i] = crds.col(i).data();
						return createMesh(parent, name, crds.cols(), crds.rows(), 
							crdarray.data(), dimLabels, dimUnits);
					}

					template <class U>
					void writeData(const char* varname,
						const Eigen::Matrix<U, Eigen::Dynamic, Eigen::Dynamic> &vals,
						const char* varUnits = nullptr)
					{
						std::vector<const U*> data(vals.cols());
						for (size_t i = 0; i<(size_t)vals.cols(); ++i)
							data[i] = vals.col(i).data();
						writeData(varname, data.data(), (size_t)vals.cols(), varUnits);
					}

					template <class U>
					void writeData(const char* varname,
						const U** data, size_t nDims, const char* varUnits = nullptr)
					{
						writePointData<U>(varname, this->name.c_str(), this->parent->df,
							this->numPoints, data, nDims, varUnits);
					}

					template <class U>
					void writeData(const char* varname,
						const U* data, const char* varUnits = nullptr)
					{
						const U * cdata[] = { data };
						writeData<U>(varname, cdata, 1, varUnits);
					}
				};

				// Delegates for pointMesh
				template<class T>
				std::shared_ptr<pointMesh<T> > createPointMesh(
					const char* name, size_t ndims, size_t nCrds,
					const T* crdarray[],
					const char *dimLabels[] = nullptr,
					const char *dimUnits[] = nullptr)
				{
					return pointMesh<T>::createMesh(shared_from_this(),
						name, ndims, nCrds, crdarray, dimLabels, dimUnits);
				}
				template<class T>
				std::shared_ptr<pointMesh<T> > createPointMesh(
					const char* name,
					const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &crds,
					const char *dimLabels[] = nullptr, const char *dimUnits[] = nullptr)
				{
					return pointMesh<T>::createMesh(shared_from_this(),
						name, crds, dimLabels, dimUnits);
				}


				template <class T>
				class rectilinearMesh : public mesh<T>
				{
					friend class siloFile;
				protected:
					rectilinearMesh(std::shared_ptr<siloFile> parent) 
						: mesh(meshType::MESH_RECTILINEAR, parent) {}
					std::vector<int> dimsizes;

					//const T* crdarray[];
				public:
					virtual ~rectilinearMesh() {}
					static std::shared_ptr<rectilinearMesh<T> > createMesh(
						std::shared_ptr<siloFile> parent,
						const char* name, size_t ndims, 
						T* crdarray[],
						const int dimsizes[],
						const char *dimLabels[] = nullptr,
						const char *dimUnits[] = nullptr);

					/// Used for grids without coordinates (auto-generate indices)
					static std::shared_ptr<rectilinearMesh<T> > createMesh(
						std::shared_ptr<siloFile> parent,
						const char* name, size_t ndims, 
						const int dimsizes[],
						const char *dimLabels[] = nullptr,
						const char *dimUnits[] = nullptr)
					{
						T** crdarray = new T*[ndims];
						//std::vector<T*> crdarray(ndims);
						for (size_t i=0; i<ndims; ++i)
						{
							crdarray[i] = new T[dimsizes[i]];
							for (size_t j=0; j<dimsizes[i]; ++j)
								crdarray[i][j] = (T) j;
						}
						auto mesh = createMesh(parent, name, ndims, crdarray,
							dimsizes, dimLabels, dimUnits);

						for (size_t i=0; i<ndims; ++i)
						{
							delete[] crdarray[i];
						}
						delete[] crdarray;
						return mesh;
					}

					template <class U>
					void writeData(const char* varname,
						const U** data, size_t nDims, 
						const char *varunits = nullptr,
						const char ** varNames = nullptr
						)
					{
						writeQuadData(varname, this->name.c_str(), this->parent->df,
							data, nDims, this->dimsizes, varUnits, varnames);
					}

					template<class U>
					void writeData(const char* varname,
						const U* data, 
						const char *varunits = nullptr,
						const char ** varNames = nullptr
						)
					{
						const U *cdata[] = { data };
						writeData<U>(varname, cdata, 1, varunits, varNames);
					}
				};

				// Delegates for rectilinearMesh
				template<class T>
				std::shared_ptr<rectilinearMesh<T> > createRectilinearMesh(
					const char* name, size_t ndims, const T* crdarray[],
					const int dimsizes[],
					const char *dimLabels[] = nullptr,
					const char *dimUnits[] = nullptr)
				{
					return rectilinearMesh<T>::createMesh(shared_from_this(),
						name, ndims, crdarray, dimsizes, dimLabels, dimUnits);
				}

				template<class T>
				std::shared_ptr<rectilinearMesh<T> > createRectilinearMesh(
					const char* name, size_t ndims, 
					const int dimsizes[],
					const char *dimLabels[] = nullptr,
					const char *dimUnits[] = nullptr)
				{
					return rectilinearMesh<T>::createMesh(shared_from_this(),
						name, ndims, dimsizes, dimLabels, dimUnits);
				}

			};
		}
	}
}
