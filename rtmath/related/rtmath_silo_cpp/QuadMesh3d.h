#pragma once
#include <string>
#include <vector>

#include "MaterialList.h"
struct DBfile;

namespace rtmath {
	namespace plugins {
		namespace silo {
			/**
			*   Abstract base class for 3D quad meshes.
			*
			* Notes:
			*
			* Programmer: Brad Whitlock
			* Creation:   Fri Jun 21 13:53:59 PST 2002
			*
			* Modifications:
			*   Brad Whitlock, Fri Oct 4 14:25:40 PST 2002
			*   I added methods to create the gradient of a scalar field.
			*
			*   Eric Brugger, Tue Mar 25 15:56:59 PST 2003
			*   I corrected an out of range index error in ZonalGradientAt.
			*
			*   Hank Childs, Mon Dec  1 09:24:23 PST 2003
			*   Added support for tensors.
			*
			*   Hank Childs, Thu Jul 21 16:49:02 PDT 2005
			*   Added support for array variables.
			*
			**/
			class QuadMesh3D
			{
			protected:
				class TensorData;
				class VectorData
				{
				public:
					VectorData(const std::string &n, int nx, int ny, int nz, bool node);
					~VectorData();
					void SetZonalValue(int x, int y, int z, float val[3]);
					void SetNodalValue(int x, int y, int z, float val[3]);
					void WriteFile(DBfile *db, const char *meshName);
				protected:
					int xdim, ydim, zdim;
					bool nodal;
					float *xd, *yd, *zd;
					std::string name;
				};

				class ScalarData
				{
				public:
					ScalarData(const std::string &n, int nx, int ny, int nz, bool node);
					~ScalarData();
					inline const std::string &GetName() const { return name; };
					int ZonalIndex(int x, int y, int z) const;
					int NodalIndex(int x, int y, int z) const;
					void SetZonalValue(int x, int y, int z, float val);
					void SetNodalValue(int x, int y, int z, float val);
					void WriteFile(DBfile *db, const char *meshName);
					void WriteDataSlice(DBfile *db, const std::string &newMeshName,
						const std::string &newVarName, int sliceVal, int sliceDimension);
					void ZonalGradientAt(int i, int j, int k, float grad[3]) const;
					void NodalGradientAt(int i, int j, int k, float grad[3]) const;
					void ZonalTensorGradientAt(int i, int j, int k, float grad[9]) const;
					void NodalTensorGradientAt(int i, int j, int k, float grad[9]) const;
					VectorData *CreateGradient(const char *name);
					TensorData *CreateGradientTensor(const char *name);
				protected:
					int xdim, ydim, zdim;
					bool nodal;
					float *data;
					std::string name;
				};

				class TensorData {
				public:
					TensorData(const std::string &n, ScalarData *comps[9]);
					~TensorData();
					void WriteFile(DBfile *db, const char *meshName);
				protected:
					ScalarData *components[9];
					std::string name;
				};

				class SliceInfo
				{
				public:
					SliceInfo(const std::string &nm, const std::string &nvn,
						int sv, int sd, ScalarData *d);
					~SliceInfo();
					void WriteFile(DBfile *db, QuadMesh3D *qm);
				protected:
					std::string newMeshName;
					std::string newVarName;
					int sliceVal;
					int sliceDimension;
					ScalarData *scalars;
				};

				typedef std::vector<ScalarData *> ScalarDataVector;
				typedef std::vector<VectorData *> VectorDataVector;
				typedef std::vector<TensorData *> TensorDataVector;
				typedef std::vector<SliceInfo *>  SliceInfoVector;

				friend class SliceInfo;
			public:
				QuadMesh3D(int nx, int ny, int nz, bool rect = true);
				virtual ~QuadMesh3D();
				void SetMeshName(const std::string &name);
				int XDim() const { return xdim; };
				int YDim() const { return ydim; };
				int ZDim() const { return zdim; };

				virtual float GetX(int, int, int) const = 0;
				virtual float GetY(int, int, int) const = 0;
				virtual float GetZ(int, int, int) const = 0;

				void CreateZonalData(const char *name, float(*zonal)(int, int, int, QuadMesh3D *));
				void CreateNodalData(const char *name, float(*nodal)(float *, QuadMesh3D *));
				void CreateNodalVectorData(const char *name, void(*nodal)(float *, int, int, int, QuadMesh3D *));
				void CreateGradient(const char *name, const char *gradName);
				void CreateGradientTensor(const char *varName, const char *outputName);
				void AddMaterial(const char *matname);
				void CreateMaterialData(void(*createmat)(int, int, int, int *, double*, int *, QuadMesh3D *));
				void AddSlice(const std::string &varName, const std::string &nmn, const std::string &nvn,
					int sliceVal, int sliceDimension);
				void WriteFile(DBfile *db);
			protected:
				virtual void WriteMesh(DBfile *db) = 0;
				virtual void WriteMeshSlice(DBfile *db, const std::string &, int, int) = 0;

				int xdim;
				int ydim;
				int zdim;
				float *coordX;
				float *coordY;
				float *coordZ;
				ScalarDataVector scalarData;
				VectorDataVector vectorData;
				TensorDataVector tensorData;
				SliceInfoVector  sliceInfo;
				MaterialList     mats;
				bool             writeMaterial;
				std::string      meshName;
			};

		}
	}
}
