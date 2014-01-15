#pragma once
#include <string>
#include <vector>

#include "MaterialList.h"
#include "QuadMesh3d.h"
struct DBfile;

namespace rtmath {
	namespace plugins {
		namespace silo {

			class RectilinearMesh3D : public QuadMesh3D
			{
			public:
				RectilinearMesh3D(int nx, int ny, int nz);
				virtual ~RectilinearMesh3D();
				virtual float GetX(int x, int, int) const;
				virtual float GetY(int, int y, int) const;
				virtual float GetZ(int, int, int z) const;
				void SetX(int x, float val);
				void SetY(int y, float val);
				void SetZ(int z, float val);
				void SetXValues(float minX, float maxX);
				void SetYValues(float minY, float maxY);
				void SetZValues(float minZ, float maxZ);
			protected:
				virtual void WriteMesh(DBfile *db);
				virtual void WriteMeshSlice(DBfile *db, const std::string &newMeshName,
					int sliceVal, int sliceDimension);
				void SetRange(float *coord, float minval, float maxval, int steps);
			};


		}
	}
}

