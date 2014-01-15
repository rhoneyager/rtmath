#pragma once
#include <string>
#include <vector>

#include "MaterialList.h"
#include "QuadMesh3d.h"
struct DBfile;

namespace rtmath {
	namespace plugins {
		namespace silo {

			/**
			*   Represents a 3D curvilinear mesh
			*
			* Notes:      
			*
			* Programmer: Brad Whitlock
			* Creation:   Fri Jun 21 13:55:27 PST 2002
			*
			* Modifications:
			*   Brad Whitlock, Fri Mar 14 09:33:32 PDT 2003
			*   I added the WriteMeshSlice method.
			*
			**/
			class CurveMesh3D : public QuadMesh3D
			{
			public:
				CurveMesh3D(int X, int Y, int Z);
				virtual ~CurveMesh3D();
				virtual float GetX(int x, int y, int z) const;
				virtual float GetY(int x, int y, int z) const;
				virtual float GetZ(int x, int y, int z) const;
				void SetX(int x, int y, int z, float val);
				void SetY(int x, int y, int z, float val);
				void SetZ(int x, int y, int z, float val);
			private:
				virtual void WriteMesh(DBfile *db, const char *name);
				virtual void WriteMeshSlice(DBfile *db, const std::string &newMeshName,
					int sliceVal, int sliceDimension);
			};


		}
	}
}

