// File based on noise.C, an example in VisIt.
/*****************************************************************************
*
* Copyright (c) 2000 - 2008, Lawrence Livermore National Security, LLC
* Produced at the Lawrence Livermore National Laboratory
* LLNL-CODE-400142
* All rights reserved.
*
* This file is  part of VisIt. For  details, see https://visit.llnl.gov/.  The
* full copyright notice is contained in the file COPYRIGHT located at the root
* of the VisIt distribution or at http://www.llnl.gov/visit/copyright.html.
*
* Redistribution  and  use  in  source  and  binary  forms,  with  or  without
* modification, are permitted provided that the following conditions are met:
*
*  - Redistributions of  source code must  retain the above  copyright notice,
*    this list of conditions and the disclaimer below.
*  - Redistributions in binary form must reproduce the above copyright notice,
*    this  list of  conditions  and  the  disclaimer (as noted below)  in  the
*    documentation and/or other materials provided with the distribution.
*  - Neither the name of  the LLNS/LLNL nor the names of  its contributors may
*    be used to endorse or promote products derived from this software without
*    specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR  IMPLIED WARRANTIES, INCLUDING,  BUT NOT  LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND  FITNESS FOR A PARTICULAR  PURPOSE
* ARE  DISCLAIMED. IN  NO EVENT  SHALL LAWRENCE  LIVERMORE NATIONAL  SECURITY,
* LLC, THE  U.S.  DEPARTMENT OF  ENERGY  OR  CONTRIBUTORS BE  LIABLE  FOR  ANY
* DIRECT,  INDIRECT,   INCIDENTAL,   SPECIAL,   EXEMPLARY,  OR   CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT  LIMITED TO, PROCUREMENT OF  SUBSTITUTE GOODS OR
* SERVICES; LOSS OF  USE, DATA, OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER
* CAUSED  AND  ON  ANY  THEORY  OF  LIABILITY,  WHETHER  IN  CONTRACT,  STRICT
* LIABILITY, OR TORT  (INCLUDING NEGLIGENCE OR OTHERWISE)  ARISING IN ANY  WAY
* OUT OF THE  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
* DAMAGE.
*
*****************************************************************************/

#include <iostream>
#include <stdio.h>
#include "RectilinearMesh3d.h"

#define DB_USE_MODERN_DTPTR
#include <silo.h>

namespace rtmath {
	namespace plugins {
		namespace silo {

			RectilinearMesh3D::RectilinearMesh3D(int nx, int ny, int nz) : QuadMesh3D(nx, ny, nz, true)
			{
			}

			RectilinearMesh3D::~RectilinearMesh3D()
			{
			}

			float RectilinearMesh3D::GetX(int x, int, int) const
			{
				return coordX[x];
			}

			float RectilinearMesh3D::GetY(int, int y, int) const
			{
				return coordY[y];
			}

			float RectilinearMesh3D::GetZ(int, int, int z) const
			{
				return coordZ[z];
			}

			void RectilinearMesh3D::SetX(int x, float val)
			{
				coordX[x] = val;
			}

			void RectilinearMesh3D::SetY(int y, float val)
			{
				coordY[y] = val;
			}

			void RectilinearMesh3D::SetZ(int z, float val)
			{
				coordZ[z] = val;
			}

			void RectilinearMesh3D::SetXValues(float minX, float maxX)
			{
				SetRange(coordX, minX, maxX, xdim);
			}

			void RectilinearMesh3D::SetYValues(float minY, float maxY)
			{
				SetRange(coordY, minY, maxY, ydim);
			}

			void RectilinearMesh3D::SetZValues(float minZ, float maxZ)
			{
				SetRange(coordZ, minZ, maxZ, zdim);
			}

			void RectilinearMesh3D::WriteMesh(DBfile *db)
			{
				float *coords[3] = { coordX, coordY, coordZ };
				int dims[3] = { xdim, ydim, zdim };

				DBoptlist *optList = DBMakeOptlist(6);
				DBAddOption(optList, DBOPT_XLABEL, (void*)"Width");
				DBAddOption(optList, DBOPT_YLABEL, (void*)"Height");
				DBAddOption(optList, DBOPT_ZLABEL, (void*)"Depth");
				DBAddOption(optList, DBOPT_XUNITS, (void*)"parsec");
				DBAddOption(optList, DBOPT_YUNITS, (void*)"parsec");
				DBAddOption(optList, DBOPT_ZUNITS, (void*)"parsec");
				DBPutQuadmesh(db, (char *)meshName.c_str(), NULL, coords, dims, 3,
					DB_FLOAT, DB_COLLINEAR, optList);
				DBFreeOptlist(optList);
			}

			void RectilinearMesh3D::WriteMeshSlice(DBfile *db, const std::string &newMeshName,
				int sliceVal, int sliceDimension)
			{
				float *slice_coordX, *slice_coordY;
				int i, dims[2];

				if (sliceDimension == 0)
				{
					dims[0] = zdim; dims[1] = ydim;
					slice_coordX = new float[zdim];
					for (i = 0; i < zdim; ++i)
						slice_coordX[i] = coordZ[i];
					slice_coordY = new float[ydim];
					for (i = 0; i < ydim; ++i)
						slice_coordY[i] = coordY[i];
				}
				else if (sliceDimension == 1)
				{
					dims[0] = xdim; dims[1] = zdim;
					slice_coordX = new float[xdim];
					for (i = 0; i < xdim; ++i)
						slice_coordX[i] = coordX[i];
					slice_coordY = new float[zdim];
					for (i = 0; i < zdim; ++i)
						slice_coordY[i] = coordZ[i];
				}
				else
				{
					dims[0] = xdim; dims[1] = ydim;
					slice_coordX = new float[xdim];
					for (i = 0; i < xdim; ++i)
						slice_coordX[i] = coordX[i];
					slice_coordY = new float[ydim];
					for (i = 0; i < ydim; ++i)
						slice_coordY[i] = coordY[i];
				}

				float *coords[] = { slice_coordX, slice_coordY };

				DBoptlist *optList = DBMakeOptlist(6);
				DBAddOption(optList, DBOPT_XLABEL, (void*)"Width");
				DBAddOption(optList, DBOPT_YLABEL, (void*)"Height");
				DBAddOption(optList, DBOPT_ZLABEL, (void*)"Depth");
				DBAddOption(optList, DBOPT_XUNITS, (void*)"parsec");
				DBAddOption(optList, DBOPT_YUNITS, (void*)"parsec");
				DBPutQuadmesh(db, (char *)newMeshName.c_str(), NULL, coords, dims, 2,
					DB_FLOAT, DB_COLLINEAR, optList);
				DBFreeOptlist(optList);

				delete[] slice_coordX;
				delete[] slice_coordY;
			}

			void RectilinearMesh3D::SetRange(float *coord, float minval, float maxval, int steps)
			{
				for (int i = 0; i < steps; ++i)
				{
					float t = float(i) / float(steps - 1);
					coord[i] = t*maxval + (1.f - t)*minval;
				}
			}


		}
	}
}
