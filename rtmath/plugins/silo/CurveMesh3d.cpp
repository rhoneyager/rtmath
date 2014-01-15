

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
#include "CurveMesh3d.h"

#define DB_USE_MODERN_DTPTR
#include <silo.h>

namespace rtmath {
	namespace plugins {
		namespace silo {

			CurveMesh3D::CurveMesh3D(int X, int Y, int Z) : QuadMesh3D(X, Y, Z, false)
			{
			}

			CurveMesh3D::~CurveMesh3D()
			{
			}

			float CurveMesh3D::GetX(int x, int y, int z) const
			{
				return coordX[z*(ydim*xdim) + y*xdim + x];
			}

			float CurveMesh3D::GetY(int x, int y, int z) const
			{
				return coordY[z*(ydim*xdim) + y*xdim + x];
			}

			float CurveMesh3D::GetZ(int x, int y, int z) const
			{
				return coordZ[z*(ydim*xdim) + y*xdim + x];
			}

			void CurveMesh3D::SetX(int x, int y, int z, float val)
			{
				coordX[z*(ydim*xdim) + y*xdim + x] = val;
			}

			void CurveMesh3D::SetY(int x, int y, int z, float val)
			{
				coordY[z*(ydim*xdim) + y*xdim + x] = val;
			}

			void CurveMesh3D::SetZ(int x, int y, int z, float val)
			{
				coordZ[z*(ydim*xdim) + y*xdim + x] = val;
			}

			void CurveMesh3D::WriteMesh(DBfile *db, const char *name)
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
				DBPutQuadmesh(db, (char *)meshName.c_str(), NULL, coords, dims,
					3, DB_FLOAT, DB_NONCOLLINEAR, NULL);
				DBFreeOptlist(optList);
			}

			void CurveMesh3D::WriteMeshSlice(DBfile *db, const std::string &newMeshName,
				int sliceVal, int sliceDimension)
			{
				float *slice_coordX, *slice_coordY;
				int index = 0;
				int dims[2];

				if (sliceDimension == 0)
				{
					dims[0] = zdim; dims[1] = ydim;
					slice_coordX = new float[zdim * ydim];
					slice_coordY = new float[zdim * ydim];
					for (int y = 0; y < ydim; ++y)
					for (int z = 0; z < zdim; ++z, ++index)
					{
						slice_coordX[index] = GetZ(sliceVal, y, z);
						slice_coordY[index] = GetY(sliceVal, y, z);
					}
				}
				else if (sliceDimension == 1)
				{
					dims[0] = xdim; dims[1] = zdim;
					slice_coordX = new float[xdim * zdim];
					slice_coordY = new float[xdim * zdim];
					for (int z = 0; z < zdim; ++z)
					for (int x = 0; x < xdim; ++x, ++index)
					{
						slice_coordX[index] = GetX(x, sliceVal, z);
						slice_coordY[index] = GetZ(x, sliceVal, z);
					}
				}
				else
				{
					dims[0] = xdim; dims[1] = ydim;
					slice_coordX = new float[xdim * ydim];
					slice_coordY = new float[xdim * ydim];
					for (int y = 0; y < ydim; ++y)
					for (int x = 0; x < xdim; ++x, ++index)
					{
						slice_coordX[index] = GetX(x, y, sliceVal);
						slice_coordY[index] = GetY(x, y, sliceVal);
					}
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


		}
	}
}

