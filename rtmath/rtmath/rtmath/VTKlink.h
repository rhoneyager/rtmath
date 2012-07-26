#pragma once

/* This links with a user-built version of VTK 5.10
 * For use with pure VS2010 build - no CMake
 */

#ifndef WITH_CMAKE

#pragma comment(lib, "MapReduceMPI.lib")
#pragma comment(lib, "mpistubs.lib")
//#pragma comment(lib, "QVTK-gd.lib")
#pragma comment(lib, "vtkalglib.lib")
#pragma comment(lib, "vtkCharts.lib")
#pragma comment(lib, "vtkCommon.lib")
#pragma comment(lib, "vtkDICOMParser.lib")
#pragma comment(lib, "vtkexoIIc.lib")
#pragma comment(lib, "vtkexpat.lib")
#pragma comment(lib, "vtkFiltering.lib")
#pragma comment(lib, "vtkfreetype.lib")
#pragma comment(lib, "vtkftgl.lib")
#pragma comment(lib, "vtkGenericFiltering.lib")
#pragma comment(lib, "vtkGeovis.lib")
#pragma comment(lib, "vtkGraphics.lib")
#pragma comment(lib, "vtkhdf5.lib")
#pragma comment(lib, "vtkhdf5_hl.lib")
#pragma comment(lib, "vtkHybrid.lib")
#pragma comment(lib, "vtkImaging.lib")
#pragma comment(lib, "vtkInfovis.lib")
#pragma comment(lib, "vtkIO.lib")
#pragma comment(lib, "vtkjpeg.lib")
#pragma comment(lib, "vtklibxml2.lib")
#pragma comment(lib, "vtkmetaio.lib")
#pragma comment(lib, "vtkNetCDF.lib")
#pragma comment(lib, "vtkNetCDF_cxx.lib")
#pragma comment(lib, "vtkpng.lib")
#pragma comment(lib, "vtkproj4.lib")
#pragma comment(lib, "vtkRendering.lib")
#pragma comment(lib, "vtksqlite.lib")
#pragma comment(lib, "vtksys.lib")
#pragma comment(lib, "vtktiff.lib")
#pragma comment(lib, "vtkverdict.lib")
#pragma comment(lib, "vtkViews.lib")
#pragma comment(lib, "vtkVolumeRendering.lib")
#pragma comment(lib, "vtkWidgets.lib")
#pragma comment(lib, "vtkzlib.lib")

#endif

