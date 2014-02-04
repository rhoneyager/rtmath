// Stdafx-voronoi.h : include file for standard system include files,
// or project specific include files that are used frequently,
// but are changed infrequently
//#pragma once
#ifndef STDAFX_VORONOI_H
#define STDAFX_VORONOI_H

#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS

#include <algorithm>

/*
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
*/

#include <cmath>
#include <complex>
#include <set>
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#pragma warning( disable : 4244 ) // warning C4244: 'return' : conversion from '__int64' to 'int', possible loss of data
#include <Voro++/voro++.hh>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay3D.h>
#include <vtkDecimatePro.h>
#include <vtkUnstructuredGrid.h>
#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkMassProperties.h>
#include <vtkTriangleFilter.h>
#include <vtkHull.h>
#include <vtkImageData.h>
#include <vtkMarchingCubes.h>
#include <vtkImplicitModeller.h>
#include <vtkVoxelModeller.h>


#include <Eigen/Core>

#endif


