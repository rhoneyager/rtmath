/* vtktest
* This program is designed to test computation and writing of hulls and delaunay
* triangulations for incorporation into the library core.
*/
#include <iostream>
#include <string>
#include <vector>
#include <set>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>

#include <vtkDelaunay3D.h>
#include <vtkOrderedTriangulator.h>
#include <vtkHull.h>
#include <vtkPointsProjectedHull.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkStructuredGrid.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkVersion.h>
#include <vtkSphereSource.h>
#include <vtkPointSource.h>
#include <vtkProperty.h>
#include <vtkDataSetMapper.h>
#include <vtkActor.h>
#include <vtkPolygon.h>
#include <vtkMath.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCleanPolyData.h>
#include <vtkXMLPolyDataReader.h>

#include <Ryan_Debug/debug.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/splitSet.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/error/error.h"

int main(int argc, char** argv)
{
	using namespace std;
	using namespace rtmath;
	using namespace boost::filesystem;

	try {
		cerr << "rtmath-vtktest\n\n";

		vtkSmartPointer<vtkSphereSource> sphereSource = 
			vtkSmartPointer<vtkSphereSource>::New();
		sphereSource->SetRadius(10);
		sphereSource->SetPhiResolution(50);
		sphereSource->SetThetaResolution(50);
		sphereSource->Update();

		vtkSmartPointer<vtkPointSource> pointSource = 
			vtkSmartPointer<vtkPointSource>::New();
		pointSource->SetNumberOfPoints(40);
		pointSource->SetRadius(2);
		pointSource->Update();

		{
			vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			writer->SetFileName("input.vtp");
			writer->SetInputConnection(sphereSource->GetOutputPort());
			writer->Write();
		}

		{
			vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			writer->SetFileName("points.vtp");
			writer->SetInputConnection(pointSource->GetOutputPort());
			writer->Write();
		}

		vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = 
			vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
		smoothFilter->SetInputConnection(0, sphereSource->GetOutputPort());
		smoothFilter->SetInputConnection(1, pointSource->GetOutputPort());
		smoothFilter->Update();

		vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
			vtkSmartPointer<vtkXMLPolyDataWriter>::New();
		writer->SetFileName("output.vtp");
		writer->SetInputConnection(smoothFilter->GetOutputPort());
		writer->Write();

		//Read the file
		vtkSmartPointer<vtkXMLPolyDataReader> reader =
			vtkSmartPointer<vtkXMLPolyDataReader>::New();
		reader->SetFileName("points.vtp");

		vtkSmartPointer<vtkDataSetMapper> originalMapper =
			vtkSmartPointer<vtkDataSetMapper>::New();
		originalMapper->SetInputConnection(reader->GetOutputPort());

		vtkSmartPointer<vtkActor> originalActor =
			vtkSmartPointer<vtkActor>::New();
		originalActor->SetMapper(originalMapper);
		originalActor->GetProperty()->SetColor(1,0,0);

		// Clean the polydata. This will remove duplicate points that may be
		// present in the input data.
		vtkSmartPointer<vtkCleanPolyData> cleaner =
			vtkSmartPointer<vtkCleanPolyData>::New();
		cleaner->SetInputConnection (reader->GetOutputPort());

		// Generate a tetrahedral mesh from the input points. By
		// default, the generated volume is the convex hull of the points.
		vtkSmartPointer<vtkDelaunay3D> delaunay3D =
			vtkSmartPointer<vtkDelaunay3D>::New();
		delaunay3D->SetInputConnection (cleaner->GetOutputPort());

		vtkSmartPointer<vtkDataSetMapper> delaunayMapper =
			vtkSmartPointer<vtkDataSetMapper>::New();
		delaunayMapper->SetInputConnection(delaunay3D->GetOutputPort());

		vtkSmartPointer<vtkActor> delaunayActor =
			vtkSmartPointer<vtkActor>::New();
		delaunayActor->SetMapper(delaunayMapper);
		delaunayActor->GetProperty()->SetColor(1,0,0);

		// Generate a mesh from the input points. If Alpha is non-zero, then
		// tetrahedra, triangles, edges and vertices that lie within the
		// alpha radius are output.
		vtkSmartPointer<vtkDelaunay3D> delaunay3DAlpha =
			vtkSmartPointer<vtkDelaunay3D>::New();
		delaunay3DAlpha->SetInputConnection (cleaner->GetOutputPort());
		delaunay3DAlpha->SetAlpha(0.1);

		vtkSmartPointer<vtkXMLUnstructuredGridWriter> writerb = 
			vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
		writerb->SetFileName("output2.vtu");
		writerb->SetInputConnection(delaunay3D->GetOutputPort());
		writerb->Write();
		/*
		vtkSmartPointer<vtkDataSetMapper> delaunayAlphaMapper =
			vtkSmartPointer<vtkDataSetMapper>::New();
		delaunayAlphaMapper->SetInputConnection(delaunay3DAlpha->GetOutputPort());

		vtkSmartPointer<vtkActor> delaunayAlphaActor =
			vtkSmartPointer<vtkActor>::New();
		delaunayAlphaActor->SetMapper(delaunayAlphaMapper);
		delaunayAlphaActor->GetProperty()->SetColor(1,0,0);

		// Visualize

		// Define viewport ranges
		// (xmin, ymin, xmax, ymax)
		double leftViewport[4] = {0.0, 0.0, 0.33, 1.0};
		double centerViewport[4] = {0.33, 0.0, 0.66, 1.0};
		double rightViewport[4] = {0.66, 0.0, 1.0, 1.0};

		// Create a renderer, render window, and interactor
		vtkSmartPointer<vtkRenderer> originalRenderer =
			vtkSmartPointer<vtkRenderer>::New();
		vtkSmartPointer<vtkRenderer> delaunayRenderer =
			vtkSmartPointer<vtkRenderer>::New();
		vtkSmartPointer<vtkRenderer> delaunayAlphaRenderer =
			vtkSmartPointer<vtkRenderer>::New();

		vtkSmartPointer<vtkRenderWindow> renderWindow =
			vtkSmartPointer<vtkRenderWindow>::New();
		renderWindow->SetSize(900,300);

		renderWindow->AddRenderer(originalRenderer);
		originalRenderer->SetViewport(leftViewport);
		renderWindow->AddRenderer(delaunayRenderer);
		delaunayRenderer->SetViewport(centerViewport);
		renderWindow->AddRenderer(delaunayAlphaRenderer);
		delaunayAlphaRenderer->SetViewport(rightViewport);

		vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
			vtkSmartPointer<vtkRenderWindowInteractor>::New();
		renderWindowInteractor->SetRenderWindow(renderWindow);

		originalRenderer->AddActor(originalActor);
		delaunayRenderer->AddActor(delaunayActor);
		delaunayAlphaRenderer->AddActor(delaunayAlphaActor);

		originalRenderer->SetBackground(.3, .6, .3);
		delaunayRenderer->SetBackground(.4, .6, .3);
		delaunayAlphaRenderer->SetBackground(.5, .6, .3);

		// Render and interact
		renderWindow->Render();
		renderWindowInteractor->Start();
		*/
	} catch (std::exception &e)
	{
		cerr << e.what() << endl;
		return 1;
	}
	return 0;
}

