#include "../rtmath/Stdafx.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSphereSource.h>
#include <vtkSmoothPolyDataFilter.h>
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
#include <vtkVersion.h>

#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace ddscat
	{
		void writeVTKpolys(const std::string &filename, const vtkSmartPointer< vtkPolyData > &src)
		{
			vtkSmartPointer<vtkXMLPolyDataWriter> pointsWriter = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			pointsWriter->SetFileName(filename.c_str());
			pointsWriter->SetInput(src);
			pointsWriter->Write();
		}

		class hullData
		{
		public:
			hullData()
				: volume(0), surfarea(0), vx(0), vy(0), vz(0), vproj(0), diameter(0)
			{
				points = vtkSmartPointer< vtkPoints >::New();
				hullPts = vtkSmartPointer< vtkPoints >::New();
				rawpolygons = vtkSmartPointer< vtkPolyData >::New();
				polygons = vtkSmartPointer< vtkPolyData >::New();
			}
			virtual ~hullData() {}
			vtkSmartPointer<vtkPoints> points;
			vtkSmartPointer<vtkPolyData> rawpolygons;
			mutable vtkSmartPointer<vtkPoints> hullPts;
			vtkSmartPointer<vtkPolyData> polygons;
			vtkSmartPointer<vtkSmoothPolyDataFilter> hull;
			double volume, surfarea, diameter;
			double vx, vy, vz, vproj;
		};

		hull::hull()
		{
			_p = boost::shared_ptr<hullData>(new hullData);
		}

		/*
		hull::hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &backend)
		{
		_p = boost::shared_ptr<hullData>(new hullData);
		_p->points = backend;
		_p->hullPts = backend;
		}
		*/

		hull::hull(const Eigen::Matrix<float, Eigen::Dynamic, 3> &backend)
		{
			_p = boost::shared_ptr<hullData>(new hullData);
			_p->points->SetNumberOfPoints(backend.rows());

			for (size_t i = 0; i < (size_t) backend.rows(); ++i)
			{
				auto it = backend.block<1,3>(i,0);
				_p->points->SetPoint(i, it(0), it(1), it(2));
			}
			_p->rawpolygons->SetPoints(_p->points);
		}

		void hull::writeVTKraw(const std::string &filename) const
		{
			writeVTKpolys(filename, _p->rawpolygons);
		}

		void hull::writeVTKhull(const std::string &filename) const
		{
			//writeVTKpolys(filename, _p->polygons);
			vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			writer->SetInputConnection(_p->hull->GetOutputPort());
			writer->SetFileName(filename.c_str());
			writer->Write();
		}

		//convexHull::convexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src)
		//	: hull(src) {}

		convexHull::convexHull(const Eigen::Matrix<float, Eigen::Dynamic, 3>& src) : hull(src) {}

		void convexHull::constructHull()
		{

			// Create the convex hull
			/*
			vtkSmartPointer<vtkHull> hullFilter = 
				vtkSmartPointer<vtkHull>::New();
			hullFilter->SetInput(_p->rawpolygons);
			//hullFilter->SetInputConnection(reader->GetOutputPort());
			hullFilter->AddCubeFacePlanes ();
			hullFilter->Update();
			*/
			/*
			vtkSmartPointer<vtkDelaunay3D> delaunay3D =
				vtkSmartPointer<vtkDelaunay3D>::New();
			delaunay3D->SetInput (_p->rawpolygons);
			delaunay3D->Update();
			*/

			vtkSmartPointer<vtkSphereSource> sphereSource = 
			vtkSmartPointer<vtkSphereSource>::New();
			sphereSource->SetRadius(50);
			sphereSource->SetPhiResolution(5);
			sphereSource->SetThetaResolution(5);
			sphereSource->Update();

			vtkSmartPointer<vtkSmoothPolyDataFilter> smoothFilter = 
			vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
			smoothFilter->SetInputConnection(0, sphereSource->GetOutputPort());
			smoothFilter->SetInput(1, _p->rawpolygons);
			// ERROR: NO CELLS TO SUBDIVIDE! Am I loading the input properly?
			smoothFilter->Update();
			/*
			vtkSmartPointer<vtkTriangleFilter> triFilter = 
				vtkSmartPointer<vtkTriangleFilter>::New();
			triFilter->SetInputConnection(delaunay3D->GetOutputPort());
			triFilter->Update();

			
			vtkSmartPointer<vtkMassProperties> massFilter = 
				vtkSmartPointer<vtkMassProperties>::New();
			massFilter->SetInputConnection(triFilter->GetOutputPort());
			massFilter->Update();

			_p->volume = massFilter->GetVolume();
			_p->surfarea = massFilter->GetSurfaceArea();
			_p->vx = massFilter->GetVolumeX();
			_p->vy = massFilter->GetVolumeY();
			_p->vz = massFilter->GetVolumeZ();
			_p->vproj = massFilter->GetVolumeProjected();
			*/

			_p->hull = smoothFilter;
			// Change this!!!!!!
			_p->hullPts->SetNumberOfPoints(_p->points->GetNumberOfPoints());
			_p->hullPts = smoothFilter->GetOutput()->GetPoints();
			//_p->polygons = delaunay3D->GetOutput();

			// Just calculate the max diameter here
			auto fMaxDiameter = [](const vtkSmartPointer<vtkPoints> &base) -> double
			{
				double maxD = 0;
				size_t a = 0, b = 0;
				//std::cerr << "Rows: " << base.rows() << " cols: " << base.cols() << std::endl;

				for (size_t i = 0; i < (size_t) base->GetNumberOfPoints(); i++)
				{
					double it[3];
					base->GetPoint(i, it);

					for (size_t j=i+1; j<(size_t) base->GetNumberOfPoints(); j++)
					{
						double ot[3];
						base->GetPoint(j,ot);
						double d = pow(it[0]-ot[0],2.f)
							+ pow(it[1]-ot[1],2.f)
							+ pow(it[2]-ot[2],2.f);
						if (d > maxD)
						{
							maxD = d;
							a = i;
							b = j;
						}
					}
				}

				return sqrt(maxD);
			};

			_p->diameter = fMaxDiameter(_p->hullPts);
		}

		double hull::maxDiameter() const { return _p->diameter; }

		double hull::volume() const { return _p->volume; }

		double hull::surfaceArea() const { return _p->surfarea; }

		/*
		//concaveHull::concaveHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src)
		//	: hull(src) { }

		concaveHull::concaveHull(const Eigen::Matrix<float, Eigen::Dynamic, 3>& src) : hull(src) {}

		concaveHull::~concaveHull() { }

		void concaveHull::constructHull(double alpha)
		{
		_p->hullPts->clear();

		pcl::ConcaveHull<pcl::PointXYZ> chull;
		chull.setDimension(3);
		chull.setInputCloud (_p->points);
		chull.setAlpha(alpha);
		chull.reconstruct(*(_p->hullPts.get()), _p->polygons);

		//_findVS();
		}

		*/
	}
}

