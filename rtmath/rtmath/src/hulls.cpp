#include "Stdafx-voronoi.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>

#include <Voro++/voro++.hh>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay3D.h>
#include <vtkDecimatePro.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSphereSource.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkXMLStructuredGridWriter.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkMassProperties.h>
#include <vtkTriangleFilter.h>
#include <vtkHull.h>
#include <vtkImageData.h>
#include <vtkVersion.h>
#include <vtkMarchingCubes.h>
#include <vtkImplicitModeller.h>
#include <vtkVoxelModeller.h>

#include "../rtmath/ddscat/hulls.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace ddscat
	{
		void SHARED_INTERNAL writeVTKpolys(const std::string &filename, const vtkSmartPointer< vtkPolyData > &src)
		{
			vtkSmartPointer<vtkXMLPolyDataWriter> pointsWriter = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			pointsWriter->SetFileName(filename.c_str());
			pointsWriter->SetInput(src);
			pointsWriter->Write();
		}

		class SHARED_INTERNAL hullData
		{
		public:
			hullData()
				: volume(0), surfarea(0), vx(0), vy(0), vz(0), vproj(0), diameter(0),
				boundsCalced(false), mcCalced(false)
			{
				points = vtkSmartPointer< vtkPoints >::New();
				surfacePoints = vtkSmartPointer< vtkPoints >::New();
				polygons = vtkSmartPointer< vtkPolyData >::New();
				surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
				decimate = vtkSmartPointer<vtkDecimatePro>::New();
				mc = vtkSmartPointer<vtkMarchingCubes>::New();
				std::fill_n(mins,3,0);
				std::fill_n(maxs,3,0);
			}
			virtual ~hullData() {}
			vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter;
			vtkSmartPointer<vtkPoints> points;
			vtkSmartPointer<vtkDecimatePro> decimate;
			vtkSmartPointer<vtkPoints> surfacePoints;
			vtkSmartPointer<vtkPolyData> polygons;
			vtkSmartPointer<vtkDelaunay3D> hull;
			vtkSmartPointer<vtkMarchingCubes> mc;
			double volume, surfarea, diameter;
			double vx, vy, vz, vproj;
			double mins[3], maxs[3];
			bool boundsCalced;
			bool mcCalced;

			/// Function to calculate the bounds of the shape
			void calcBounds()
			{
				if (boundsCalced) return;

				using namespace boost::accumulators;
				std::vector<accumulator_set<double, stats<tag::min, tag::max> > > m(3);
				//for (auto it = _shp->latticePtsStd.begin(); it != _shp->latticePtsStd.end(); ++it)
				for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++)
				{
					double crds[3];
					points->GetPoint(i,crds);
					for (size_t j=0;j<3;j++)
						m[j](crds[j]);
				}
				//double mins[3], maxs[3];
				for (size_t j=0; j<3; j++)
				{
					mins[j] = boost::accumulators::min(m[j]);
					maxs[j] = boost::accumulators::max(m[j]);
				}

				boundsCalced=true;
			}

			/// Function to generate the surface using marching cubes
			void calcMarchingCubes()
			{
				if (mcCalced) return;
				calcBounds();

				vtkSmartPointer<vtkPolyData> pointsPolys = vtkSmartPointer< vtkPolyData >::New();
				pointsPolys->SetPoints(points);

				vtkSmartPointer<vtkImageData> volume =
					vtkSmartPointer<vtkImageData>::New();

				vtkSmartPointer<vtkVoxelModeller> voxelModeller = 
					vtkSmartPointer<vtkVoxelModeller>::New();
				//vtkSmartPointer<vtkImplicitModeller> voxelModeller = 
				//	vtkSmartPointer<vtkImplicitModeller>::New();
				voxelModeller->SetSampleDimensions(50,50,50);
				voxelModeller->SetModelBounds(mins[0],maxs[0],mins[1],maxs[1],mins[2],maxs[2]);
				voxelModeller->SetScalarTypeToFloat();
				voxelModeller->SetMaximumDistance(.1);

				voxelModeller->SetInput(pointsPolys);
				voxelModeller->Update();
				volume->DeepCopy(voxelModeller->GetOutput());


				mc->SetInput(volume);

				mc->ComputeNormalsOn();
				mc->ComputeGradientsOn();
				mc->ComputeScalarsOn();
				double isoValue = 0.5;
				mc->GenerateValues(3,0.5,2.5);
				//mc->SetValue(0, isoValue);
				mc->Update();

				mcCalced = true;
			}
		};

		hull::hull()
		{
			_p = boost::shared_ptr<hullData>(new hullData);
		}

		hull::hull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend)
		{
			_p = boost::shared_ptr<hullData>(new hullData);
			_p->points->SetNumberOfPoints(backend->rows());
			_p->surfacePoints->SetNumberOfPoints(backend->rows());
			for (size_t i = 0; i < (size_t) backend->rows(); ++i)
			{
				auto it = backend->block<1,3>(i,0);
				_p->points->SetPoint(i, it(0), it(1), it(2));
				_p->surfacePoints->SetPoint(i, it(0), it(1), it(2));
			}
		}

		void hull::writeVTKraw(const std::string &filename) const
		{
			//writeVTKpolys(filename, _p->rawpolygons);
			_p->calcMarchingCubes();
			vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			writer->SetInputConnection(_p->mc->GetOutputPort());
			writer->SetFileName(filename.c_str());
			writer->Write();
		}

		void hull::writeVTKhull(const std::string &filename) const
		{
			//writeVTKpolys(filename, _p->polygons);
			vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
				vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			writer->SetInputConnection(_p->surfaceFilter->GetOutputPort());
			writer->SetFileName(filename.c_str());
			writer->Write();
		}

		//convexHull::convexHull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &src)
		//	: hull(src) {}

		convexHull::convexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> >  src) : hull(src)
		{
		}

		void convexHull::constructHull()
		{
			/// \todo Move this to a funtion that executes on library load
			vtkObject::GlobalWarningDisplayOff();

			//_p->calcVoronoi();

			vtkSmartPointer<vtkPolyData> surfacePointsPolys = vtkSmartPointer< vtkPolyData >::New();
			surfacePointsPolys->SetPoints(_p->surfacePoints);

			vtkSmartPointer<vtkDelaunay3D> delaunay3D =
				vtkSmartPointer<vtkDelaunay3D>::New();
			delaunay3D->SetInput(surfacePointsPolys);
			delaunay3D->Update();

			
			_p->decimate->SetInputConnection(delaunay3D->GetOutputPort());
			_p->decimate->SetBoundaryVertexDeletion(1);
			_p->decimate->SetPreserveTopology(1);
			_p->decimate->SetTargetReduction(1.0);
			_p->decimate->Update();

			_p->surfaceFilter->SetInputConnection(delaunay3D->GetOutputPort());
			_p->surfaceFilter->Update();

			vtkSmartPointer<vtkTriangleFilter> triFilter = 
				vtkSmartPointer<vtkTriangleFilter>::New();
			triFilter->SetInputConnection(_p->surfaceFilter->GetOutputPort());
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
			

			_p->hull = delaunay3D;
			// Change this!!!!!!

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

			_p->diameter = fMaxDiameter(delaunay3D->GetOutput()->GetPoints());
		}

		double hull::maxDiameter() const { return _p->diameter; }

		double hull::volume() const { return _p->volume; }

		double hull::surfaceArea() const { return _p->surfarea; }

	}
}

