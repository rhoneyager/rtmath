//#include "Stdafx-voronoi.h"
#include "../../rtmath/rtmath/defs.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

//#include <Voro/voro++.hh>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkDelaunay2D.h>
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
#include <vtkTransform.h>
#include <vtkTransformFilter.h>
#include <vtkHull.h>
#include <vtkImageData.h>
#include <vtkVersion.h>
#include <vtkMarchingCubes.h>
#include <vtkImplicitModeller.h>
#include <vtkVoxelModeller.h>
#include <vtkCleanPolyData.h>

#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-vtk.h"
namespace rtmath
{
	namespace plugins
	{
		namespace vtk
		{
			class SHARED_INTERNAL hullData
			{
			public:
				hullData()
					: volume(0), surfarea(0), vx(0), vy(0), vz(0), vproj(0), diameter(0),
					diameter1(0), diameter2(0), diameter3(0),
					beta(0), theta(0), phi(0),
					boundsCalced(false), mcCalced(false), alpha(0)
				{
					points = vtkSmartPointer< vtkPoints >::New();
					surfacePoints = vtkSmartPointer< vtkPoints >::New();
					polygons = vtkSmartPointer< vtkPolyData >::New();
					surfaceFilter = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
					decimate = vtkSmartPointer<vtkDecimatePro>::New();
					mc = vtkSmartPointer<vtkMarchingCubes>::New();
					std::fill_n(mins, 3, 0);
					std::fill_n(maxs, 3, 0);
					std::fill_n(area2d, 3, 0);
					std::fill_n(perimeter2d, 3, 0);
				}
				virtual ~hullData() {}
				vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter;
				vtkSmartPointer<vtkPoints> points;
				vtkSmartPointer<vtkDecimatePro> decimate;
				vtkSmartPointer<vtkPoints> surfacePoints;
				vtkSmartPointer<vtkPolyData> polygons;
				vtkSmartPointer<vtkDelaunay3D> hull;
				double alpha;
				double area2d[3], perimeter2d[3];

				vtkSmartPointer<vtkMarchingCubes> mc;
				double volume, surfarea, diameter;
				/// Max distance information for the three max axes
				double diameter1, diameter2, diameter3;
				double vx, vy, vz, vproj;
				/// Rotations to the principal axes
				double beta, theta, phi;
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
						points->GetPoint(i, crds);
						for (size_t j = 0; j < 3; j++)
							m[j](crds[j]);
					}
					//double mins[3], maxs[3];
					for (size_t j = 0; j < 3; j++)
					{
						mins[j] = boost::accumulators::min(m[j]);
						maxs[j] = boost::accumulators::max(m[j]);
					}

					boundsCalced = true;
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
					voxelModeller->SetSampleDimensions(50, 50, 50);
					voxelModeller->SetModelBounds(mins[0], maxs[0], mins[1], maxs[1], mins[2], maxs[2]);
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
					mc->GenerateValues(3, 0.5, 2.5);
					//mc->SetValue(0, isoValue);
					mc->Update();

					mcCalced = true;
				}
			};


			vtkConvexHull::vtkConvexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend) : rtmath::ddscat::convexHull(backend)
			{
				_p = boost::shared_ptr<hullData>(new hullData);
				_p->points->SetNumberOfPoints(backend->rows());
				_p->surfacePoints->SetNumberOfPoints(backend->rows());
				for (size_t i = 0; i < (size_t)backend->rows(); ++i)
				{
					auto it = backend->block<1, 3>(i, 0);
					_p->points->SetPoint(i, it(0), it(1), it(2));
					_p->surfacePoints->SetPoint(i, it(0), it(1), it(2));
				}
			}
			//vtkConvexHull() { _p = boost::shared_ptr<hullData>(new hullData); }
			boost::shared_ptr<rtmath::ddscat::convexHull> vtkConvexHull::generate
				(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend)
			{
				return boost::shared_ptr<rtmath::ddscat::convexHull>(new vtkConvexHull(backend));
			}

			vtkConvexHull::~vtkConvexHull() {}
			/// Construct the convex hull, and populate the quantities
			/// \todo Invoke in constructor?
			void vtkConvexHull::constructHull()
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
				auto fMaxDiameter = [](vtkPoints *base, size_t &a, size_t &b) -> double
				{
					double maxD = 0;
					a = 0;  b = 0;
					//std::cerr << "Rows: " << base.rows() << " cols: " << base.cols() << std::endl;
					size_t np = (size_t)base->GetNumberOfPoints();
					for (size_t i = 0; i < np; i++)
					{
						double it[3];
						base->GetPoint(i, it);

						for (size_t j = i + 1; j < (size_t)base->GetNumberOfPoints(); j++)
						{
							double ot[3];
							base->GetPoint(j, ot);
							double d = pow(it[0] - ot[0], 2.f)
								+ pow(it[1] - ot[1], 2.f)
								+ pow(it[2] - ot[2], 2.f);
							if (d > maxD)
							{
								maxD = d;
								a = i;
								b = j;
							}
						}
					}

					// These get optimized away, but are kept for debugging
					double ma[3];
					double mb[3];
					base->GetPoint(a, ma);
					base->GetPoint(b, mb);

					return sqrt(maxD);
				};

				// Get max diameter and store the point ids
				size_t mp1, mp2;
				_p->diameter = fMaxDiameter(delaunay3D->GetOutput()->GetPoints(), mp1, mp2);
				_p->diameter1 = _p->diameter;


				auto get2d = [&](size_t missingcrd)
				{
					// x,y,z refers to the coordinate that is ignored
					vtkSmartPointer<vtkTransform> trns = vtkSmartPointer<vtkTransform>::New();
					//trns->SetInput(surfacePointsPolys);
					// using pre-multiplication semantics
					if (missingcrd == 0)
						trns->RotateY(-90.);
					else if (missingcrd == 1)
						trns->RotateX(90.);
					// missingcrd == 2 is the default case - no need to rotate for delaunay2d
					trns->Scale(1, 1, 0);
					trns->Update();

					vtkSmartPointer<vtkCleanPolyData> cleanPolyData =
						vtkSmartPointer<vtkCleanPolyData>::New();
					cleanPolyData->SetInput(surfacePointsPolys);
					cleanPolyData->Update();

					vtkSmartPointer<vtkDelaunay2D> hull2d = vtkSmartPointer<vtkDelaunay2D>::New();
					hull2d->SetInput(cleanPolyData->GetOutput());
					hull2d->SetTransform(trns);
					hull2d->Update();

					vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter2 = vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
					vtkSmartPointer<vtkDecimatePro> decimate2 = vtkSmartPointer<vtkDecimatePro>::New();

					decimate2->SetInputConnection(hull2d->GetOutputPort());
					decimate2->SetBoundaryVertexDeletion(1);
					decimate2->SetPreserveTopology(1);
					decimate2->SetTargetReduction(1.0);
					decimate2->Update();

					surfaceFilter2->SetInputConnection(hull2d->GetOutputPort());
					surfaceFilter2->Update();

					vtkSmartPointer<vtkTriangleFilter> triFilter2 =
						vtkSmartPointer<vtkTriangleFilter>::New();
					triFilter2->SetInputConnection(surfaceFilter2->GetOutputPort());
					triFilter2->Update();


					vtkSmartPointer<vtkMassProperties> massFilter2 =
						vtkSmartPointer<vtkMassProperties>::New();
					massFilter2->SetInputConnection(triFilter->GetOutputPort());
					massFilter2->Update();

					_p->area2d[missingcrd] = massFilter2->GetVolume();
					_p->perimeter2d[missingcrd] = massFilter2->GetSurfaceArea();
				};

				//get2d(0);
				//get2d(1);
				//get2d(2);



				/*
				// Use point ids to construct a rotation that puts these points parallel to the x axis
				auto calcRot = [](const vtkSmartPointer<vtkPoints> &base, size_t &a, size_t &b, double &thetar, double &phir)
				{
				double i[3];
				base->GetPoint(a,i);
				Eigen::Vector3d va(i[0], i[1], i[2]);
				base->GetPoint(b, i);
				Eigen::Vector3d vb(i[0], i[1], i[2]);

				Eigen::Vector3d vnet = va - vb;
				vnet.normalize();

				// By convenient convention, a1=x^ cos theta + y^ sin theta cos phi + z^ sin theta sin phi
				// All I really need are theta and phi at this stage, not beta.

				/// \todo Check these: they do not seem correct!
				thetar = acos(vnet(0));
				phir = acos(vnet(1) / sin(thetar));
				};

				double thetar, phir;
				calcRot(delaunay3D->GetOutput()->GetPoints(), mp1, mp2, thetar, phir);
				double scale = (boost::math::constants::pi<double>()) / 180.0;
				double thetad = thetar / scale;
				double phid = phir / scale;

				Eigen::Matrix3d RotEff;
				rtmath::ddscat::rotationMatrix(thetad, phid, 0., RotEff);
				Eigen::Matrix3d RotInv = RotEff.inverse();
				// Rotate the points and project into the y-z axis.
				vtkSmartPointer<vtkPoints> ptsRotated1 = vtkSmartPointer< vtkPoints >::New();
				vtkSmartPointer<vtkPoints> ptsProj1 = vtkSmartPointer< vtkPoints >::New();
				ptsRotated1->SetNumberOfPoints(delaunay3D->GetOutput()->GetPoints()->GetNumberOfPoints());
				ptsProj1->SetNumberOfPoints(delaunay3D->GetOutput()->GetPoints()->GetNumberOfPoints());
				for (size_t i = 0; i < (size_t)ptsRotated1->GetNumberOfPoints(); ++i)
				{
				Eigen::Vector3d x;
				delaunay3D->GetOutput()->GetPoints()->GetPoint(i, x.data());
				Eigen::Vector3d y = RotInv * x;
				ptsRotated1->SetPoint(i, y.data());
				Eigen::Vector3d z = y;
				z(0) = 0;
				ptsProj1->SetPoint(i, y.data());
				//std::cerr << i << "\t" << y.transpose() << std::endl;
				}

				// Get the second max diameter and store the point ids
				_p->diameter2 = fMaxDiameter(ptsProj1, mp1, mp2);

				// Use these next point ids to construct a rotation with them parallel to the y axis

				auto calcRot2d = [](const vtkSmartPointer<vtkPoints> &base, size_t &a, size_t &b, double &betar)
				{
				double i[3];
				base->GetPoint(a, i);
				Eigen::Vector3d va(i[0], i[1], i[2]);
				base->GetPoint(b, i);
				Eigen::Vector3d vb(i[0], i[1], i[2]);

				Eigen::Vector3d vnet = va - vb;
				vnet.normalize();

				// By convenient convention, a1=x^ cos theta + y^ sin theta cos phi + z^ sin theta sin phi
				// All I really need are theta and phi at this stage, not beta.

				betar = atan2(vnet(2), vnet(1));
				};

				double betar;
				//calcRot2d(delaunay2D->GetOutput()->GetPoints(), mp1, mp2, betar);
				calcRot2d(ptsProj1, mp1, mp2, betar);
				double betad = betar / scale;

				// Store the required rotation information
				if (betad < 0) betad += 360;
				if (phid < 0) phid += 360;
				if (thetad < 0) thetad *= -1.;
				_p->beta = betad;
				_p->theta = thetad;
				_p->phi = phid;

				/// \todo Calculate the third diameter, eventually.
				*/
			}

			double vtkConvexHull::volume() const { return _p->volume; }
			double vtkConvexHull::surfaceArea() const { return _p->surfarea; }
			double vtkConvexHull::maxDiameter() const { return _p->diameter; }
			void vtkConvexHull::principalAxes(double &beta, double &theta, double &phi) const { beta = _p->beta; theta = _p->theta; phi = _p->phi; }
			void vtkConvexHull::area2d(double out[3]) const { std::copy_n(_p->area2d, 3, out); }
			void vtkConvexHull::perimeter2d(double out[3]) const { std::copy_n(_p->perimeter2d, 3, out); }


			void vtkConvexHull::writeVTKraw(const std::string &filename) const
			{
				//writeVTKpolys(filename, _p->rawpolygons);
				_p->calcMarchingCubes();
				vtkSmartPointer<vtkXMLPolyDataWriter> writer =
					vtkSmartPointer<vtkXMLPolyDataWriter>::New();
				writer->SetInputConnection(_p->mc->GetOutputPort());
				writer->SetFileName(filename.c_str());
				writer->Write();
			}

			void vtkConvexHull::writeVTKhull(const std::string &filename) const
			{
				//writeVTKpolys(filename, _p->polygons);
				vtkSmartPointer<vtkXMLPolyDataWriter> writer =
					vtkSmartPointer<vtkXMLPolyDataWriter>::New();
				writer->SetInputConnection(_p->surfaceFilter->GetOutputPort());
				writer->SetFileName(filename.c_str());
				writer->Write();
			}

			/*
			void writeVTKpolys(const std::string &filename, const vtkSmartPointer< vtkPolyData > &src)
			{
			vtkSmartPointer<vtkXMLPolyDataWriter> pointsWriter =
			vtkSmartPointer<vtkXMLPolyDataWriter>::New();
			pointsWriter->SetFileName(filename.c_str());
			pointsWriter->SetInput(src);
			pointsWriter->Write();
			}
			*/
		}
	}
}

