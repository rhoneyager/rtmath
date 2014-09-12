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

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkSphereSource.h>
#include <vtkStructuredGrid.h>
#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkXMLStructuredGridWriter.h>

#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-vtk.h"

namespace rtmath {
	namespace registry {
		using std::shared_ptr;
		using rtmath::ddscat::ddOutput;
		//using namespace rtmath::plugins::silo;

		/*
		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<rtmath::ddscat::shapefile::shapefile>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const rtmath::ddscat::shapefile::shapefile *s)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->iotype();

			using std::shared_ptr;
			std::shared_ptr<silo_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<silo_handle>(new silo_handle(filename.c_str(), iotype));
			}
			else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<silo_handle>(sh);
			}

			/// \todo Modify to also support external symlinks
			//shared_ptr<Group> newstatsbase = write_hdf5_statsrawdata(grpHash, s);
			//shared_ptr<Group> newshapebase = write_hdf5_shaperawdata(grpHash, s->_shp.get());

			std::string meshname("Points_");
			//if (key)
			//	meshname.append(std::string(key));
			//else meshname.append(s->filename);

			Eigen::MatrixXf lPts(s->latticePts.rows(), s->latticePts.cols());
			lPts = s->latticePts;
			const char* axislabels[] = { "x", "y", "z" };
			const char* axisunits[] = { "dipoles", "dipoles", "dipoles" };

			std::string dielsName = meshname;
			dielsName.append("Dielectrics");
			std::string indexName = meshname;
			indexName.append("Point_IDs");
			auto pm = h->file->createPointMesh<float>(meshname.c_str(), lPts, axislabels, axisunits);

			// Create a 3d mesh, also for the dielectrics
			Eigen::Array3i mins = s->mins.cast<int>(), maxs = s->maxs.cast<int>();
			// Doing this so that the ends of the shape do not get chopped off
			mins -= 2 * Eigen::Array3i::Ones(); maxs += 2 * Eigen::Array3i::Ones();
			Eigen::Array3i span = maxs - mins + 1;
			int meshSize = span.prod();
			auto getCoords = [&](int i)->Eigen::Array3i
			{
				Eigen::Array3i crd;
				int x, y, z;
				// Iterate first over z, then y, then x
				x = i / (span(0)*span(1));
				//crd(1) = (i % (span(2)*span(1))) / span(2);
				y = (i - (x*span(0)*span(1))) / span(0); // it's not (i - i), as x involves an INTEGER division!
				z = i % span(0);
				crd(2) = x; crd(1) = y; crd(0) = z;
				crd += mins;
				//x = crd(0); y = crd(1); z = crd(2);
				return crd;
			};
			auto getIndex = [&](Eigen::Array3i i) -> int
			{
				int res = 0;
				i -= mins;
				res = span(0) * span(1) * i(2);
				res += span(0) * i(1);
				res += i(0);
				//int x = i(2), y = i(1), z = i(0);
				//res = z + (span(2) * (y + (span(1)*x)));
				return res;
			};
			Eigen::MatrixXf mDiels(meshSize, 1);
			mDiels.setZero();
			for (size_t i = 0; i < (size_t)s->latticePts.rows(); ++i)
			{
				Eigen::Array3i a; a(0) = (int)s->latticePts(i, 0);
				a(1) = (int)s->latticePts(i, 1); a(2) = (int)s->latticePts(i, 2);
				int index = getIndex(a);
				mDiels(index, 0) = (float)s->latticePtsRi(i, 0);
			}
			Eigen::VectorXf xs(span(0), 1), ys(span(1), 1), zs(span(2), 1);
			xs.setLinSpaced(s->mins(0), s->maxs(0));
			ys.setLinSpaced(s->mins(1), s->maxs(1));
			zs.setLinSpaced(s->mins(2), s->maxs(2));

			int dimsizes[] = { span(0), span(1), span(2) };
			const float *dims[] = { xs.data(), ys.data(), zs.data() };
			auto mesh = h->file->createRectilinearMesh<float>(
				"Shp_Mesh",
				3, dims, dimsizes,
				axislabels, axisunits);
			// Write the array of zone ids
			mesh->writeData<float>("Shp_Mesh_Dielectrics", mDiels.data(), "None");


			// Write the other matrices
			Eigen::MatrixXi lRi = s->latticePtsRi.col(0).cast<int>();
			pm->writeData<int>(dielsName.c_str(), lRi.data(), "Dimensionless");
			Eigen::MatrixXi lIndices = s->latticeIndex.cast<int>();
			pm->writeData<int>(indexName.c_str(), lIndices.data(), "Dimensionless");

			// Write the extra matrices
			for (const auto &extras : s->latticeExtras)
			{
				std::string varname = meshname;
				varname.append(extras.first);
				if (extras.second->cols() < 4)
					pm->writeData<float>(varname.c_str(), extras.second->data(), "Unknown");
				else
				{
					// Make a new mesh from the first three columns of the data
					std::string meshname2(varname);
					meshname2.append("_mesh");
					Eigen::MatrixXf pts = extras.second->block(0, 0, extras.second->rows(), 3);
					// Extract the values
					size_t ndims = extras.second->cols() - 3;
					Eigen::MatrixXf vals = extras.second->block(0, 3, extras.second->rows(), ndims);
					auto npm = h->file->createPointMesh<float>(meshname2.c_str(), pts, axislabels, axisunits);

					npm->writeData<float>(varname.c_str(), vals, "Unknown");
					//npm->writeData<float>(varname.c_str(), vals.data(), "Unknown");
				}
			}

			return h; // Pass back the handle
		}
		*/
	}
}

