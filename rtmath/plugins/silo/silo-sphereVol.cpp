/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <array>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#define DB_USE_MODERN_DTPTR
#include <silo.h>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/points.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/debug.h>
#include <Ryan_Debug/error.h>

#include "plugin-silo.h"
#include "../../related/rtmath_silo_cpp/WritePoints.h"


namespace Ryan_Debug {
	namespace registry {
		using std::shared_ptr;
		using rtmath::ddscat::ddOutput;
		using namespace rtmath::plugins::silo;


		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<rtmath::ddscat::points::sphereVol>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const boost::shared_ptr<const rtmath::ddscat::points::sphereVol> s)
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
					if (std::string(sh->getId()) != std::string(PLUGINID)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<silo_handle>(sh);
				}

				std::string meshname("Points_");
				auto sdata = s->getData();
				Eigen::MatrixXf lPts(sdata->rows(), sdata->cols());
				lPts = (*sdata).cast<float>();
				Eigen::MatrixXf meshPts(sdata->rows(), 3);
				meshPts = lPts.block(0,0,sdata->rows(), 3);
				Eigen::MatrixXi lVals(sdata->rows(), 1);
				lVals = sdata->block(0,3,sdata->rows(), 1);
				const char* axislabels[] = { "x", "y", "z" };
				const char* axisunits[] = { "dipoles", "dipoles", "dipoles" };

				std::string dielsName = meshname;
				dielsName.append("In_Sphere");
				std::string indexName = meshname;
				indexName.append("Point_IDs");
				auto pm = h->file->createPointMesh<float>(
					meshname.c_str(), meshPts, axislabels, axisunits);
				pm->writeData<int>(dielsName.c_str(), lVals.data(), "In_Sphere");
				// Create a 3d mesh, also for the dielectrics

				Eigen::Array3i mins = sdata->block(0,0,1,3).transpose(),
					maxs = sdata->block(sdata->rows()-1,0,1,3).transpose();
				// Doing this so that the ends of the shape do not get chopped off
				//mins -= 2 * Eigen::Array3i::Ones(); maxs += 2 * Eigen::Array3i::Ones();
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

				for (size_t i = 0; i < (size_t)sdata->rows(); ++i)
				{
					Eigen::Array3i a; a(0) = (int)(*sdata)(i, 0);
					a(1) = (int)(*sdata)(i, 1);
					a(2) = (int)(*sdata)(i, 2);
					int index = getIndex(a);
					mDiels(index, 0) = (float) (*sdata)(i, 3);
				}
				Eigen::VectorXf xs(span(0), 1), ys(span(1), 1), zs(span(2), 1);
				xs.setLinSpaced((*sdata)(0,0), (*sdata)(sdata->rows()-1,0));
				ys.setLinSpaced((*sdata)(0,1), (*sdata)(sdata->rows()-1,1));
				zs.setLinSpaced((*sdata)(0,2), (*sdata)(sdata->rows()-1,2));

				int dimsizes[] = { span(0), span(1), span(2) };
				const float *dims[] = { xs.data(), ys.data(), zs.data() };
				auto mesh = h->file->createRectilinearMesh<float>(
					"Sphere_Mesh",
					3, dims, dimsizes,
					axislabels, axisunits);
				// Write the array of zone ids
				mesh->writeData<float>("Shp_Mesh_Sphere_Bnds", mDiels.data(), "None");

				return h; // Pass back the handle
		}
	}
}
