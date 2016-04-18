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
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
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
			write_file_type_multi<rtmath::ddscat::shapefile::shapefile>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			const boost::shared_ptr<const rtmath::ddscat::shapefile::shapefile> s)
		{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				using std::shared_ptr;
				std::shared_ptr<silo_handle> h;
				if (!sh)
					h = std::shared_ptr<silo_handle>(new silo_handle(filename.c_str(), iotype));
				else {
					if (std::string(sh->getId()) != std::string(PLUGINID)) RDthrow(Ryan_Debug::error::xDuplicateHook());
					h = std::dynamic_pointer_cast<silo_handle>(sh);
				}

				std::string meshname("Points");
				Eigen::MatrixXf lPts(s->latticePts.rows(), s->latticePts.cols());
				lPts = s->latticePts;
				const char* axislabels[] = { "x", "y", "z" };
				const char* axisunits[] = { "dipoles", "dipoles", "dipoles" };

				std::string dielsName = meshname;
				dielsName.append("_Dielectrics");
				std::string indexName = meshname;
				indexName.append("_Point_IDs");
				//auto pm = h->file->createPointMesh<float>(meshname.c_str(), lPts, axislabels, axisunits);

				// meshPadding adds N cells before and after mins and maxs.
				// Useful when adding colocated mesh information.
				int meshPadding = opts->getVal<int>("meshPadding", 2);

				// Create a 3d mesh, also for the dielectrics
				Eigen::Array3i mins = s->mins.cast<int>(), maxs = s->maxs.cast<int>();
				// Doing this so that the ends of the shape do not get chopped off
				mins -= meshPadding * Eigen::Array3i::Ones();
				maxs += meshPadding * Eigen::Array3i::Ones();
				Eigen::Array3i span = maxs - mins + 1;
				int meshSize = span.prod();
				std::cerr << "Writing silo file with mins " << mins.transpose()
					<< " maxs " << maxs.transpose() << " span " << span.transpose() << std::endl;
				auto getCoords = [&](int i)->Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>
				{
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> crd;
					crd.resize(1,3);
					// Iterate first over z, then y, then x
					crd(0) = i % span(0);
					crd(2) = i / (span(0)*span(1));
					crd(1) = (int) (i - (crd(2)*span(0)*span(1))) / span(0); // it's not (i - i), as x involves an INTEGER division!
					crd(0) = crd(0) + mins(0);
					crd(1) = crd(1) + mins(1);
					crd(2) = crd(2) + mins(2); //+= mins;
					return crd;
				};
				auto getIndex = [&](Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> i) -> int
				{
					// Iterate first over z, then y, then x
					int res = 0;
					i(0) = i(0) - mins(0);
					i(1) = i(1) - mins(1);
					i(2) = i(2) - mins(2);
					//int ip = i(0);
					//i(0) = i(2); i(2) = ip;
					//i -= mins;
					//res = span(2) * span(1) * i(0);
					//res += span(2) * i(1);
					//res += i(2);
					res = span(0) * span(1) * i(2);
					res += span(0) * i(1);
					res += i(0);
					return res;
				};

				Eigen::VectorXf xs(span(0), 1), ys(span(1), 1), zs(span(2), 1);
				xs.setLinSpaced(mins(0), maxs(0));
				ys.setLinSpaced(mins(1), maxs(1));
				zs.setLinSpaced(mins(2), maxs(2));

				int dimsizes[] = { span(0), span(1), span(2) };
				const float *dims[] = { xs.data(), ys.data(), zs.data() };
				auto mesh = h->file->createRectilinearMesh<float>(
					"Shp_Mesh",
					3, dims, dimsizes,
					axislabels, axisunits);

				auto writeMeshData = [&](
					const char* varName,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &coords,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &indata,
					const char* varUnits) {
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> data(meshSize, indata.cols());
					data.setZero();
					int j = 0;
					std::cerr << "\tWriting mesh " << varName << std::endl;
					std::cerr << "\t\tHas " << coords.rows() << " coords, "
						"and " << indata.rows() << " points" << std::endl;
					if (coords.rows() != indata.rows()) {
						std::cerr << "\t\tMISMATCH. SKIPPING." << std::endl;
						return;
					}
					//std::cerr << "\tHas min " << mins.transpose() << " with span "
					//	<< span.transpose() << std::endl;
					for (int i=0; i < indata.rows(); ++i)
					{
						Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> a, b;
						a.resize(1,3); b.resize(1,3);
						a = coords.block(i,0,1,3);
						int index = getIndex(a);
						b = getCoords(index);
						//std::cerr << i << " " << a << " -> "
						//	<< index << " -> " << b << std::endl;
						if ((index < 0) || (index >= meshSize)) {
							std::cerr << "Silo mesh is too small for data size "
								<< "for variable " << varName << std::endl;
							std::cerr << "Mins " << mins.transpose()
								<< " Maxs " << maxs.transpose()
								<< " Span " << span.transpose()
								<< " prod " << meshSize << std::endl;
							std::cerr << "Failed at point i " << i << ": " << a
								<< " with index " << index << std::endl;
							return;
						}
						++j;
						//data.block(i,3,1,indata.cols()-3)
						//	= indata.block(i,3,1,indata.cols()-3);
						data.block(index,0,1,indata.cols())
							= indata.block(i,0,1,indata.cols());
					}
					if (meshSize < j) {
						std::cerr << "Silo mesh data write failed for variable "
							<< varName << std::endl;
						std::cerr << "MeshSize " << meshSize << " != j " << j << std::endl;
						return;
					}
					mesh->writeData<float>(varName, data, varUnits);
				};

				writeMeshData("Shp_Mesh_Dielectrics3",
					s->latticePts, s->latticePtsRi, "None");
				writeMeshData("Shp_Mesh_Dielectrics",
					s->latticePts,
					s->latticePtsRi.block(0,0,s->numPoints,1), "None");
				//double fScale = opts->getVal<double>("dielScalingFactor", 1.);
				//Eigen::ArrayXf mDielsScaled =
				//	s->latticePtsRi.block(0,0,s->numPoints,1) / fScale;
				//writeMeshData("Shp_Mesh_Scaled",
				//	s->latticePts, mDielsScaled, "None");
				writeMeshData("Shp_Mesh_Indices",
					s->latticePts, s->latticeIndex.cast<float>(), "None");

				// Write the extra matrices
				for (const auto &extras : s->latticeExtras)
				{
					std::string varname = meshname;
					varname.append("_");
					varname.append(extras.first);
					std::cerr << "Writing supplemental mesh " << varname << std::endl;
					std::cerr << "\thas " << extras.second->rows() << " rows and "
						<< extras.second->cols() << " columns." << std::endl;
					if (extras.second->cols() < 4) {
						writeMeshData(varname.c_str(),
							s->latticePts,
							*(extras.second.get()), "Unknown");
						//pm->writeData<float>(varname.c_str(), extras.second->data(), "Unknown");
					} else {
						Eigen::MatrixXf pts = extras.second->block(0, 0, extras.second->rows(), 3);
						// Extract the values
						size_t ndims = extras.second->cols() - 3;
						Eigen::MatrixXf vals = extras.second->block(0, 3, extras.second->rows(), ndims);
						writeMeshData(varname.c_str(),
							pts, vals, "Unknown");
						//auto npm = h->file->createPointMesh<float>(meshname2.c_str(), pts, axislabels, axisunits);
						//npm->writeData<float>(varname.c_str(), vals, "Unknown");
					}
				}

				return h; // Pass back the handle
		}


	}
}
