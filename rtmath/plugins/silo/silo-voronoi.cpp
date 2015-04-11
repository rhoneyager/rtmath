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

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-silo.h"
#include "../../related/rtmath_silo_cpp/WritePoints.h"

#define DB_USE_MODERN_DTPTR
#include <silo.h>


namespace rtmath {
	namespace registry {
		using std::shared_ptr;
		using namespace rtmath::Voronoi;
		using namespace rtmath::plugins::silo;


		template<>
		shared_ptr<IOhandler>
			write_file_type_multi<rtmath::Voronoi::VoronoiDiagram>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts, 
			const boost::shared_ptr<const rtmath::Voronoi::VoronoiDiagram > v)
		{
				std::string filename = opts->filename();
				IOhandler::IOtype iotype = opts->iotype();

				using std::shared_ptr;
				std::shared_ptr<silo_handle> h;
				if (!sh)
				{
					h = std::shared_ptr<silo_handle>(new silo_handle(filename.c_str(), iotype));
				}
				else {
					if (sh->getId() != PLUGINID) RTthrow(debug::xDuplicateHook());
					h = std::dynamic_pointer_cast<silo_handle>(sh);
				}

				
				std::string meshname("Region_");
				//if (key)
				//	meshname.append(std::string(key));
				//else meshname.append(s->filename);

				auto cellmap = v->getCellMap();
				std::map<std::string, VoronoiDiagram::matrixType> tbl;
				v->getResultsTable(tbl);

				// Create a common mesh for all of the possible points
				// The cellmap contains all of the possible points.


				const char* axislabels[] = { "x", "y", "z" };
				//const char* axislabels2[] = { "z", "y", "x" };
				const char* axisunits[] = { "dipoles", "dipoles", "dipoles" };
				Eigen::Array3f minsf, maxsf, spanf;
				v->getBounds(minsf, maxsf, spanf);
				Eigen::Array3i mins = minsf.cast<int>(), maxs = maxsf.cast<int>(),
					span = spanf.cast<int>();

				Eigen::VectorXf xs(span(0), 1), ys(span(1), 1), zs(span(2), 1);
				xs.setLinSpaced(minsf(0), maxsf(0));
				ys.setLinSpaced(minsf(1), maxsf(1));
				zs.setLinSpaced(minsf(2), maxsf(2));

				int dimsizes[] = { span(0), span(1), span(2) };
				// Casting to a float because the silo format requires float or double quad mesh coordinates
				Eigen::MatrixXf grid = cellmap->cast<float>();
				const float *dims[] = { xs.data(), ys.data(), zs.data() };
				
				// Point mesh --- for debug visualization
				auto meshp = h->file->createPointMesh<float>("Region_CellIDs_points_mesh",
					cellmap->cast<float>().block(0,0,cellmap->rows(),3), axislabels, axisunits);
				meshp->writeData<float>("Region_CellIDs_points", grid.col(3).data(), "None");

				// Rectilinear space mesh
				auto mesh = h->file->createRectilinearMesh<float>(
					meshname.c_str(),
					3, dims, dimsizes, 
					axislabels, axisunits);
				// Write the array of zone ids
				mesh->writeData<float>("Region_CellIDs", grid.col(3).data(), "None");
				


				// For each of the arrays in tbl, attempt to match each to the relevent 
				// cell listing.
				for (const auto &t : tbl)
				{
					const std::string name = t.first;
					// Make a copy of the output matrix
					VoronoiDiagram::matrixTypeMutable m(new Eigen::MatrixXf(*(t.second)));
					// Only write the matrices that have the correct number of rows
					if ((size_t) m->rows() != v->numPoints()) continue;
					// If m has 4 or more cols, then the first three are the point location.
					// If it has 3 or fewer, then the point ids are the row numbers, sequentially.
					size_t numCols = (m->cols() > 3) ? m->cols() - 3 : m->cols();
					size_t idcol = (m->cols() > 3) ? 3 : 0;

					Eigen::MatrixXf res(cellmap->rows(), numCols);
					// It helps to construct a map between the probe point id and the matched point id.
					// For the cellMap table, the row number is the probe point id. The fourth column is 
					// the voronoi point id (which is the row of matrix m).
					for (int i=0; i < cellmap->rows(); ++i)
					{
						int probeId = i;
						int voroId = (*cellmap)(i,idcol);
						if (voroId >= 0)
							res.block(i,0,1,numCols) = m->block(voroId,idcol,1,numCols);
						else res.block(i,0,1,numCols).fill(-1);
					}

					mesh->writeData<float>(name.c_str(), res);
				}
				

				return h; // Pass back the handle
		}


	}
}
