/// \brief Provides hdf5 file IO
#define _SCL_SECURE_NO_WARNINGS
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <array>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>

#include <boost/filesystem.hpp>

#include "../../rtmath/rtmath/defs.h"
//#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "plugin-hdf5.h"
#include <hdf5.h>
#include <H5Cpp.h>

namespace rtmath {
	namespace plugins {
		namespace hdf5 {


			/// \param base is the base to write the subgroups to. From here, "./Shape" is the root of the routine's output.
			std::shared_ptr<H5::Group> write_hdf5_voro(std::shared_ptr<H5::Group> base, 
				const rtmath::Voronoi::VoronoiDiagram *v)
			{
				using std::shared_ptr;
				using namespace H5;
				using namespace rtmath::Voronoi;


				shared_ptr<Group> shpraw = base;

				int fillvalue = -1;   /* Fill value for the dataset */
				DSetCreatPropList plist;
				plist.setFillValue(PredType::NATIVE_INT, &fillvalue);

				// Write out all of the generated diagrams
				std::map<std::string, VoronoiDiagram::matrixType> results;
				v->getResultsTable(results);
				for (const auto &res : results)
				{
					hsize_t fDims[] = { res.second->rows(), res.second->cols() };
					DataSpace fSpace(2, fDims);
					
					shared_ptr<DataSet> pts(new DataSet(shpraw->createDataSet(res.first,
						PredType::NATIVE_FLOAT, fSpace, plist)));
					Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> data = res.second->cast<float>();
					pts->write(data.data(), PredType::NATIVE_FLOAT);
				}

				// Write out all of the cell information
				shared_ptr<Group> gCells = openOrCreateGroup(shpraw, "Cells");

				addAttr<size_t, Group>(gCells, "NumCells", 0);
				addAttr<size_t, Group>(gCells, "NumSurfaceCells", 0);
				addAttr<size_t, Group>(gCells, "NumInteriorCells", 0);
				addAttr<size_t, Group>(gCells, "Volume", 0);

				shared_ptr<Group> gDom = openOrCreateGroup(gCells, "Domains");
				shared_ptr<Group> gSfc = openOrCreateGroup(gDom, "Surface");
				shared_ptr<Group> gInt = openOrCreateGroup(gDom, "Interior");

				/* Cell table contains information on the:
				 * - id
				 * - surface area
				 * - exterior surface area
				 * - surface fraction
				 * - centroid
				 * - anchor point
				 * - integer points within the cell
				 * - number of integer points within the cell
				*/

				return shpraw;
			}


		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;
		

		shared_ptr<IOhandler> 
			write_file_type_multi
			(shared_ptr<IOhandler> sh, const char* filename, 
			const rtmath::Voronoi::VoronoiDiagram *v, 
			const char* key, IOhandler::IOtype iotype)
		{
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename, iotype));
			} else {
				if (sh->getId() != PLUGINID) RTthrow debug::xDuplicateHook("Bad passed plugin");
				h = std::dynamic_pointer_cast<hdf5_handle>(sh);
			}

			// Check for the existence of the appropriate:
			// Group "Hashed"
			shared_ptr<Group> grpHashes = openOrCreateGroup(h->file, "Hashed");
			// Group "Hashed"/shp->hash
			shared_ptr<Group> grpShape = openOrCreateGroup(grpHashes, v->hash().string().c_str());
			// Group "Hashed"/shp->hash/"Voronoi". If it exists, overwrite it. There should be no hard links here.
			shared_ptr<Group> grpVoro = openOrCreateGroup(grpShape, "Voronoi");

			std::string skey(key);
			if (!skey.size()) skey = "Unknown";
			/// \note The unlink operation does not really free the space..... Should warn the user.
			if (groupExists(grpVoro, skey.c_str())) return h; //grpShape->unlink("Shape");
			shared_ptr<Group> grpVoroObj = openOrCreateGroup(grpVoro, skey.c_str());
			shared_ptr<Group> newbase = write_hdf5_voro(grpVoroObj, v);

			return h; // Pass back the handle
		}

	}
}
