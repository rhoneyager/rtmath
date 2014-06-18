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
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/Voronoi/CachedVoronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"
#include "../../rtmath/rtmath/error/error.h"

#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "plugin-hdf5.h"
#include <hdf5.h>
#include <H5Cpp.h>

namespace rtmath {
	namespace plugins {
		namespace hdf5 {


			/// \param base is the base to write the subgroups to. From here, "./Shape" is the root of the routine's output.
			std::shared_ptr<H5::Group> write_hdf5_voro(std::shared_ptr<H5::Group> base, 
				std::shared_ptr<rtmath::registry::IO_options> opts,
				const rtmath::Voronoi::VoronoiDiagram *s)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace rtmath::Voronoi;

				//shared_ptr<Group> base(new Group(g->createGroup("Voronoi")));

				addAttr<string, Group>(base, "ingest_timestamp", s->ingest_timestamp);
				addAttr<string, Group>(base, "ingest_hostname", s->ingest_hostname);
				addAttr<string, Group>(base, "ingest_username", s->ingest_username); // Not all ingests have this...
				addAttr<int, Group>(base, "ingest_rtmath_version", s->ingest_rtmath_version);
				addAttr<string, Group>(base, "hostname", s->hostname);

				addAttr<uint64_t, Group>(base, "Hash_lower", s->hash().lower);
				addAttr<uint64_t, Group>(base, "Hash_upper", s->hash().upper);
				addAttr<size_t, Group>(base, "Num_Points", s->numPoints());

				Eigen::Array3f mins, maxs, span;
				s->getBounds(mins, maxs, span);
				addAttrEigen<Eigen::Array3f, Group>(base, "mins", mins);
				addAttrEigen<Eigen::Array3f, Group>(base, "maxs", maxs);
				addAttrEigen<Eigen::Array3f, Group>(base, "span", span);

				addAttr<double, Group>(base, "surfaceArea", s->surfaceArea());
				addAttr<double, Group>(base, "volume", s->volume());

				// Enable compression
				hsize_t chunk_colwise[2] = { (hsize_t)s->numPoints(), 1 };
				hsize_t chunk_cell_doubles[2] = { (hsize_t)s->numPoints(), CachedVoronoi::NUM_CELL_DEFS_DOUBLES };
				hsize_t chunk_cell_ints[2] = { (hsize_t)s->numPoints(), CachedVoronoi::NUM_CELL_DEFS_INTS };
				hsize_t chunk_cell_CachedVoronoi_MaxNeighbors_VAL[2] = { (hsize_t)s->numPoints(), CachedVoronoi_MaxNeighbors_VAL };
				auto plistCol = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistCol->setChunk(2, chunk_colwise);
				auto plistDoubles = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistDoubles->setChunk(2, chunk_cell_doubles);
				auto plistInts = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistInts->setChunk(2, chunk_cell_ints);
				auto plistArray = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
				plistArray->setChunk(2, chunk_cell_CachedVoronoi_MaxNeighbors_VAL);
				//int fillvalue = -1;
				//plistCol->setFillValue(PredType::NATIVE_INT, &fillvalue);
#if COMPRESS_ZLIB
				plistCol->setDeflate(6);
				plistDoubles->setDeflate(6);
				plistInts->setDeflate(6);
				plistArray->setDeflate(6);
#endif

				// Store the source matrix
				addDatasetEigen(base, "Source", *(s->src), plistCol);

				// Store the precalced objects
				shared_ptr<Group> grpcache(new Group(base->createGroup("Voronoi_Cache")));
				for (const auto& e : s->cache)
				{
					shared_ptr<Group> grp(new Group(grpcache->createGroup(e.first)));
					// Store the cell map
					auto cellmap = e.second->getCellMap();
					addDatasetEigen(grp, "Cell_Map", *cellmap, plistCol);

					addDatasetEigen(grp, "tblDoubles", e.second->tblDoubles, plistDoubles);
					addDatasetEigen(grp, "tblInts", e.second->tblInts, plistInts);
					addDatasetEigen(grp, "tblCellNeighs", e.second->tblCellNeighs, plistArray);
					addDatasetEigen(grp, "tblCellF_verts", e.second->tblCellF_verts, plistArray);
					addDatasetEigen(grp, "tblCellF_areas", e.second->tblCellF_areas, plistArray);
					addAttrEigen<Eigen::Array3f, Group>(grp, "mins", e.second->mins);
					addAttrEigen<Eigen::Array3f, Group>(grp, "maxs", e.second->maxs);
					addAttrEigen<Eigen::Array3i, Group>(grp, "span", e.second->span);

					addAttr<double, Group>(grp, "surfaceArea", e.second->surfaceArea());
					addAttr<double, Group>(grp, "volume", e.second->volume());
				}

				// Store the results tables
				std::map<std::string, VoronoiDiagram::matrixType> results;
				s->getResultsTable(results);
				shared_ptr<Group> grpres(new Group(base->createGroup("Results")));
				// Write out all of the generated diagrams
				for (const auto &res : results)
				{
					hsize_t fDims[] = { (hsize_t) res.second->rows(), (hsize_t) res.second->cols() };
					DataSpace fSpace(2, fDims);
					auto plist = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
					plist->setChunk(2, fDims);
#if COMPRESS_ZLIB
					plist->setDeflate(6);
#endif
					addDatasetEigen(grpres, res.first.c_str(), *(res.second.get()), plist);
				}

				/*
				// Write out all of the cell information
				shared_ptr<Group> gCells = openOrCreateGroup(shpraw, "Cells");

				addAttr<size_t, Group>(gCells, "NumCells", 0);
				addAttr<size_t, Group>(gCells, "NumSurfaceCells", 0);
				addAttr<size_t, Group>(gCells, "NumInteriorCells", 0);
				addAttr<size_t, Group>(gCells, "Volume", 0);

				shared_ptr<Group> gDom = openOrCreateGroup(gCells, "Domains");
				shared_ptr<Group> gSfc = openOrCreateGroup(gDom, "Surface");
				shared_ptr<Group> gInt = openOrCreateGroup(gDom, "Interior");
				*/

				return base;
			}


		}
	}

	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;
		

		template<>
		shared_ptr<IOhandler> 
			write_file_type_multi<rtmath::Voronoi::VoronoiDiagram>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts, 
			const rtmath::Voronoi::VoronoiDiagram *v)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", "");

			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h;
			if (!sh)
			{
				// Access the hdf5 file
				h = std::shared_ptr<hdf5_handle>(new hdf5_handle(filename.c_str(), iotype));
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

			write_hdf5_voro(grpVoro, opts, v);

			return h; // Pass back the handle
		}

	}
}
