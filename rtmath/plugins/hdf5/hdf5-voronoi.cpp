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

#include "cmake-settings.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {

			template <class T>
			void stringifyArray(size_t rows, size_t mxcols, T &obj, std::vector<std::string> &res, std::vector<const char*> &resb, char sep = ',')
			{
				res.resize(rows);
				resb.resize(rows);
				for (size_t i = 0; i < rows; ++i)
				{
					std::ostringstream out;
					size_t lastcol = 0;
					for (size_t j = 0; j < mxcols; ++j)
					{
						if (obj(i, j) != 0)
						{
							// Output everything from the previous mark to the present
							for (size_t c = lastcol; c < j; ++c)
							{
								out << obj(i, c) << sep;
							}
							lastcol = j;
						}
					}
					out << obj(i, lastcol);

					res[i] = out.str();
					resb[i] = res[i].c_str();
				}
			}


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
				addAttr<string, Group>(base, "hostname", s->hostname);
				addAttr<string, Group>(base, "ingest_username", s->ingest_username); // Not all ingests have this...
				addAttr<int, Group>(base, "ingest_rtmath_version", s->ingest_rtmath_version);
				addAttr<string, Group>(base, "pluginId", s->pluginId);

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
				auto make_plist = [](size_t rows, size_t cols)
				{
					hsize_t chunk[2] = { (hsize_t)rows, (hsize_t)cols };
					auto plist = std::shared_ptr<DSetCreatPropList>(new DSetCreatPropList);
					plist->setChunk(2, chunk);
#if COMPRESS_ZLIB
					plist->setDeflate(6);
#endif
					return plist;
				};

				// Store the source matrix
				addDatasetEigen(base, "Source", *(s->src), make_plist((size_t)s->src->rows(), (size_t)s->src->cols()));

				// Store the precalced objects
				shared_ptr<Group> grpcache(new Group(base->createGroup("Voronoi_Cache")));
				for (const auto& e : s->cache)
				{
					shared_ptr<Group> grp(new Group(grpcache->createGroup(e.first)));
					// Store the cell map
					auto cellmap = e.second->getCellMap();

					addDatasetEigen(grp, "Cell_Map", *cellmap, make_plist((size_t)cellmap->rows(), (size_t)cellmap->cols()));


					addDatasetEigen(grp, "tblDoubles", e.second->tblDoubles, make_plist((size_t)e.second->tblDoubles.rows(), (size_t)e.second->tblDoubles.cols()));
					addDatasetEigen(grp, "tblInts", e.second->tblInts, make_plist((size_t)e.second->tblInts.rows(), (size_t)e.second->tblInts.cols()));

					/*
					std::vector<std::string> res;
					std::vector<const char*> resb;

					stringifyArray((size_t)e.second->tblCellNeighs.rows(), (size_t)e.second->tblCellNeighs.cols(), e.second->tblCellNeighs, res, resb);
					addDatasetArray<const char*, Group>(grp, "tblCellNeighs", resb.size(), 1, resb.data());

					stringifyArray((size_t)e.second->tblCellF_verts.rows(), (size_t)e.second->tblCellF_verts.cols(), e.second->tblCellF_verts, res, resb);
					addDatasetArray<const char*, Group>(grp, "tblCellF_verts", resb.size(), 1, resb.data());

					stringifyArray((size_t)e.second->tblCellF_areas.rows(), (size_t)e.second->tblCellF_areas.cols(), e.second->tblCellF_areas, res, resb);
					addDatasetArray<const char*, Group>(grp, "tblCellF_areas", resb.size(), 1, resb.data());
					*/

					addDatasetEigen(grp, "tblCellNeighs", e.second->tblCellNeighs, make_plist((size_t)e.second->tblCellNeighs.rows(), (size_t)e.second->tblCellNeighs.cols()));
					addDatasetEigen(grp, "tblCellF_verts", e.second->tblCellF_verts, make_plist((size_t)e.second->tblCellF_verts.rows(), (size_t)e.second->tblCellF_verts.cols()));
					addDatasetEigen(grp, "tblCellF_areas", e.second->tblCellF_areas, make_plist((size_t)e.second->tblCellF_areas.rows(), (size_t)e.second->tblCellF_areas.cols()));
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
					auto resb = res.second->block(0, 3, res.second->rows(), res.second->cols() - 3);
					addDatasetEigen(grpres, res.first.c_str(), resb, make_plist((size_t)resb.rows(), (size_t)resb.cols()));
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
			std::string key = opts->getVal<std::string>("key", "standard");

			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h = registry::construct_handle
				<registry::IOhandler, hdf5_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
				new hdf5_handle(filename.c_str(), iotype)); });

			// Check for the existence of the appropriate:
			// Group "Hashed"
			shared_ptr<Group> grpHashes = openOrCreateGroup(h->file, "Hashed");
			// Group "Hashed"/shp->hash
			shared_ptr<Group> grpShape = openOrCreateGroup(grpHashes, v->hash().string().c_str());
			// Group "Hashed"/shp->hash/"Voronoi". If it exists, overwrite it. There should be no hard links here.
			shared_ptr<Group> grpVoro = openOrCreateGroup(grpShape, "Voronoi");

			shared_ptr<Group> grp = openOrCreateGroup(grpVoro, key.c_str());
			write_hdf5_voro(grp, opts, v);

			return h; // Pass back the handle
		}

	}
}
