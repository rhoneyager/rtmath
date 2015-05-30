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
#include <Ryan_Debug/hash.h>
#include <Ryan_Debug/error.h>

#include "../../related/rtmath_hdf5_cpp/export-hdf5.h"
#include "plugin-hdf5.h"
#include <hdf5.h>
#include <H5Cpp.h>

#include "cmake-settings.h"

namespace rtmath {
	namespace plugins {
		namespace hdf5 {


			/// \param base is the base to write the subgroups to. From here, "./Shape" is the root of the routine's output.
			std::shared_ptr<H5::Group> read_hdf5_voro(std::shared_ptr<H5::Group> base,
				std::shared_ptr<Ryan_Debug::registry::IO_options> opts,
				boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram > s)
			{
				using std::shared_ptr;
				using std::string;
				using namespace H5;
				using namespace rtmath::Voronoi;

				//shared_ptr<Group> base(new Group(g->createGroup("Voronoi")));

				readAttr<string, Group>(base, "ingest_timestamp", s->ingest_timestamp);
				readAttr<string, Group>(base, "hostname", s->hostname);
				readAttr<string, Group>(base, "ingest_username", s->ingest_username); // Not all ingests have this...
				readAttr<int, Group>(base, "ingest_rtmath_version", s->ingest_rtmath_version);
				if (attrExists(base, "pluginId"))
					readAttr<string, Group>(base, "pluginId", s->pluginId);

				Ryan_Debug::hash::HASH_t hash;
				readAttr<uint64_t, Group>(base, "Hash_lower", hash.lower);
				readAttr<uint64_t, Group>(base, "Hash_upper", hash.upper);
				s->setHash(hash);

				readAttrEigen<Eigen::Array3f, Group>(base, "mins", s->mins);
				readAttrEigen<Eigen::Array3f, Group>(base, "maxs", s->maxs);
				// span is a derived value. Read not used.

				// Store the source matrix
				boost::shared_ptr<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > src(new Eigen::MatrixXf);
				readDatasetEigen(base, "Source", *(src));
				s->src = src;

				// Store the precalced objects
				shared_ptr<Group> grpcache(openGroup(base, "Voronoi_Cache"));
				hsize_t sz = grpcache->getNumObjs();
				for (hsize_t i = 0; i < sz; ++i)
				{
					std::string hname = grpcache->getObjnameByIdx(i);
					H5G_obj_t t = grpcache->getObjTypeByIdx(i);
					if (t != H5G_obj_t::H5G_GROUP) continue;

					shared_ptr<Group> grp(openGroup(grpcache, hname.c_str()));

					boost::shared_ptr<CachedVoronoi> vobj(new CachedVoronoi);


					// Store the cell map
					readDatasetEigen(grp, "Cell_Map", *(vobj->cellmap));


					readDatasetEigen(grp, "tblDoubles", *(vobj->tblDoubles));
					readDatasetEigen(grp, "tblInts", *(vobj->tblInts));


					readDatasetEigen(grp, "tblCellNeighs", *(vobj->tblCellNeighs));
					readDatasetEigen(grp, "tblCellF_verts", *(vobj->tblCellF_verts));
					readDatasetEigen(grp, "tblCellF_areas", *(vobj->tblCellF_areas));
					readAttrEigen<Eigen::Array3f, Group>(grp, "mins", vobj->mins);
					readAttrEigen<Eigen::Array3f, Group>(grp, "maxs", vobj->maxs);
					readAttrEigen<Eigen::Array3i, Group>(grp, "span", vobj->span);

					double sa, vol;
					readAttr<double, Group>(grp, "surfaceArea", sa);
					readAttr<double, Group>(grp, "volume", vol);
					vobj->surfaceArea(sa);
					vobj->volume(vol);

					// Store object in the cache
					s->cache[hname] = vobj;
				}

				// Store the results tables
				std::map<std::string, VoronoiDiagram::matrixType> results;
				s->getResultsTable(results);
				shared_ptr<Group> grpres(openGroup(base, "Results"));
				sz = grpres->getNumObjs();
				// Write out all of the generated diagrams
				//for (const auto &res : results)
				//{
				for (hsize_t i = 0; i < sz; ++i)
				{
					std::string hname = grpres->getObjnameByIdx(i);
					H5G_obj_t t = grpres->getObjTypeByIdx(i);
					if (t != H5G_obj_t::H5G_DATASET) continue;

					VoronoiDiagram::matrixTypeMutable res(new Eigen::MatrixXf);
					Eigen::MatrixXf resb;
					readDatasetEigen(grpres, hname.c_str(), resb);
					res->resize(resb.rows(), resb.cols() + 3);
					res->block(0, 3, resb.rows(), resb.cols()) = resb;
					// First part of table matches the point coordinates (for objects with matching sizes)
					if (s->src->rows() == resb.rows())
						res->block(0, 0, resb.rows(), 3) = *(s->src);
					// Otherwise, the loaded columns represent a row mapping (see calcCandidateConvexHullPoints),
					// so pull out the indexed row coordinates
					else {
						for (size_t r = 0; r < (size_t)resb.rows(); ++r)
						{
							res->block<1, 3>(r, 0) = src->block<1, 3>((int)resb(r, 0), 0);
						}
					}


					s->results[hname] = res;
				}

				return base;
			}


		}
	}

}
namespace Ryan_Debug {
	namespace registry
	{
		using std::shared_ptr;
		using namespace rtmath::plugins::hdf5;


		template<>
		shared_ptr<IOhandler>
			read_file_type_multi<rtmath::Voronoi::VoronoiDiagram>
			(shared_ptr<IOhandler> sh, shared_ptr<IO_options> opts,
			boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram > s,
			std::shared_ptr<const Ryan_Debug::registry::collectionTyped<rtmath::Voronoi::VoronoiDiagram> >)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string hash = opts->getVal<std::string>("hash");
			std::string key = opts->getVal<std::string>("key", "standard");
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h = registry::construct_handle
				<registry::IOhandler, hdf5_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
				new hdf5_handle(filename.c_str(), iotype)); });
			

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (!grpHashes) RDthrow(Ryan_Debug::error::xMissingKey())
				<< Ryan_Debug::error::key("Hashed")
				<< Ryan_Debug::error::hash(key);
			shared_ptr<Group> grpHash = openGroup(grpHashes, hash.c_str());
			if (!grpHash) RDthrow(Ryan_Debug::error::xMissingHash())
				<< Ryan_Debug::error::key("Hashed")
				<< Ryan_Debug::error::hash(key);
			shared_ptr<Group> grpVoro = openGroup(grpHash, "Voronoi");
			if (!grpVoro) RDthrow(Ryan_Debug::error::xMissingKey())
				<< Ryan_Debug::error::key("Voronoi")
				<< Ryan_Debug::error::hash(key);
			shared_ptr<Group> grpKey = openGroup(grpVoro, key.c_str());
			if (!grpKey) RDthrow(Ryan_Debug::error::xMissingKey())
				<< Ryan_Debug::error::key(key);
			read_hdf5_voro(grpKey, opts, s);

			return h;
		}

		template<>
		std::shared_ptr<IOhandler>
			read_file_type_vector<rtmath::Voronoi::VoronoiDiagram>
			(std::shared_ptr<IOhandler> sh, std::shared_ptr<IO_options> opts,
			std::vector<boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram> > &s,
			std::shared_ptr<const Ryan_Debug::registry::collectionTyped<rtmath::Voronoi::VoronoiDiagram> > filter)
		{
			std::string filename = opts->filename();
			IOhandler::IOtype iotype = opts->getVal<IOhandler::IOtype>("iotype", IOhandler::IOtype::READONLY);
			//IOhandler::IOtype iotype = opts->iotype();
			std::string key = opts->getVal<std::string>("key", ""); // okay to leave blank here
			using std::shared_ptr;
			using namespace H5;
			Exception::dontPrint();
			std::shared_ptr<hdf5_handle> h = registry::construct_handle
				<registry::IOhandler, hdf5_handle>(
				sh, PLUGINID, [&](){return std::shared_ptr<hdf5_handle>(
				new hdf5_handle(filename.c_str(), iotype)); });

			shared_ptr<Group> grpHashes = openGroup(h->file, "Hashed");
			if (grpHashes)
			{
				hsize_t sz = grpHashes->getNumObjs();
				//s.reserve(s.size() + sz);
				for (hsize_t i = 0; i < sz; ++i)
				{
					std::string hname = grpHashes->getObjnameByIdx(i);
					H5G_obj_t t = grpHashes->getObjTypeByIdx(i);
					if (t != H5G_obj_t::H5G_GROUP) continue;

					shared_ptr<Group> grpHash = openGroup(grpHashes, hname.c_str());
					if (!grpHash) continue; // Should never happen
					shared_ptr<Group> grpVoro = openGroup(grpHash, "Voronoi");
					if (!grpVoro) continue;

					hsize_t rz = grpVoro->getNumObjs();
					for (hsize_t i = 0; i < sz; ++i)
					{
						std::string runname = grpVoro->getObjnameByIdx(i);
						H5G_obj_t t = grpVoro->getObjTypeByIdx(i);
						if (t != H5G_obj_t::H5G_GROUP) continue;
						if (key.size() && key != runname) continue;
						shared_ptr<H5::Group> grpRun = openGroup(grpVoro, runname.c_str());
						if (!grpRun) continue;

						boost::shared_ptr<rtmath::Voronoi::VoronoiDiagram>
							run(new rtmath::Voronoi::VoronoiDiagram);
						read_hdf5_voro(grpRun, opts, run);
						if (filter) {
							if (filter->filter(run.get()))
								s.push_back(run);
						}
						else s.push_back(run);
					}
				}
			}

			return h;
		}
	}
}
