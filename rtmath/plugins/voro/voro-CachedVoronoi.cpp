#pragma warning( disable : 4996 ) // crt fopen
#pragma warning( disable : 4244 ) // convertion from int64 to int
#define _CRT_SECURE_NO_WARNINGS
#include <boost/shared_ptr.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/managed_heap_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/flat_map.hpp>
#include <boost/interprocess/containers/flat_set.hpp>
#include <boost/interprocess/containers/set.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <functional>
#include <scoped_allocator>
#include <boost/functional/hash.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/unordered_set.hpp>
#include <Voro/voro++.hh>
#include <cmath>
//#include <Ryan_Serialization/serialization.h>
#include "../../rtmath/rtmath/depGraph.h"
//#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "voro-CachedVoronoi.h"
//#include "../../rtmath/rtmath/Voronoi/CachedVoronoi.h"
#include "../../rtmath/rtmath/error/error.h"
//#include "voro-Voronoi.h"

#undef min
#undef max

namespace rtmath
{
	namespace plugins {
		namespace voro {
			VoroCachedVoronoi::~VoroCachedVoronoi() {}
			VoroCachedVoronoi::VoroCachedVoronoi() {}

			VoroCachedVoronoi::VoroCachedVoronoi(boost::shared_ptr<CachedVoronoi> src,
				boost::shared_ptr<::voro::container> vc)
			{
				this->sa = src->surfaceArea();
				this->vol = src->volume();
				this->span = src->span;
				this->cellmap = src->cellmap;
				this->tblDoubles = src->tblDoubles;
				this->tblInts = src->tblInts;
				this->tblCellNeighs = src->tblCellNeighs;
				this->tblCellF_verts = src->tblCellF_verts;
				this->tblCellF_areas = src->tblCellF_areas;
				this->mins = src->mins;
				this->maxs = src->maxs;

				// Create voronoi container
				this->vc = vc;
			}

			VoroCachedVoronoi::VoroCachedVoronoi(size_t numPoints, 
				boost::shared_ptr<::voro::container> vc, 
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs) :
				CachedVoronoi(numPoints,mins,maxs),
				vc(vc)
			{
				if (vc) regenerateCache(numPoints);
			}

			void VoroCachedVoronoi::generateCellMap() const
			{
				if (cellmap.rows() > 0) return;
				if (!vc) return;
				// Defining the span as an inclusive bound
				Eigen::Array3i mins = this->mins.cast<int>(), maxs = this->maxs.cast<int>();
				// so that the ends of the shape do not get chopped off...
				// NOTE: if changed here, need to update silo-voronoi.cpp with it also
				// NOTE: no need to do this, as VisIt and ParaView do not support coloring on an existing contour, 
				//       so the meshes can differ.
				// NOTE: if changed, will break existing stored voronoi data :(
				//mins -= 2 * Eigen::Array3i::Ones(); maxs += 2 * Eigen::Array3i::Ones();

				span = maxs - mins + 1;
				int numBoxes = span.prod();
				cellmap.resize(numBoxes, 4);

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

				// Iterate over all integer combinations
				// Cannot parallelize since find_voronoi_cell is NOT thread-safe. Raises memory access violation exception.
				for (int i = 0; i < numBoxes; ++i)
				{
					Eigen::Array3i crd = getCoords(i);
					Eigen::Array3d crdd = crd.cast<double>();
					Eigen::Array3d pt;
					int cellId = 0;
					if (!vc->find_voronoi_cell(crdd(0), crdd(1), crdd(2), pt(0), pt(1), pt(2), cellId))
					{
						// Point could not be found
						// Should not happen
						cellId = -1;
					}
					cellmap(i, 3) = cellId;
					cellmap.block<1, 3>(i, 0) = crd;
				}
			}

			void VoroCachedVoronoi::regenerateCache(size_t numPoints)
			{
				if (!vc) return;
				// Iterate over cells and store cell information 
				// (prevents constant recalculations)
				using namespace boost::interprocess;

				// Cannot just use resize(numPoints) here because the appropriate allocators must be specified.
				//if (c.size() != numPoints)
				//	c.resize(numPoints);

				using namespace voro;
				::voro::voronoicell_neighbor n;
				::voro::c_loop_all cl(*(vc.get()));
				if (cl.start()) do if (vc->compute_cell(n, cl)) {
					// Quantities explicitly retrieved this way to avoid any c->at(id) potential issues.
					int id;
					Eigen::Matrix3d pos;
					double r;
					cl.pos(id, pos(0), pos(1), pos(2), r); // getting id directly into field would be problematic

					if (id % 1000 == 0) std::cerr << id << "\n";

					auto od = tblDoubles.block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(id, 0);
					auto oi = tblInts.block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(id, 0);

					// These are found from the iterator...
					oi(CachedVoronoi::ID) = id;
					od(CachedVoronoi::RADIUS) = r;
					od(CachedVoronoi::POS_X) = pos(0);
					od(CachedVoronoi::POS_Y) = pos(1);
					od(CachedVoronoi::POS_Z) = pos(2);

					// Get the rest of the parameters
					this->calcCell(n, id);

					vol += od(CachedVoronoi::VOLUME);
					sa += od(CachedVoronoi::SA_EXT);

				} while (cl.inc());
			}

			void VoroCachedVoronoi::calcCell(::voro::voronoicell_neighbor &vc, size_t _row)
			{
				auto od = tblDoubles.block<1, CachedVoronoi::NUM_CELL_DEFS_DOUBLES>(_row, 0);
				auto oi = tblInts.block<1, CachedVoronoi::NUM_CELL_DEFS_INTS>(_row, 0);

				Eigen::Matrix3d crds;
				// Need to copy vectors due to potentially different allocators
				std::vector<int> lneigh, lf_vert;
				std::vector<double> lf_areas;
				vc.neighbors(lneigh);
				vc.face_vertices(lf_vert);
				//vc.vertices(crds(0),crds(1),crds(2),v);
				vc.face_areas(lf_areas);

				const size_t cArraySize = CachedVoronoi_MaxNeighbors_VAL;

				auto neigh = tblCellNeighs.block<1, CachedVoronoi_MaxNeighbors_VAL>(_row, 0);
				auto f_vert = tblCellF_verts.block<1, CachedVoronoi_MaxNeighbors_VAL>(_row, 0);
				auto f_areas = tblCellF_areas.block<1, CachedVoronoi_MaxNeighbors_VAL>(_row, 0);

				for (size_t i = 0; i < std::min(cArraySize, lneigh.size()); ++i)
					neigh(i) = lneigh[i];
				for (size_t i = 0; i < std::min(cArraySize, lf_vert.size()); ++i)
					f_vert(i) = lf_vert[i];
				for (size_t i = 0; i < std::min(cArraySize, lf_areas.size()); ++i)
					f_areas(i) = lf_areas[i];

				od(CachedVoronoi::VOLUME) = vc.volume();
				od(CachedVoronoi::SA_FULL) = vc.surface_area();
				oi(CachedVoronoi::NFACES) = vc.number_of_faces();
				oi(CachedVoronoi::NEDGES) = vc.number_of_edges();
				vc.centroid(od(CachedVoronoi::CENTROID_X), od(CachedVoronoi::CENTROID_Y), od(CachedVoronoi::CENTROID_Z));

				od(CachedVoronoi::MAX_RADIUS_SQUARED) = vc.max_radius_squared();
				od(CachedVoronoi::TOTAL_EDGE_DISTANCE) = vc.total_edge_distance();

				od(CachedVoronoi::SA_EXT) = 0;
				for (int i = 0; i < lneigh.size(); ++i)
					if (lneigh[i] <= 0)
						od(CachedVoronoi::SA_EXT) += f_areas[i];
			}

		}
	}
}

