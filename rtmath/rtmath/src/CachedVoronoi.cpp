#include "Stdafx-voronoi.h"
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
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/unordered_set.hpp>
#include <Voro/voro++.hh>
#include <Ryan_Serialization/serialization.h>
#include "../rtmath/depGraph.h"
#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/Voronoi/CachedVoronoi.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace Voronoi {
		CachedVoronoi::~CachedVoronoi() {}

		void CachedVoronoi::resize(size_t n)
		{
			tblDoubles.resize(n, NUM_CELL_DEFS_DOUBLES);
			tblInts.resize(n, NUM_CELL_DEFS_INTS);
			tblCellNeighs.resize(n, CachedVoronoi_MaxNeighbors::value);
			tblCellF_verts.resize(n, CachedVoronoi_MaxNeighbors::value);
			tblCellF_areas.resize(n, CachedVoronoi_MaxNeighbors::value);

			tblCellNeighs.setZero();
			tblCellF_verts.setZero();
			tblCellF_areas.setZero();

		}

		CachedVoronoi::CachedVoronoi(size_t numPoints, boost::shared_ptr<voro::container> vc, const Eigen::Array3f &mins, const Eigen::Array3f &maxs) :
			vc(vc),
			sa(0),
			vol(0),
			mins(mins),
			maxs(maxs)
		{
			resize(numPoints);
			if (vc) regenerateCache(numPoints);
		}

		void CachedVoronoi::generateCellMap() const
		{
			// Defining the span as an inclusive bound
			Eigen::Array3i mins = this->mins.cast<int>(), maxs = this->maxs.cast<int>();
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
				y = (i - (x*span(0)*span(1))) / span(0);
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


		void CachedVoronoi::regenerateCache(size_t numPoints)
		{
			if (!vc) return;
			// Iterate over cells and store cell information 
			// (prevents constant recalculations)
			using namespace boost::interprocess;

			// Cannot just use resize(numPoints) here because the appropriate allocators must be specified.
			//if (c.size() != numPoints)
			//	c.resize(numPoints);

			using namespace voro;
			voronoicell_neighbor n;
			c_loop_all cl(*(vc.get()));
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
				calcCell(n, id);

				vol += od(CachedVoronoi::VOLUME);
				sa += od(CachedVoronoi::SA_EXT);

			} while (cl.inc());
		}


		void CachedVoronoi::calcCell(voro::voronoicell_neighbor &vc, size_t _row)
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

			const size_t cArraySize = CachedVoronoi_MaxNeighbors::value;

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

