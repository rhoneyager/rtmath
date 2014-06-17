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
#include "../rtmath/Voronoi/CachedVoronoiCell.h"
#include "../rtmath/Voronoi/CachedVoronoi.h"
#include "../rtmath/error/error.h"

namespace rtmath
{
	namespace Voronoi {
		CachedVoronoi::~CachedVoronoi() {}

		CachedVoronoi::CachedVoronoi(size_t numPoints, boost::shared_ptr<voro::container> vc, const Eigen::Array3f &mins, const Eigen::Array3f &maxs) :
			c(nullptr),
			vc(vc),
			m(10 * 1024 * numPoints), // 10 kb per point should be enough for point lists + vertices
			intAllocator(m.get_segment_manager()),
			doubleAllocator(m.get_segment_manager()),
			cachedVoronoiCellAllocator(m.get_segment_manager()),
			sa(0),
			vol(0),
			mins(mins),
			maxs(maxs)
		{
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


		void CachedVoronoi::regenerateCache(size_t numPoints) const
		{
			if (!vc) return;
			// Iterate over cells and store cell information 
			// (prevents constant recalculations)
			using namespace boost::interprocess;

			c = m.find_or_construct<boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator>,
				CachedVoronoiCellAllocator> >("cells")(cachedVoronoiCellAllocator);
			// Cannot just use resize(numPoints) here because the appropriate allocators must be specified.
			if (c->size() != numPoints)
				c->resize(numPoints
				//, CachedVoronoiCell<IntAllocator, DoubleAllocator>()
				//(m)
				//(intAllocator, doubleAllocator)
				);

			using namespace voro;
			voronoicell_neighbor n;
			c_loop_all cl(*(vc.get()));
			if (cl.start()) do if (vc->compute_cell(n, cl)) {
				// Quantities explicitly retrieved this way to avoid any c->at(id) potential issues.
				int id;
				Eigen::Matrix3d pos;
				double r;
				cl.pos(id, pos(0), pos(1), pos(2), r); // getting id directly into field would be problematic

				auto &ci = c->at(id);

				ci.id = id;
				ci.r = r;
				ci.pos = pos;

				if (id % 1000 == 0) std::cerr << id << "\n";
				ci.calc(n);

				// Set a few fields for convenience
				vol += ci.vol;
				sa += ci.sa_ext;
			} while (cl.inc());
		}

	}
}

