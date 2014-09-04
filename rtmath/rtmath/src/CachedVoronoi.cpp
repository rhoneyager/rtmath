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
#include <cmath>
//#include <Ryan_Serialization/serialization.h>
#include "../rtmath/depGraph.h"
//#include "../rtmath/Voronoi/Voronoi.h"
#include "../rtmath/Voronoi/CachedVoronoi.h"
#include "../rtmath/error/error.h"

#undef min
#undef max

namespace rtmath
{
	namespace Voronoi {
		CachedVoronoi::~CachedVoronoi() {}

		void CachedVoronoi::resize(size_t n)
		{
			tblDoubles.resize(n, NUM_CELL_DEFS_DOUBLES);
			tblInts.resize(n, NUM_CELL_DEFS_INTS);
			tblCellNeighs.resize(n, CachedVoronoi_MaxNeighbors_VAL);
			tblCellF_verts.resize(n, CachedVoronoi_MaxNeighbors_VAL);
			tblCellF_areas.resize(n, CachedVoronoi_MaxNeighbors_VAL);

			tblCellNeighs.setZero();
			tblCellF_verts.setZero();
			tblCellF_areas.setZero();

		}

		CachedVoronoi::CachedVoronoi() {}

		CachedVoronoi::CachedVoronoi(size_t numPoints, const Eigen::Array3f &mins, const Eigen::Array3f &maxs) :
			sa(0),
			vol(0),
			mins(mins),
			maxs(maxs)
		{
			resize(numPoints);
		}

		void CachedVoronoi::generateCellMap() const
		{
			if (cellmap.rows() > 0) return;

			throw rtmath::debug::xUpcast("CachedVoronoi", "generateCellMap");
		}

		void CachedVoronoi::regenerateCache(size_t numPoints)
		{
			throw rtmath::debug::xUpcast("CachedVoronoi", "regenerateCache");
		}

		const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* CachedVoronoi::getCellMap() const
		{
			generateCellMap(); return &cellmap;
		}

	}
}

