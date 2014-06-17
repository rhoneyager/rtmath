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
		void CachedVoronoiCellBase::baseInit()
		{
			id = 0;
			r = 0;
			sa_full = 0;
			sa_ext = 0;
			vol = 0;
			max_radius_squared = 0;
			total_edge_distance = 0;
			nFaces = 0;
			nEdges = 0;
			pos.setZero();
		}

		CachedVoronoiCellBase::CachedVoronoiCellBase() { baseInit(); }

		CachedVoronoiCellBase::~CachedVoronoiCellBase() {}

		template class CachedVoronoiCell < std::allocator<int>, std::allocator<double> >;

		template class CachedVoronoiCell <BoostIntAllocator, BoostDoubleAllocator>;
	}
}

