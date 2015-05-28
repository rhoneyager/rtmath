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
#include <Ryan_Debug/error.h>

#undef min
#undef max

namespace rtmath
{
	namespace Voronoi {
		CachedVoronoi::~CachedVoronoi() {}

		void CachedVoronoi::resize(size_t n)
		{
			if (!tblDoubles)
			{
				this->tblDoubles = boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, NUM_CELL_DEFS_DOUBLES> >(
					new Eigen::Matrix<double, Eigen::Dynamic, NUM_CELL_DEFS_DOUBLES>);
				this->tblInts = boost::shared_ptr<Eigen::Matrix<int, Eigen::Dynamic, NUM_CELL_DEFS_INTS> >(
					new Eigen::Matrix<int, Eigen::Dynamic, NUM_CELL_DEFS_INTS>);
				this->tblCellNeighs = boost::shared_ptr<Eigen::Matrix<int, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL> >(
					new Eigen::Matrix<int, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL>);
				this->tblCellF_verts = boost::shared_ptr<Eigen::Matrix<int, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL> >(
					new Eigen::Matrix<int, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL>);
				this->tblCellF_areas = boost::shared_ptr<Eigen::Matrix<double, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL> >(
					new Eigen::Matrix<double, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL>);
				this->cellmap = boost::shared_ptr<Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> >(
					new Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>);
			}

			tblDoubles->resize(n, NUM_CELL_DEFS_DOUBLES);
			tblInts->resize(n, NUM_CELL_DEFS_INTS);
			tblCellNeighs->resize(n, CachedVoronoi_MaxNeighbors_VAL);
			tblCellF_verts->resize(n, CachedVoronoi_MaxNeighbors_VAL);
			tblCellF_areas->resize(n, CachedVoronoi_MaxNeighbors_VAL);

			tblCellNeighs->setZero();
			tblCellF_verts->setZero();
			tblCellF_areas->setZero();

		}

		CachedVoronoi::CachedVoronoi() : sa(0), vol(0) { resize(0); }

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
			if (cellmap->rows() > 0) return;

			RDthrow(Ryan_Debug::error::xUpcast());
		}

		void CachedVoronoi::regenerateCache(size_t numPoints)
		{
			RDthrow(Ryan_Debug::error::xUpcast());
		}

		boost::shared_ptr<const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> > CachedVoronoi::getCellMap() const
		{
			generateCellMap(); return cellmap;
		}

	}
}

