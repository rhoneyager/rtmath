#pragma once
#include <array>
#include <scoped_allocator>
#include <boost/interprocess/allocators/allocator.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "CachedVoronoiCell.h"

namespace voro
{
	class container;
	class voronicell_neighbor;
	class c_loop_all;
}

namespace rtmath {
	namespace Voronoi {

		/// Storage container class for the cached Voronoi cell information
		class DLEXPORT_rtmath_voronoi CachedVoronoi
		{
			friend class VoronoiDiagram;
		private:
			mutable boost::interprocess::managed_heap_memory m;
			mutable double sa, vol;
		public:
			typedef boost::interprocess::allocator<int, boost::interprocess::managed_heap_memory::segment_manager> IntAllocator;
			typedef boost::interprocess::allocator<double, boost::interprocess::managed_heap_memory::segment_manager> DoubleAllocator;
			typedef boost::interprocess::allocator<CachedVoronoiCell<IntAllocator, DoubleAllocator>,
				boost::interprocess::managed_heap_memory::segment_manager> CachedVoronoiCellAllocator;
			const IntAllocator intAllocator;
			const DoubleAllocator doubleAllocator;
			const CachedVoronoiCellAllocator cachedVoronoiCellAllocator;

			mutable boost::shared_ptr<voro::container> vc;
		private:
			mutable boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator>,
				CachedVoronoiCellAllocator> *c;
			Eigen::Array3f mins, maxs;
			mutable Eigen::Array3i span;
			/// Maps each possible integral coordinate to a given cell.
			mutable Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cellmap;

			/// Iterate over all possible coordinates in the container and find a matching cell
			void generateCellMap() const;

		public:

			CachedVoronoi(size_t numPoints, boost::shared_ptr<voro::container> vc, 
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs);
			~CachedVoronoi();
			void regenerateCache(size_t numPoints) const;

			/// Calculate the surface area of the bulk figure
			inline double surfaceArea() const { return sa; }
			/// Calculate the volume of the bulk figure
			inline double volume() const { return vol; }
			/// Get the span of the bulk figure
			inline Eigen::Array3i getSpan() const { return span; }
			/// Get pointer to the set of stored voronoi cells
			inline boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator>,
				CachedVoronoiCellAllocator>* getCells() const
			{
				return c;
				//return m.find<boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator> > >("cells").first;
			}
			/// Get pointer to the mapping between coordinates and the stored cell
			inline const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* getCellMap() const { generateCellMap(); return &cellmap; }
		};


	}
}
