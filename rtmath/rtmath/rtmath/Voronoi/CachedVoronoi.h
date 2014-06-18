#pragma once
#include <array>
#include <scoped_allocator>
#include <boost/interprocess/allocators/allocator.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stddef.h>

namespace voro
{
	class container;
	class voronoicell_neighbor;
	class c_loop_all;
}

namespace rtmath {
	namespace Voronoi {

		template< class Dummy >
		struct CachedVoronoi_MaxNeighbors_
		{
			static ::ptrdiff_t const value;
		};

#define CachedVoronoi_MaxNeighbors_VAL 50
		template< class Dummy >
		::ptrdiff_t const CachedVoronoi_MaxNeighbors_<Dummy>::value = CachedVoronoi_MaxNeighbors_VAL;

		typedef CachedVoronoi_MaxNeighbors_<void> CachedVoronoi_MaxNeighbors;

		/// Storage container class for the cached Voronoi cell information
		class DLEXPORT_rtmath_voronoi CachedVoronoi : public CachedVoronoi_MaxNeighbors
		{
			friend class VoronoiDiagram;
		public: // TODO: make private and change the hdf5 reader into a friend
			//mutable boost::interprocess::managed_heap_memory m;
			mutable double sa, vol;
		public:
			mutable boost::shared_ptr<voro::container> vc;
		public:
			void resize(size_t numCells);
			Eigen::Array3f mins, maxs;
			mutable Eigen::Array3i span;
			/// Maps each possible integral coordinate to a given cell.
			mutable Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cellmap;

			/// Iterate over all possible coordinates in the container and find a matching cell
			void generateCellMap() const;

			void calcCell(voro::voronoicell_neighbor &vc, size_t _row);
		public:
			enum CellDefsDoubles
			{
				RADIUS,
				SA_FULL,
				SA_EXT,
				VOLUME,
				MAX_RADIUS_SQUARED,
				TOTAL_EDGE_DISTANCE,
				CENTROID_X,
				CENTROID_Y,
				CENTROID_Z,
				POS_X,
				POS_Y,
				POS_Z,
				NUM_CELL_DEFS_DOUBLES
			};
			enum CellDefsInts
			{
				ID,
				NFACES,
				NEDGES,
				NUM_CELL_DEFS_INTS
			};

			Eigen::Matrix<double, Eigen::Dynamic, NUM_CELL_DEFS_DOUBLES> tblDoubles;
			Eigen::Matrix<int, Eigen::Dynamic, NUM_CELL_DEFS_INTS> tblInts;
			Eigen::Matrix<int, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL> tblCellNeighs;
			Eigen::Matrix<int, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL> tblCellF_verts;
			Eigen::Matrix<double, Eigen::Dynamic, CachedVoronoi_MaxNeighbors_VAL> tblCellF_areas;

			CachedVoronoi(size_t numPoints, boost::shared_ptr<voro::container> vc, 
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs);
			CachedVoronoi(); // Used in hdf5 read
			~CachedVoronoi();
			void regenerateCache(size_t numPoints);

			/// Calculate the surface area of the bulk figure
			inline double surfaceArea() const { return sa; }
			/// Calculate the volume of the bulk figure
			inline double volume() const { return vol; }
			/// Get the span of the bulk figure
			inline Eigen::Array3i getSpan() const { return span; }
			/*
			/// Get pointer to the set of stored voronoi cells
			inline std::vector<CachedVoronoiCell>* getCells() const
			{
				return &c;
				//return m.find<boost::interprocess::vector<CachedVoronoiCell<IntAllocator, DoubleAllocator> > >("cells").first;
			}
			*/
			/// Get pointer to the mapping between coordinates and the stored cell
			inline const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* getCellMap() const { generateCellMap(); return &cellmap; }
		};


	}
}
