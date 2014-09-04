#pragma once
#include <array>
//#include <scoped_allocator>
//#include <boost/interprocess/allocators/allocator.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
//#include <stddef.h>
namespace rtmath {
	namespace Voronoi {

#define CachedVoronoi_MaxNeighbors_VAL 50
		
		/// \brief Storage container class for the cached Voronoi cell information
		/// \note This is a base class! It can be overridden to store more. Everything here 
		/// gets written with the hdf5 plugin.
		class DLEXPORT_rtmath_voronoi CachedVoronoi
		{
			//friend class VoronoiDiagram;
		public:
			mutable Eigen::Array3i span;
			/// Maps each possible integral coordinate to a given cell.
			mutable Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cellmap;
		protected:
			void resize(size_t numCells);
			mutable double sa, vol;
			//mutable boost::shared_ptr<voro::container> vc;
			//void calcCell(voro::voronoicell_neighbor &vc, size_t _row);
			virtual void regenerateCache(size_t numPoints);
			/// Iterate over all possible coordinates in the container and find a matching cell
			virtual void generateCellMap() const;
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

			Eigen::Array3f mins, maxs;

			CachedVoronoi(size_t numPoints, 
				const Eigen::Array3f &mins, const Eigen::Array3f &maxs);
			CachedVoronoi(); // Used in hdf5 read
			virtual ~CachedVoronoi();
			
			/// Calculate the surface area of the bulk figure
			inline double surfaceArea() const { return sa; }
			inline void surfaceArea(double s) { sa = s; } // used in io reads
			/// Calculate the volume of the bulk figure
			inline double volume() const { return vol; }
			inline void volume(double v) { vol = v; } // used in io reads
			/// Get the span of the bulk figure
			inline Eigen::Array3i getSpan() const { return span; }
			/// Get pointer to the mapping between coordinates and the stored cell
			virtual const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* getCellMap() const;
		};


	}
}
