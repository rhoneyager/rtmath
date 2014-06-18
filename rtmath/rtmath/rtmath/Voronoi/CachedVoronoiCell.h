#pragma once
#include <array>
#include <scoped_allocator>
#include <boost/interprocess/allocators/allocator.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace voro
{
	class container;
	class voronicell_neighbor;
	class c_loop_all;
}

namespace rtmath {
	namespace Voronoi {
		

		/// \brief Sets the size for the arrays in the CachedVoronoiCell class
		/// \see CachedVoronoiCell
		//const size_t ArraySize = 50;

		/// \brief Internal aligned class for persistent voronoi cell information storage
		/// \note MSVC 2012 has a bug in object construction at runtime. 
		/// It seems related to passing constructor arguments with two allocators / using the 
		/// segment manager twice in constructors. As such, I am using static arrays
		class DLEXPORT_rtmath_voronoi CachedVoronoiCell
		{
			CachedVoronoi &_parent;
			size_t _row;
		public:
			//static const size_t ArraySize = 50;
			virtual ~CachedVoronoiCell();
			CachedVoronoiCell(CachedVoronoi& parent, size_t row);
			CachedVoronoiCell(CachedVoronoi& parent, size_t row, voro::voronoicell_neighbor &vc);
			/// Cell neighbor and vertex lists. The integer in neigh 
			// corresponds to the cell id in CachedVoronoi.
			//boost::interprocess::vector<int, AllocInt> neigh, f_vert;
			//std::array<int, ArraySize> neigh, f_vert;
			/// Areas of each face
			//boost::interprocess::vector<double, AllocDouble> f_areas;
			//std::array<double, ArraySize> f_areas;
			//std::vector<double> v;
			/// \note Position information must be set separately (not in vc)
			//static void calc(voro::voronoicell_neighbor &vc, CachedVoronoi &_parent, size_t _row);
			bool isSfc() const;
		};

	}
}
