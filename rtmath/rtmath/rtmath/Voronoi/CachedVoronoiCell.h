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
		/// Persistent internal Voronoi cell storage object
		class DLEXPORT_rtmath_voronoi CachedVoronoiCellBase
		{
		protected:
			void baseInit();
			CachedVoronoiCellBase();
			virtual ~CachedVoronoiCellBase();
		public:
			/// Particle id
			int id;
			/// Particle radius (unused)
			double r;
			/// Surface area of entire Voronoi cell
			double sa_full;
			/// Surface area of the cell that does not touch another cell
			double sa_ext;
			/// Particle volume
			double vol;
			/// Maximum radius squared from particle centroid
			double max_radius_squared;
			/// Sum of all edge lengths
			double total_edge_distance;
			/// Number of faces and edges in the particle
			int nFaces, nEdges;
			/// Centroid of particle (uncertain of coordinate system)
			Eigen::Matrix3d centroid;
			/// Position vector of particle
			Eigen::Matrix3d pos;
			/// Convenience function to see if the particle touches the 'surface'
			inline bool isSurface() const { if (!sa_full) return false;  if (sa_ext) return true; return false; }
		};


		/// \brief Sets the size for the arrays in the CachedVoronoiCell class
		/// \see CachedVoronoiCell
#define ArraySize 50
		//const size_t ArraySize = 50;

		/// \brief Internal aligned class for persistent voronoi cell information storage
		/// \note MSVC 2012 has a bug in object construction at runtime. 
		/// It seems related to passing constructor arguments with two allocators / using the 
		/// segment manager twice in constructors. As such, I am using static arrays
		template <class AllocInt = std::allocator<int>, class AllocDouble = std::allocator<double> >
		class CachedVoronoiCell : public CachedVoronoiCellBase
		{
		private:
			// Cannot store these because it causes annoying errors when generating operator=.
			//AllocInt allocInt;
			//AllocDouble allocDouble;
		public:
			//static const size_t ArraySize = 50;
			virtual ~CachedVoronoiCell() {}
			CachedVoronoiCell()
			{ }
			CachedVoronoiCell(voro::voronoicell_neighbor &vc)
			{
				calc(vc);
			}
			/// Cell neighbor and vertex lists. The integer in neigh 
			// corresponds to the cell id in CachedVoronoi.
			//boost::interprocess::vector<int, AllocInt> neigh, f_vert;
			std::array<int, ArraySize> neigh, f_vert;
			/// Areas of each face
			//boost::interprocess::vector<double, AllocDouble> f_areas;
			std::array<double, ArraySize> f_areas;
			//std::vector<double> v;
			/// \note Position information must be set separately (not in vc)
			void calc(voro::voronoicell_neighbor &vc)
			{
				Eigen::Matrix3d crds;
				// Need to copy vectors due to different allocators
				std::vector<int> lneigh, lf_vert;
				std::vector<double> lf_areas;
				vc.neighbors(lneigh);
				vc.face_vertices(lf_vert);
				//vc.vertices(crds(0),crds(1),crds(2),v);
				vc.face_areas(lf_areas);

				//neigh.resize(lneigh.size());

				const size_t cArraySize = ArraySize;
				std::copy_n(lneigh.begin(), std::min(cArraySize, lneigh.size()), neigh.begin());
				//f_vert.resize(lf_vert.size());
				std::copy_n(lf_vert.begin(), std::min(cArraySize, lf_vert.size()), f_vert.begin());
				//f_areas.resize(lf_areas.size());
				std::copy_n(lf_areas.begin(), std::min(cArraySize, lf_areas.size()), f_areas.begin());

				vol = vc.volume();
				sa_full = vc.surface_area();
				nFaces = vc.number_of_faces();
				nEdges = vc.number_of_edges();
				vc.centroid(centroid(0), centroid(1), centroid(2));

				max_radius_squared = vc.max_radius_squared();
				total_edge_distance = vc.total_edge_distance();

				for (int i = 0; i < neigh.size(); ++i)
					if (neigh[i] <= 0)
						sa_ext += f_areas[i];
			}
		};

		extern template DLEXPORT_rtmath_voronoi class CachedVoronoiCell < std::allocator<int>, std::allocator<double> >;
		typedef boost::interprocess::allocator<int, boost::interprocess::managed_heap_memory::segment_manager> BoostIntAllocator;
		typedef boost::interprocess::allocator<double, boost::interprocess::managed_heap_memory::segment_manager> BoostDoubleAllocator;
		extern template DLEXPORT_rtmath_voronoi class CachedVoronoiCell <BoostIntAllocator, BoostDoubleAllocator>;
	}
}
