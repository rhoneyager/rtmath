#pragma once
#include <array>
//#include <scoped_allocator>
//#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
//#include <stddef.h>
#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/Voronoi/CachedVoronoi.h"

namespace voro
{
	class container;
	class voronoicell_neighbor;
	class c_loop_all;
}

namespace rtmath {
	namespace plugins {
		namespace voro2d {
			class VoroVoronoiDiagram;

			/// Storage container class for the cached Voronoi cell information
			class VoroCachedVoronoi : 
				public ::rtmath::Voronoi::CachedVoronoi
			{
				friend class VoronoiDiagram;
				friend class VoroVoronoiDiagram;
			private:
				mutable boost::shared_ptr<::voro::container> vc;
				void calcCell(::voro::voronoicell_neighbor &vc, size_t _row);
				virtual void regenerateCache(size_t numPoints) override;
				virtual void generateCellMap() const override;
			public:
				VoroCachedVoronoi(size_t numPoints, boost::shared_ptr<::voro::container> vc,
					const Eigen::Array3f &mins, const Eigen::Array3f &maxs);
				VoroCachedVoronoi();
				VoroCachedVoronoi(boost::shared_ptr<CachedVoronoi>,
					boost::shared_ptr<::voro::container>);
				~VoroCachedVoronoi();
			};


		}
	}
}
