#pragma once
/// Private header that provides encapsulation of the Voro++ data

#include "../defs.h"
#include <boost/shared_ptr.hpp>
//#include <Voro++/voro++.hh>

namespace voro
{
	class container;
	class voronicell_neighbor;
	class c_loop_all;
}

namespace rtmath
{
	namespace Voronoi
	{
		// SHARED_INTERNAL / DLEXPORT_rtmath_voronoi?
		/// Acts as a storage container for the Voronoi cell information
		class SHARED_INTERNAL VoronoiData
		{
		public:
			VoronoiData();
			~VoronoiData();

			boost::shared_ptr<voro::container> vc;

			// Expected functionality:


			// Constructor to import data from an Eigen Matrix
			
			// Create a Voronoi object, with preshrunk cell bounds
			
			// Project a Voronoi object along a desired plane
			
			// Get projective area

			// Get voronoi-encompassed volume and surface area
			
			// Calculate cells that touch the surface

			// Calculate a depth map of distance from a set of points
			// (surface points, a central point, ...)

			// Concave hull calculation

			// VTK export functions
			// Export just cell points
			// Export cells with point and surrounding cell


		};
	}
}

