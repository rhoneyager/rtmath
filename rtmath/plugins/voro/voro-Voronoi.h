//#include "Stdafx-voronoi.h"
#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
//#include <Voro/voro++.hh>

namespace voro
{
	class container;
	class voronicell_neighbor;
	class c_loop_all;
}

namespace rtmath
{
	namespace plugins
	{
		namespace voro
		{
			class VoroVoronoiDiagram :
				public ::rtmath::Voronoi::VoronoiDiagram
			{
			private:
				/// Shared voronoi container object
				mutable boost::shared_ptr<::voro::container> vc;
			public:
				VoroVoronoiDiagram();
				virtual ~VoroVoronoiDiagram();

				virtual void regenerateVoronoi() const override;
				virtual void regenerateFull() const override;

				virtual void getResultsTable(std::map<std::string, matrixType> &res) const override;
				virtual const Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>* getCellMap() const override;
				virtual void getBounds(Eigen::Array3f &mins, Eigen::Array3f &maxs, Eigen::Array3f &span) const override;

				virtual matrixType calcSurfaceDepth() const override;
				virtual matrixType calcSurfaceDepthVectors() const override;
				virtual matrixType calcSurfaceNumNeighs() const override;
				virtual matrixType calcSurfaceFillingOrder() const override;
				virtual matrixType calcCandidateConvexHullPoints() const override;
				virtual matrixType calcPointsSAfracExternal() const override;
				virtual double surfaceArea() const override;
				virtual double volume() const override;
				virtual size_t numPoints() const override;
				virtual Eigen::Array3i getSpan() const override;

				// These three are registered in a Voronoi_provider object.
				static boost::shared_ptr<VoronoiDiagram> generateStandard(
					const Eigen::Array3f &mins, const Eigen::Array3f &maxs,
					const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>& points
					);
				static boost::shared_ptr<VoronoiDiagram> generateBlank();
				static boost::shared_ptr<VoronoiDiagram> generateUpcast(
					boost::shared_ptr<VoronoiDiagram>);

			};
		}
	}
}

