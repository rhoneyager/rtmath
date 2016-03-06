//#include "Stdafx-voronoi.h"
#include "../../rtmath/rtmath/defs.h"
#pragma warning( disable : 4996 )
#pragma warning( disable : 4305 )
#pragma warning( disable : 4244 )

#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/math/constants/constants.hpp>

#include <Eigen/Dense>

#include "../../rtmath/rtmath/ddscat/rotations.h"
#include "../../rtmath/rtmath/ddscat/hulls.h"
#include <Ryan_Debug/error.h>

#include "plugin-qhull.h"
using orgQhull::Qhull;
using orgQhull::QhullError;
using orgQhull::QhullFacet;
using orgQhull::QhullFacetList;
using orgQhull::QhullQh;
using orgQhull::RboxPoints;
using orgQhull::QhullVertex;
using orgQhull::QhullVertexSet;
using orgQhull::QhullPoints;
using orgQhull::QhullPoint;

namespace rtmath
{
	namespace plugins
	{
		namespace qhull
		{
			hullData::hullData() :
				volume(0), surfarea(0), diameter(0),
				beta(0), theta(0), phi(0)
			{}

			qhullConvexHull::qhullConvexHull(
				boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend) : rtmath::ddscat::convexHull(backend)
			{
				_p = boost::shared_ptr<hullData>(new hullData);
				_p->pts = (*(backend.get())).cast<double>();
			}

			boost::shared_ptr<rtmath::ddscat::convexHull> qhullConvexHull::generate
				(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend)
			{
				return boost::shared_ptr<rtmath::ddscat::convexHull>(new qhullConvexHull(backend));
			}

			qhullConvexHull::~qhullConvexHull() {}
			/// Construct the convex hull, and populate the quantities
			/// \todo Invoke in constructor?
			void qhullConvexHull::constructHull()
			{
				orgQhull::Qhull qhull;
				const char* chull = "hull";
				qhull.runQhull(chull, 3, _p->pts.rows(), _p->pts.data(), "Qt");

				_p->surfarea = qhull.area();
				_p->volume = qhull.volume();
				_p->diameter = 0;

				QhullPoints pts = qhull.points();
				std::vector< QhullPoint > ptsv = pts.toStdVector();
				// Take the input points and convert to a row-defined matrix
				// Just calculate the max diameter here
				auto fMaxDiameter = [](const std::vector< QhullPoint > &base,
					size_t &a, size_t &b) -> double
				{
					double maxD = 0;
					a = 0;  b = 0;
					size_t np = (size_t)base.size();
					for (size_t i = 0; i < np; i++)
					{
						double it[3];
						it[0] = base.at(i)[0];
						it[1] = base.at(i)[1];
						it[2] = base.at(i)[2];

						for (size_t j = i + 1; j < np; j++)
						{
							double ot[3];
							ot[0] = base.at(j)[0];
							ot[1] = base.at(j)[1];
							ot[2] = base.at(j)[2];

							double d = pow(it[0] - ot[0], 2.f)
								+ pow(it[1] - ot[1], 2.f)
								+ pow(it[2] - ot[2], 2.f);
							if (d > maxD)
							{
								maxD = d;
								a = i;
								b = j;
							}
						}
					}

					// These get optimized away, but are kept for debugging

					return sqrt(maxD);
				};

				// Get max diameter and store the point ids
				size_t mp1, mp2;
				_p->diameter = fMaxDiameter(ptsv, mp1, mp2);

			}

			double qhullConvexHull::volume() const { return _p->volume; }
			double qhullConvexHull::surfaceArea() const { return _p->surfarea; }
			double qhullConvexHull::maxDiameter() const { return _p->diameter; }
			void qhullConvexHull::principalAxes(double &beta, double &theta, double &phi) const { beta = _p->beta; theta = _p->theta; phi = _p->phi; }
			void qhullConvexHull::area2d(double out[3]) const { std::copy_n(_p->area2d.data(), 3, out); }
			void qhullConvexHull::perimeter2d(double out[3]) const { std::copy_n(_p->perimeter2d.data(), 3, out); }

		}
	}
}

