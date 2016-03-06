#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <array>
#include <vector>
#include <memory>
#include <string>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullPoints.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/Qhull.h"

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/registry.h>
#include "../../rtmath/rtmath/error/debug.h"

#define PLUGINID "3fb0f8e2-0d55-4bd1-88a8-7ff10367d2ee"

namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace qhull {

			struct SHARED_INTERNAL hullData {
				hullData();
				double volume, surfarea, diameter;
				double beta, theta, phi;
				std::array<double, 3>  area2d, perimeter2d;
				Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> pts;
			};

			class SHARED_INTERNAL qhullConvexHull :
				virtual public ::rtmath::ddscat::convexHull
			{
			protected:
				boost::shared_ptr<hullData> _p;
				qhullConvexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);
			public:
				static boost::shared_ptr<rtmath::ddscat::convexHull> generate
					(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);

				virtual ~qhullConvexHull();
				virtual void constructHull();
				virtual double volume() const;
				virtual double surfaceArea() const;
				virtual double maxDiameter() const;
				virtual void principalAxes(double &beta, double &theta, double &phi) const;
				virtual void area2d(double out[3]) const;
				virtual void perimeter2d(double out[3]) const;
			};


		}
	}
}

