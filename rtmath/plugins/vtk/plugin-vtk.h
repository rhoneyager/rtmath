#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include <Ryan_Debug/registry.h>
#include "../../rtmath/rtmath/error/debug.h"

#define PLUGINID "694928E5-D4C1-4463-B2D1-3B10EFBFB15B"

class vtkXMLStructuredGridWriter;
namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace vtk {

			struct vtk_xml_s_handle : public Ryan_Debug::registry::IOhandler
			{
				vtk_xml_s_handle(const char* filename, IOtype t);
				virtual ~vtk_xml_s_handle() {}
				void open(const char* filename, IOtype t);
				std::shared_ptr<vtkXMLStructuredGridWriter> writer;
			};



			class SHARED_INTERNAL hullData;
			class SHARED_INTERNAL vtkConvexHull : virtual public ::rtmath::ddscat::convexHull
			{
			protected:
				boost::shared_ptr<hullData> _p;
				vtkConvexHull(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);
			public:
				static boost::shared_ptr<rtmath::ddscat::convexHull> generate
					(boost::shared_ptr< const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > backend);

				virtual ~vtkConvexHull();
				virtual void constructHull();
				virtual double volume() const;
				virtual double surfaceArea() const;
				virtual double maxDiameter() const;
				virtual void principalAxes(double &beta, double &theta, double &phi) const;
				virtual void area2d(double out[3]) const;
				virtual void perimeter2d(double out[3]) const;
				void writeVTKraw(const std::string &filename) const;
				void writeVTKhull(const std::string &filename) const;
				//void writeVTKpolys(const std::string &filename, const vtkSmartPointer< vtkPolyData > &src);

			};


		}
	}
}

