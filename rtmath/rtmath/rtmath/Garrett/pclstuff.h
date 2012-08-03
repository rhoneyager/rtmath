#pragma once
/* Private header for PCL and other libraries
 * This header is private to speed up compilation times and to allow 
 * for selectivity in the build system
 */

#include <string>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include <pcl/point_types.h>

#pragma GCC diagnostic pop

namespace rtmath
{
	namespace Garrett
	{
		class pointContainer
		{
			public:
				pointContainer();
				~pointContainer();
				void readPCD(const std::string &filename);
				void writePCD(const std::string &filename) const;
				void readVTKpoints(const std::string &filename);
				void writeVTKpoints(const std::string &filename) const;
				void readPNG(const std::string &filename);
				void writePNG(const std::string &filename) const;
				void writeROOTsurf(const std::string &filename) const;
				void writeROOTzhist(const std::string &filename) const;

				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
		};
	}
}


