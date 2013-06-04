#pragma once
/* Private header for PCL and other libraries
 * This header is private to speed up compilation times and to allow 
 * for selectivity in the build system
 */

#include <string>

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#pragma message("Warning: pclstuff.h needs revision")
#include <pcl/point_types.h>

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

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


