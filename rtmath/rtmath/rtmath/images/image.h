#pragma once

#include "../defs.h"
#include <functional>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Ryan_Debug/registry.h>
#include <Ryan_Debug/io.h>

namespace rtmath
{
	namespace images
	{
		class image;
		class image_IO_input_registry {};
		class image_IO_output_registry {};
	}
}
namespace Ryan_Debug {
	namespace registry {
		extern template struct IO_class_registry_writer <
			::rtmath::images::image > ;

		extern template struct IO_class_registry_reader <
			::rtmath::images::image > ;

		extern template class usesDLLregistry <
			::rtmath::images::image_IO_input_registry,
			IO_class_registry_reader<::rtmath::images::image> > ;

		extern template class usesDLLregistry <
			::rtmath::images::image_IO_output_registry,
			IO_class_registry_writer<::rtmath::images::image> > ;
	}
}
namespace rtmath {
	/// Contains image manipulation code
	namespace images
	{
		class DLEXPORT_rtmath_images image : 
			virtual public ::Ryan_Debug::registry::usesDLLregistry<
				image_IO_input_registry, 
				::Ryan_Debug::registry::IO_class_registry_reader<image> >,
				virtual public ::Ryan_Debug::registry::usesDLLregistry<
				image_IO_output_registry, 
				::Ryan_Debug::registry::IO_class_registry_writer<image> >,
				virtual public ::Ryan_Debug::io::implementsStandardWriter<image, image_IO_output_registry>,
				virtual public ::Ryan_Debug::io::implementsStandardReader<image, image_IO_input_registry>
		{
		public:
			image();
			image(const std::string &filename);
			virtual ~image();

			mutable std::map < std::string, 
				boost::shared_ptr<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > 
				imageMaps;

			void doStats();

			/// x, y, val
			Eigen::Vector3f mins, maxs, means, variances;
			size_t numTotal, numFilled, rows, cols;
			float frac;
		};
	}
}
