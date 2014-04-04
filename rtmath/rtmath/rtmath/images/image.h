#pragma once

#include "../defs.h"
#include <functional>
#include <map>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "../registry.h"
#include "../io.h"

namespace rtmath
{
	namespace images
	{
		class image;
		class image_IO_input_registry {};
		class image_IO_output_registry {};
	}
	namespace registry {
		extern template struct IO_class_registry_writer<
			::rtmath::images::image>;

		extern template struct IO_class_registry_reader<
			::rtmath::images::image>;

		extern template class usesDLLregistry<
			::rtmath::images::image_IO_input_registry,
			IO_class_registry_reader<::rtmath::images::image> >;

		extern template class usesDLLregistry<
			::rtmath::images::image_IO_output_registry,
			IO_class_registry_writer<::rtmath::images::image> >;

	}
	/// Contains image manipulation code
	namespace images
	{
		class DLEXPORT_rtmath_images image : 
			virtual public ::rtmath::registry::usesDLLregistry<
				image_IO_input_registry, 
				::rtmath::registry::IO_class_registry_reader<image> >,
			virtual public ::rtmath::registry::usesDLLregistry<
				image_IO_output_registry, 
				::rtmath::registry::IO_class_registry_writer<image> >,
			virtual public ::rtmath::io::implementsStandardWriter<image, image_IO_output_registry>,
			virtual public ::rtmath::io::implementsStandardReader<image, image_IO_input_registry>
		{
		public:
			image();
			image(const std::string &filename);
			virtual ~image();

			mutable std::map < std::string, 
				boost::shared_ptr<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> > > 
				imageMaps;

		};
	}
}
