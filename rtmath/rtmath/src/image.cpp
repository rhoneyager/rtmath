#include "Stdafx-images.h"
#include "../rtmath/images/image.h"

namespace rtmath {
	namespace registry {
		template struct IO_class_registry_writer<
			::rtmath::images::image>;

		template struct IO_class_registry_reader<
			::rtmath::images::image>;

		template class usesDLLregistry<
			::rtmath::images::image_IO_input_registry,
			IO_class_registry_reader<::rtmath::images::image> >;

		template class usesDLLregistry<
			::rtmath::images::image_IO_output_registry,
			IO_class_registry_writer<::rtmath::images::image> >;
	}

	namespace images {

		image::~image() {}

		image::image(const std::string &filename)
		{
			this->read(filename);
		}

		image::image()
		{
		}
	}
}
