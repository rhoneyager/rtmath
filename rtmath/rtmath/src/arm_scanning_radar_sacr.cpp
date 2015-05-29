#include "Stdafx-data.h"

#include <sstream>

#include "../rtmath/data/arm_scanning_radar_sacr.h"
#include "../rtmath/error/debug.h"
#include <Ryan_Debug/error.h>

namespace Ryan_Debug
{
	namespace registry {

		template struct IO_class_registry_writer <
			::rtmath::data::arm::arm_scanning_radar_sacr > ;

		template struct IO_class_registry_reader <
			::rtmath::data::arm::arm_scanning_radar_sacr > ;

		template class usesDLLregistry <
			::rtmath::data::arm::arm_IO_sacr_input_registry,
			IO_class_registry_reader<::rtmath::data::arm::arm_scanning_radar_sacr> > ;

		template class usesDLLregistry <
			::rtmath::data::arm::arm_IO_sacr_output_registry,
			IO_class_registry_writer<::rtmath::data::arm::arm_scanning_radar_sacr> > ;

	}
}
namespace rtmath {
	namespace data
	{
		namespace arm
		{
			arm_scanning_radar_sacr::arm_scanning_radar_sacr() { _init(); }
			arm_scanning_radar_sacr::~arm_scanning_radar_sacr() { }

			arm_scanning_radar_sacr::arm_scanning_radar_sacr(const std::string &filename)
			{
				_init();
				read(filename);
				info = boost::shared_ptr<const arm_info>(new arm_info(filename));
			}

			void arm_scanning_radar_sacr::_init()
			{
			}

		}
	}
}

