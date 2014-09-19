#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "02DD63A5-C820-474F-933C-33D62AA65CDA"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace mie {
			/*
			std::shared_ptr<rtmath::registry::IOhandler> write_silo_multi_ddoutputs
				(std::shared_ptr<rtmath::registry::IOhandler> h, 
				const char* filename, 
				const rtmath::ddscat::ddOutput*, 
				const char* key, 
				rtmath::registry::IOhandler::IOtype iotype);


			struct silo_handle : public rtmath::registry::IOhandler
			{
				silo_handle(const char* filename, IOtype t);
				virtual ~silo_handle() {}
				void open(const char* filename, IOtype t);
				std::shared_ptr<siloFile> file;
			};
			*/

		}
	}
}

