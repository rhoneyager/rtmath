#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "f8340412-f146-47c4-8b32-a395d829f7b2"


namespace rtmath {
	namespace ddscat {
		class ddOutput;
		class ddOutputSingle;
		class ddScattMatrix;
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace silo {
			class siloFile;

			std::shared_ptr<rtmath::registry::IOhandler> write_silo_multi_shapefile
				(std::shared_ptr<rtmath::registry::IOhandler> h, 
				const char* filename, 
				const rtmath::ddscat::shapefile::shapefile *shp, 
				const char* key, 
				rtmath::registry::IOhandler::IOtype iotype);

			std::shared_ptr<rtmath::registry::IOhandler> write_silo_multi_shapestats
				(std::shared_ptr<rtmath::registry::IOhandler> h, 
				const char* filename, 
				const rtmath::ddscat::stats::shapeFileStats *s, 
				const char* key, 
				rtmath::registry::IOhandler::IOtype iotype);
			
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


		}
	}
}

