/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/plugin.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath {
	namespace plugins {
		namespace silo {

			bool match_silo_shapefile(const char*, const char*);
			void write_silo_shapefile(const char*,
				const rtmath::ddscat::shapefile::shapefile *shp);

			bool match_silo_ddOutput(const char*, const char*);
			void write_silo_ddOutput(const char*,
				const rtmath::ddscat::ddOutput*);
		}
	}
}

void dllEntry()
{
	static const rtmath::registry::DLLpreamble id(
		"Plugin-SILO",
		"Example plugin to provide shapefile class with the ability to "
		"read and write SILO files.",
		"f8340412-f146-47c4-8b32-a395d829f7b2");
	rtmath_registry_register_dll(id);

	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::shapefile::shapefile> s;
	s.io_matches = rtmath::plugins::silo::match_silo_shapefile;
	s.io_processor = rtmath::plugins::silo::write_silo_shapefile;

	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::ddOutput> o;
	o.io_matches = rtmath::plugins::silo::match_silo_ddOutput;
	o.io_processor = rtmath::plugins::silo::write_silo_ddOutput;

	rtmath::ddscat::shapefile::shapefile::usesDLLregistry<
		rtmath::ddscat::shapefile::shapefile_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >
		::registerHook(s);
	
	rtmath::ddscat::ddOutput::usesDLLregistry<
		rtmath::ddscat::ddOutput_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::ddOutput> >
		::registerHook(o);
}
