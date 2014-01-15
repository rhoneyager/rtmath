/// \brief Provides silo file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/plugin.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

bool match_silo_shapefile(const char*);
void write_silo_shapefile(const char*, 
	const rtmath::ddscat::shapefile::shapefile *shp);


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
	s.io_matches = match_silo_shapefile;
	s.io_processor = write_silo_shapefile;
	rtmath::ddscat::shapefile::shapefile::usesDLLregistry<
		rtmath::ddscat::shapefile::shapefile_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >
		::registerHook(s);
	//std::cerr << "plugin-silo dll loaded!\n\n";
}
