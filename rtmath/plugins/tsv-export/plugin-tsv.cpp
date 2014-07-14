/// \brief Provides TSV file IO
#define _SCL_SECURE_NO_WARNINGS

#include <string>

#include "../../rtmath/rtmath/defs.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/images/image.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/data/arm_info.h"
#include "../../rtmath/rtmath/data/arm_scanning_radar_sacr.h"
#include "../../rtmath/rtmath/plugin.h"

#include "plugin-tsv.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

void dllEntry()
{
	using namespace rtmath::registry;
	static const rtmath::registry::DLLpreamble id(
		"Plugin-tsv",
		"Provides tsv export of some data.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	const size_t nExts = 1;
	const char* exportExts[nExts] = { "tsv" };

	//genAndRegisterIOregistryPlural_writer<::rtmath::data::arm::arm_scanning_radar_sacr,
	//	::rtmath::data::arm::arm_IO_sacr_output_registry>(
	//	nExts, exportExts, PLUGINID_SACR_REFL, "reflectivity");

	genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>(
		nExts, exportExts, PLUGINID_ARS, "ar_rot_data");

	rtmath::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>(
		nExts, exportExts, PLUGINID_VORO, "summary_data");

	rtmath::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDORI, "orientation_data");

	rtmath::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDISO, "isotropic_data");

	rtmath::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDSTATS, "stats");

	rtmath::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::images::image,
		::rtmath::images::image_IO_output_registry>(
		nExts, exportExts, PLUGINID_IMAGE, "image_basicstats");


}


