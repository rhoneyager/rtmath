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


D_Ryan_Debug_validator();
D_rtmath_validator();

D_Ryan_Debug_start()
{
	using namespace Ryan_Debug::registry;
	static const Ryan_Debug::registry::DLLpreamble id(
		"Plugin-tsv",
		"Provides tsv export of some data.",
		PLUGINID);
	dllInitResult res = Ryan_Debug_registry_register_dll(id, (void*)dllStart);
	if (res != SUCCESS) return res;

	const size_t nExts = 1;
	const char* exportExts[nExts] = { "tsv" };

	//genAndRegisterIOregistryPlural_writer<::rtmath::data::arm::arm_scanning_radar_sacr,
	//	::rtmath::data::arm::arm_IO_sacr_output_registry>(
	//	nExts, exportExts, PLUGINID_SACR_REFL, "reflectivity");
	genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::shapefile::shapefile,
		rtmath::ddscat::shapefile::shapefile_IO_output_registry>(
		nExts, exportExts, PLUGINID, "shape_data");

	genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::shapefile::shapefile,
		rtmath::ddscat::shapefile::shapefile_IO_output_registry>(
		nExts, exportExts, PLUGINID_SHP2, "shape_points");

	genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>(
		nExts, exportExts, PLUGINID_ARS, "ar_rot_data");

	Ryan_Debug::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>(
		nExts, exportExts, PLUGINID_VORO, "summary_data");

	Ryan_Debug::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDORI, "orientation_data");

	Ryan_Debug::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDISO, "isotropic_data");

	Ryan_Debug::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDISOSMALL, "isotropic_data_small");

	Ryan_Debug::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::ddscat::ddOutput,
		::rtmath::ddscat::ddOutput_IO_output_registry>(
		nExts, exportExts, PLUGINID_DDSTATS, "stats");

	Ryan_Debug::registry::genAndRegisterIOregistryPlural_writer
		<::rtmath::images::image,
		::rtmath::images::image_IO_output_registry>(
		nExts, exportExts, PLUGINID_IMAGE, "image_basicstats");

	return SUCCESS;
}


