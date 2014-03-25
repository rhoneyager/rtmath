/// \brief Provides HDF5 io routines
#pragma warning( disable : 4251 ) // warning C4251: dll-interface
#define _SCL_SECURE_NO_WARNINGS

//#include <cstdio>
//#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <hdf5.h>
#include <H5Cpp.h>


#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/ddscat/shapefile.h"
#include "../../rtmath/rtmath/ddscat/shapestats.h"
#include "../../rtmath/rtmath/ddscat/ddOutput.h"
#include "../../rtmath/rtmath/Voronoi/Voronoi.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"

#include "plugin-hdf5.h"

void dllEntry();
rtmath_plugin_init(dllEntry);

namespace rtmath {
	namespace plugins {
		namespace hdf5 {
			hdf5_handle::hdf5_handle(const char* filename, IOtype t)
				: IOhandler(PLUGINID)
			{
				open(filename, t);
			}

			void hdf5_handle::open(const char* filename, IOtype t)
				{
					using namespace H5;
					switch (t)
					{
					case IOtype::READWRITE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_RDWR ));
						break;
					case IOtype::EXCLUSIVE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_EXCL ));
						break;
					case IOtype::DEBUG:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_DEBUG ));
						break;
					case IOtype::CREATE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_CREAT ));
						break;
					case IOtype::READONLY:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_RDONLY ));
						break;
					case IOtype::TRUNCATE:
						file = std::shared_ptr<H5File>(new H5File(filename, H5F_ACC_TRUNC ));
						break;
					}
				}

			template <class DataType>
			MatchAttributeTypeType MatchAttributeType() { throw("Unsupported type during attribute conversion in rtmath::plugins::hdf5::MatchAttributeType."); }
			template<> MatchAttributeTypeType MatchAttributeType<std::string>() { return std::shared_ptr<H5::AtomType>(new H5::StrType(0, H5T_VARIABLE)); }
			template<> MatchAttributeTypeType MatchAttributeType<const char*>() { return std::shared_ptr<H5::AtomType>(new H5::StrType(0, H5T_VARIABLE)); }
			template<> MatchAttributeTypeType MatchAttributeType<int>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_INT)); }
			template<> MatchAttributeTypeType MatchAttributeType<unsigned long long>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_ULLONG)); }
			template<> MatchAttributeTypeType MatchAttributeType<unsigned long>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_ULONG)); }
			template<> MatchAttributeTypeType MatchAttributeType<float>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_FLOAT)); }
			template<> MatchAttributeTypeType MatchAttributeType<double>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_DOUBLE)); }
			// \note bools are not recommended in HDF5. This type may be switched later on.
			//template<> MatchAttributeTypeType MatchAttributeType<bool>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_HBOOL)); }

			template<> bool isStrType<std::string>() { return true; }
			template<> bool isStrType<const char*>() { return true; }

			template <> void insertAttr<std::string>(H5::Attribute &attr, std::shared_ptr<H5::AtomType> vls_type, const std::string& value)
			{
				attr.write(*vls_type, value);
			}
			/*
			template <> void insertAttr<char const*>(H5::Attribute &, std::shared_ptr<H5::AtomType>, char const * const &);
			template <> void insertAttr<int>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const int&);
			template <> void insertAttr<unsigned __int64>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const unsigned __int64&);
			template <> void insertAttr<unsigned long long>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const unsigned long long&);
			template <> void insertAttr<float>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const float&);
			template <> void insertAttr<double>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const double&);
			*/
		}
	}
}


void dllEntry()
{
	using namespace rtmath::registry;
	using namespace rtmath::plugins::hdf5;
	static const DLLpreamble id(
		"Plugin-HDF5",
		"Plugin that provides HDF5 io routines for various classes.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	genAndRegisterIOregistry_writer<::rtmath::ddscat::shapefile::shapefile,
		rtmath::ddscat::shapefile::shapefile_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::ddscat::stats::shapeFileStats,
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::ddscat::ddOutput,
		rtmath::ddscat::ddOutput_IO_output_registry>("hdf5", PLUGINID);
	genAndRegisterIOregistry_writer<::rtmath::Voronoi::VoronoiDiagram,
		::rtmath::Voronoi::Voronoi_IO_output_registry>("hdf5", PLUGINID);


	//genAndRegisterIOregistry_reader<::rtmath::ddscat::shapefile::shapefile,
	//	rtmath::ddscat::shapefile::shapefile_IO_input_registry>("hdf5", PLUGINID);
	//genAndRegisterIOregistry_reader<::rtmath::ddscat::stats::shapeFileStats,
	//	rtmath::ddscat::stats::shapeFileStats_IO_input_registry>("hdf5", PLUGINID);
	//genAndRegisterIOregistry_reader<::rtmath::ddscat::ddOutput,
	//	rtmath::ddscat::ddOutput_IO_input_registry>("hdf5", PLUGINID);
}
