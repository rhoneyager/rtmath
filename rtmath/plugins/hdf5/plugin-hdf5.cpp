/// \brief Provides HDF5 io routines
#pragma warning( disable : 4251 ) // warning C4251: dll-interface
#define _SCL_SECURE_NO_WARNINGS

#include <cstdio>
#include <cstring>
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
			template<> MatchAttributeTypeType MatchAttributeType<float>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_FLOAT)); }
			template<> MatchAttributeTypeType MatchAttributeType<double>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_DOUBLE)); }

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
	static const rtmath::registry::DLLpreamble id(
		"Plugin-HDF5",
		"Plugin that provides HDF5 io routines for various classes.",
		PLUGINID);
	rtmath_registry_register_dll(id);

	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::shapefile::shapefile> s;
	s.io_matches = rtmath::plugins::hdf5::match_hdf5_shapefile;
	s.io_processor = rtmath::plugins::hdf5::write_hdf5_shapefile;
	s.io_multi_matches = rtmath::plugins::hdf5::match_hdf5_multi;
	s.io_multi_processor = rtmath::plugins::hdf5::write_hdf5_multi_shapefile;
	rtmath::ddscat::shapefile::shapefile::usesDLLregistry<
		rtmath::ddscat::shapefile::shapefile_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::shapefile::shapefile> >
		::registerHook(s);


	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::stats::shapeFileStats> s2;
	s2.io_matches = rtmath::plugins::hdf5::match_hdf5_shapefile;
	s2.io_processor = rtmath::plugins::hdf5::write_hdf5_shapestats;
	s2.io_multi_matches = rtmath::plugins::hdf5::match_hdf5_multi;
	s2.io_multi_processor = rtmath::plugins::hdf5::write_hdf5_multi_shapestats;
	rtmath::ddscat::stats::shapeFileStats::usesDLLregistry<
		rtmath::ddscat::stats::shapeFileStats_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::stats::shapeFileStats> >
		::registerHook(s2);

	static rtmath::registry::IO_class_registry<
		::rtmath::ddscat::ddOutput> s3;
	s3.io_matches = rtmath::plugins::hdf5::match_hdf5_shapefile;
	s3.io_processor = rtmath::plugins::hdf5::write_hdf5_ddOutput;
	s3.io_multi_matches = rtmath::plugins::hdf5::match_hdf5_multi;
	s3.io_multi_processor = rtmath::plugins::hdf5::write_hdf5_multi_ddoutputs;
	rtmath::ddscat::ddOutput::usesDLLregistry<
		rtmath::ddscat::ddOutput_IO_output_registry,
		rtmath::registry::IO_class_registry<::rtmath::ddscat::ddOutput> >
		::registerHook(s3);
}
