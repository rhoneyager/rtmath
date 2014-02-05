#pragma once
#pragma warning( disable : 4251 ) // warning C4251: dll-interface

#include <memory>
#include <string>

#include <hdf5.h>
#include <H5Cpp.h>

#include "../../rtmath/rtmath/common_templates.h"
#include "../../rtmath/rtmath/plugin.h"
#include "../../rtmath/rtmath/error/debug.h"


#define PLUGINID "3733FE13-F12A-4AEA-A377-F3FAAE28D5A8"


namespace rtmath {
	namespace ddscat {
		namespace shapefile { class shapefile; }
		namespace stats { class shapeFileStats; }
	}
	namespace plugins {
		namespace hdf5 {

			bool match_hdf5_shapefile(const char*, const char*);
			bool match_hdf5_multi(const char*, const char*, 
				std::shared_ptr<rtmath::registry::IOhandler>);
			void write_hdf5_shapefile(const char*,
				const rtmath::ddscat::shapefile::shapefile *shp);
			std::shared_ptr<rtmath::registry::IOhandler> write_hdf5_multi_shapefile
				(std::shared_ptr<rtmath::registry::IOhandler> h, 
				const char* filename, 
				const rtmath::ddscat::shapefile::shapefile *shp, 
				const char* key, 
				rtmath::registry::IOhandler::IOtype iotype);
			std::shared_ptr<H5::Group> write_hdf5_shaperawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::shapefile::shapefile *shp);

			//bool match_hdf5_shapestats(const char*, const char*);
			void write_hdf5_shapestats(const char*,
				const rtmath::ddscat::stats::shapeFileStats *s);
			std::shared_ptr<rtmath::registry::IOhandler> write_hdf5_multi_shapestats
				(std::shared_ptr<rtmath::registry::IOhandler> h, 
				const char* filename, 
				const rtmath::ddscat::stats::shapeFileStats *s, 
				const char* key, 
				rtmath::registry::IOhandler::IOtype iotype);
			std::shared_ptr<H5::Group> write_hdf5_statsrawdata(std::shared_ptr<H5::Group> base, 
				const rtmath::ddscat::stats::shapeFileStats *s);

			struct hdf5_handle : public rtmath::registry::IOhandler
			{
				hdf5_handle(const char* filename, IOtype t);
				virtual ~hdf5_handle() {}
				void open(const char* filename, IOtype t);
				std::shared_ptr<H5::H5File> file;
			};


			/// \param std::shared_ptr<H5::AtomType> is a pointer to a newly-constructed matching type
			/// \returns A pair of (the matching type, a flag indicating passing by pointer or reference)
			typedef std::shared_ptr<H5::AtomType> MatchAttributeTypeType;
			template <class DataType>
			MatchAttributeTypeType MatchAttributeType();
			
			
			/// Handles proper insertion of strings versus other data types
			template <class DataType>
			void insertAttr(H5::Attribute &attr, std::shared_ptr<H5::AtomType> vls_type, const DataType& value)
			{
				attr.write(*vls_type, &value);
			}
			template <> void insertAttr<std::string>(H5::Attribute &attr, std::shared_ptr<H5::AtomType> vls_type, const std::string& value);
			
			/// Convenient template to add an attribute of a variable type to a group or dataset
			template <class DataType, class Container>
			void addAttr(std::shared_ptr<Container> obj, const char* attname, const DataType &value)
			{
				std::shared_ptr<H5::AtomType> vls_type = MatchAttributeType<DataType>();
				H5::DataSpace att_space(H5S_SCALAR);
				H5::Attribute attr = obj->createAttribute(attname, *vls_type, att_space);
				insertAttr<DataType>(attr, vls_type, value);
			}
			
			/// Eigen objects have a special writing function, as MSVC 2012 disallows partial template specialization.
			template <class DataType, class Container>
			void addAttrEigen(std::shared_ptr<Container> obj, const char* attname, const DataType &value)
			{
				hsize_t sz[] = { value.rows(), value.cols() };
				if (sz[0] == 1)
				{
					sz[0] = sz[1];
					sz[1] = 1;
				}
				int dimensionality = (sz[1] == 1) ? 1 : 2;

				std::shared_ptr<H5::AtomType> ftype = MatchAttributeType<DataType::Scalar>();
				//H5::IntType ftype(H5::PredType::NATIVE_FLOAT);
				H5::ArrayType vls_type(*ftype, dimensionality, sz);

				H5::DataSpace att_space(H5S_SCALAR);
				H5::Attribute attr = obj->createAttribute(attname, vls_type, att_space);
				attr.write(vls_type, value.data());
			}

			/// Convenience function to either open or create a group
			std::shared_ptr<H5::Group> openOrCreateGroup(
				std::shared_ptr<H5::CommonFG> base, const char* name);

			/// Convenience function to check if a given group exists
			bool groupExists(std::shared_ptr<H5::CommonFG> base, const char* name);

			/// Convenience function to write an Eigen object, in the correct format
			template <class DataType, class Container>
			void addDatasetEigen(std::shared_ptr<Container> obj, const char* name, const DataType &value)
			{
				using namespace H5;
				hsize_t sz[] = { value.rows(), value.cols() };
				int dimensionality = 2;
				DataSpace fspace(dimensionality, sz);
				std::shared_ptr<H5::AtomType> ftype = MatchAttributeType<DataType::Scalar>();

				DSetCreatPropList plist;
				int fillvalue = -1;
				plist.setFillValue(PredType::NATIVE_INT, &fillvalue);

				std::shared_ptr<DataSet> dataset(new DataSet(obj->createDataSet(name, *(ftype.get()), 
					fspace, plist)));
				Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> rowmajor(value);
				dataset->write(value.data(), *(ftype.get()));
			}

		}
	}
}

