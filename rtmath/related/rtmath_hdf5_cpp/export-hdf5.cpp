#include "export-hdf5.h"

namespace rtmath {
	namespace plugins
	{
		namespace hdf5
		{


			template <class DataType>
			MatchAttributeTypeType MatchAttributeType() { throw("Unsupported type during attribute conversion in rtmath::plugins::hdf5::MatchAttributeType."); }
			template<> MatchAttributeTypeType MatchAttributeType<std::string>() { return std::shared_ptr<H5::AtomType>(new H5::StrType(0, H5T_VARIABLE)); }
			template<> MatchAttributeTypeType MatchAttributeType<const char*>() { return std::shared_ptr<H5::AtomType>(new H5::StrType(0, H5T_VARIABLE)); }
			template<> MatchAttributeTypeType MatchAttributeType<int>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_INT)); }
			template<> MatchAttributeTypeType MatchAttributeType<unsigned long long>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_ULLONG)); }
			template<> MatchAttributeTypeType MatchAttributeType<unsigned long>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_ULONG)); }
			template<> MatchAttributeTypeType MatchAttributeType<float>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_FLOAT)); }
			template<> MatchAttributeTypeType MatchAttributeType<short>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_SHORT)); }
			template<> MatchAttributeTypeType MatchAttributeType<unsigned short>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_USHORT)); }
			template<> MatchAttributeTypeType MatchAttributeType<double>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_DOUBLE)); }
			// \note bools are not recommended in HDF5. This type may be switched later on.
			//template<> MatchAttributeTypeType MatchAttributeType<bool>() { return std::shared_ptr<H5::AtomType>(new H5::IntType(H5::PredType::NATIVE_HBOOL)); }

			template<> bool isStrType<std::string>() { return true; }
			template<> bool isStrType<const char*>() { return true; }

			template <> void insertAttr<std::string>(H5::Attribute &attr, std::shared_ptr<H5::AtomType> vls_type, const std::string& value)
			{
				attr.write(*vls_type, value);
			}
			template <> void loadAttr<std::string>(H5::Attribute &attr, std::shared_ptr<H5::AtomType> vls_type, std::string& value)
			{
				attr.read(*vls_type, value);
				//attr.write(*vls_type, value);
			}
			/*
			template <> void insertAttr<char const*>(H5::Attribute &, std::shared_ptr<H5::AtomType>, char const * const &);
			template <> void insertAttr<int>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const int&);
			template <> void insertAttr<unsigned __int64>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const unsigned __int64&);
			template <> void insertAttr<unsigned long long>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const unsigned long long&);
			template <> void insertAttr<float>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const float&);
			template <> void insertAttr<double>(H5::Attribute &, std::shared_ptr<H5::AtomType>, const double&);
			*/

			std::shared_ptr<H5::Group> openOrCreateGroup(std::shared_ptr<H5::CommonFG> base, const char* name)
			{
				std::shared_ptr<H5::Group> res;
				try {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->openGroup( name )));
				} catch( H5::GroupIException not_found_error ) {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->createGroup( name )));
				} catch( H5::FileIException not_found_error ) {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->createGroup( name )));
				}
				return res;
			}

			std::shared_ptr<H5::Group> openGroup(std::shared_ptr<H5::CommonFG> base, const char* name)
			{
				std::shared_ptr<H5::Group> res;
				try {
					res = std::shared_ptr<H5::Group>(new H5::Group( base->openGroup( name )));
				} catch( H5::GroupIException not_found_error ) {
					return nullptr;
				} catch( H5::FileIException not_found_error ) {
					return nullptr;
				}
				return res;
			}

			bool attrExists(std::shared_ptr<H5::H5Object> base, const char* name)
			{
				try {
					H5::Attribute(base->openAttribute(name));
					return true;
				}
				catch (H5::AttributeIException not_found_error) {
					return false;
				}
				catch (H5::FileIException not_found_error) {
					return false;
				}
				catch (H5::GroupIException not_found_error) {
					return false;
				}
			}

			bool groupExists(std::shared_ptr<H5::CommonFG> base, const char* name)
			{
				try {
					H5::Group( base->openGroup( name ));
					return true;
				} catch( H5::GroupIException not_found_error ) {
					return false;
				}
				catch (H5::FileIException not_found_error) {
					return false;
				}
			}

			bool datasetExists(std::shared_ptr<H5::CommonFG> base, const char* name)
			{
				try {
					H5::DataSet(base->openDataSet(name));
					return true;
				}
				catch (H5::GroupIException not_found_error) {
					return false;
				}
				catch (H5::FileIException not_found_error) {
					return false;
				}
			}


		}
	}
}

