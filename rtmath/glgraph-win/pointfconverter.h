#pragma once
/* pointfconverter - provides the necessary TypeConverter for PointF so that 
it can be edited in the properties box!
*/

using namespace System;
using namespace System::Drawing;
using namespace System::ComponentModel;

namespace glgraphwin {

	public ref class PointFTypeConverter : System::ComponentModel::TypeConverter
	{
	public: virtual bool CanConvertFrom(System::ComponentModel::ITypeDescriptorContext^ context, System::Type^ sourceType) override
			{
				if (sourceType = String::typeid)
					return true;
				return System::ComponentModel::TypeConverter::CanConvertFrom(context, sourceType);
			}

			/*
			public: virtual bool CanConvertTo(System::ComponentModel::ITypeDescriptorContext^ context, System::Type^ destinationType) override
			{
			return true;
			}
			*/

	public: virtual System::Object^ ConvertFrom(System::ComponentModel::ITypeDescriptorContext^ context, 
				System::Globalization::CultureInfo^ culture, System::Object^ value) override
			{
				if (value->GetType() == String::typeid)
				{
					array<String^>^ parsed = ((System::String^) value)->Split(gcnew array<Char> {','});
					return gcnew PointF(Single::Parse(parsed[0]), Single::Parse(parsed[1]));
				}
				return System::ComponentModel::TypeConverter::ConvertFrom(context, culture, value);
			}
	};

}; // end glgraphwin
