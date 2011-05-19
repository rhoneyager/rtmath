#pragma once
/* pointfconverter - provides the necessary TypeConverter for PointF so that 
it can be edited in the properties box!
*/

// Inspired by http://msdn.microsoft.com/en-us/library/ayybcxe5(v=VS.100).aspx#Y400
// Original in C#, but I converted it to C++/CLI (which Microsoft ignores...) and added CanConvertTo

using namespace System;
using namespace System::Drawing;
using namespace System::ComponentModel;

namespace glgraphwin {

	public ref class PointFTypeConverter : System::ComponentModel::ExpandableObjectConverter
	{
	public: virtual bool CanConvertFrom(System::ComponentModel::ITypeDescriptorContext^ context, System::Type^ sourceType) override
			{
				if (sourceType = String::typeid)
					return true;
				if (sourceType = PointFTypeConverter::typeid)
					return true;
				return System::ComponentModel::TypeConverter::CanConvertFrom(context, sourceType);
			}

			
	public: virtual bool CanConvertTo(System::ComponentModel::ITypeDescriptorContext^ context, System::Type^ destinationType) override
			{
				if (destinationType = String::typeid)
					return true;
				if (destinationType = PointFTypeConverter::typeid)
					return true;
				return System::ComponentModel::TypeConverter::CanConvertTo(context, destinationType);
			}

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

	public: virtual System::Object^ ConvertTo(System::ComponentModel::ITypeDescriptorContext^ context, 
				System::Globalization::CultureInfo^ culture, System::Object^ value, System::Type^ destinationType) override
			{
				if (destinationType = String::typeid)
				{
					// in cs, this is return ((PointF) value)->X + ", " + ((PointF) value)->Y;
					return String::Concat( ((PointF^) value)->X, ", ", ((PointF^) value)->Y);
				}
				return System::ComponentModel::TypeConverter::ConvertTo(context, culture, value, destinationType);
			}
	};

}; // end glgraphwin
