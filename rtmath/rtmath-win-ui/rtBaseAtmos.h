#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;


namespace rtmathwinui {

	public ref class BaseAtmosTypeConverter : System::ComponentModel::StringConverter
	{
	public: virtual bool GetStandardValuesSupported(System::ComponentModel::ITypeDescriptorContext^ context) override
			{
				return true;
			}

	public: virtual System::ComponentModel::TypeConverter::StandardValuesCollection^ 
				GetStandardValues(System::ComponentModel::ITypeDescriptorContext^ context) override
			{
				array<System::String^ >^ names = gcnew array<System::String^> {"Test 1", "Test 2"};
				return gcnew System::ComponentModel::TypeConverter::StandardValuesCollection(
					names);
			}
	};

	public ref class SpecUnitsTypeConverter : System::ComponentModel::StringConverter
	{
	public: virtual bool GetStandardValuesSupported(System::ComponentModel::ITypeDescriptorContext^ context) override
			{
				return true;
			}

	public: virtual System::ComponentModel::TypeConverter::StandardValuesCollection^ 
				GetStandardValues(System::ComponentModel::ITypeDescriptorContext^ context) override
			{
				array<System::String^ >^ names = gcnew array<System::String^> {"Wavenumber (cm^-1)", "Wavelength (m)", "Frequency (Hz)"};
				return gcnew System::ComponentModel::TypeConverter::StandardValuesCollection(
					names);
			}
	};

	public ref class baseAtmos :  public System::ComponentModel::Component
	{
	public:
		baseAtmos(void);
		baseAtmos(System::ComponentModel::IContainer ^container);
	protected:
		~baseAtmos();
	private:
	private:
		System::ComponentModel::Container ^components;
#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			components = gcnew System::ComponentModel::Container();
		}
#pragma endregion
	};
}
