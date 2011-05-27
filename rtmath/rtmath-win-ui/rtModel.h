#pragma once
#include "rtBaseAtmos.h"
using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;

namespace rtmathwinui {
	public ref class rtModel :  public System::ComponentModel::Component
	{
	public:
		rtModel(void)
		{
			//_baseAtmosphere = gcnew String();
			InitializeComponent();
			_baseAtmosphereName = gcnew String("");
			_specUnits = gcnew String("");
			_specMin = 0;
			_specMax = 0;
		}
		rtModel(System::ComponentModel::IContainer ^container)
		{
			container->Add(this);
			InitializeComponent();
		}

	private:
		System::String^ _baseAtmosphereName;
		//System::String^ _surfaceModel;
		System::String^ _specUnits;
		double _specMin, _specMax;
	public:
		
		[CategoryAttribute("Base Settings"),
		DescriptionAttribute("Select the base atmospheric model from the list. This list includes all base atmospheres that have been defined."),
		TypeConverter(BaseAtmosTypeConverter::typeid)]
		property System::String^ baseAtmosphere
		{
			System::String^ get()
			{
				return _baseAtmosphereName;
			}
			void set(System::String^ value)
			{
				_baseAtmosphereName = value;
			}
		}
		
		[CategoryAttribute("Spectral Region"),
		DescriptionAttribute("Select the appropriate units for the spectral region"),
		TypeConverter(SpecUnitsTypeConverter::typeid)]
		property System::String^ spectralUnits
		{
			System::String^ get()
			{
				return _specUnits;
			}
			void set(System::String^ value)
			{
				// TODO: Do conversion of min and max from old units to new units
				_specUnits = value;
			}
		}

		[CategoryAttribute("Spectral Region"),
		DescriptionAttribute("Select minimum")]
		property double specMin
		{
			double get()
			{
				return _specMin;
			}
			void set(double value)
			{
				_specMin = value;
			}
		}

		[CategoryAttribute("Spectral Region"),
		DescriptionAttribute("Select maximum")]
		property double specMax
		{
			double get()
			{
				return _specMax;
			}
			void set(double value)
			{
				_specMax = value;
			}
		}

	protected:
		~rtModel()
		{
			if (components)
			{
				delete components;
			}
		}
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

	//public ref class 
}
