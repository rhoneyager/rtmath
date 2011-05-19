#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;

#include "pointfconverter.h"

namespace glgraphwin {
	/// <summary>
	/// Summary for camera
	/// </summary>
	public delegate void CameraChangedEventHandler(System::Object^  sender);

	public ref class camera :  public System::ComponentModel::Component
	{
	public:
		camera(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			this->_width.X = 10.0f;
			this->_width.Y = 10.0f;
			this->_min.X = -5.0f;
			this->_max.X = 5.0f;
			this->_min.Y = -5.0f;
			this->_max.Y = 5.0f;
		}
		camera(System::ComponentModel::IContainer ^container)
		{
			/// <summary>
			/// Required for Windows.Forms Class Composition Designer support
			/// </summary>

			container->Add(this);
			InitializeComponent();
			this->_width.X = 10.0f;
			this->_width.Y = 10.0f;
			this->_min.X = -5.0f;
			this->_max.X = 5.0f;
			this->_min.Y = -5.0f;
			this->_max.Y = 5.0f;
		}
		// The rendering function which sends the correct openGL commands to the plot
		void Render();
		// Event when camera is updated
	public: event CameraChangedEventHandler^ cameraChanged;

	protected: virtual System::Void OnCameraChanged()
		{
			cameraChanged(this);
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~camera()
		{
			if (components)
			{
				delete components;
			}
		}

	private:
		// Some basic camera properties
		System::Drawing::PointF _center;
		System::Drawing::PointF _min;
		System::Drawing::PointF _max;
		System::Drawing::PointF _width;
		System::Drawing::PointF _scale;
		//System::Drawing::PointF _test;
		// Reverse axis flag
		/*
		xybool _rev;
		public:
		property xybool reverse
		{
		xybool get()
		{
		return _rev;
		}
		void set(xybool value)
		{
		_rev = value;
		}
		}
		*/
	public:
		/*
		System::Drawing::PointF _test;
		[DesignerSerializationVisibility(DesignerSerializationVisibility.Content), 
			EditorBrowsable(EditorBrowsableState.Advanced), 
			TypeConverter(typeof(PointFConverter))]
		public System::Drawing::PointFF test {
			get { return _test; }
			set { _test = value; }
		}
		*/

		[DescriptionAttribute("The center of the graph"),
		TypeConverter(PointFTypeConverter::typeid)]
		property System::Drawing::PointF Center
		{
			System::Drawing::PointF get()
			{
				return _center;
			}
			void set(System::Drawing::PointF value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_min.X = value.X - (_width.X / 2.0f);
				_max.X = value.X + (_width.X / 2.0f);
				_min.Y = value.Y - (_width.Y / 2.0f);
				_max.Y = value.Y + (_width.Y / 2.0f);
				_center = value;
				OnCameraChanged();
			}
		}

		[DescriptionAttribute("The minimum X and Y values on the graph (bottom-left corner)"),
		TypeConverter(PointFTypeConverter::typeid)]
		property System::Drawing::PointF Min
		{
			System::Drawing::PointF get()
			{
				return _min;
			}
			void set(System::Drawing::PointF value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_min = value;
				_center.X = (_min.X / 2.0f) + (_max.X / 2.0f);
				_center.Y = (_min.Y / 2.0f) + (_max.Y / 2.0f);
				_width.X = _max.X - _min.X;
				_width.Y = _max.Y - _min.Y;
				// TODO: recalculate scale
				OnCameraChanged();
			}
		}

		[DescriptionAttribute("The maximum X and Y values on the graph (top-right corner)"),
		TypeConverter(PointFTypeConverter::typeid)]
		property System::Drawing::PointF Max
		{
			System::Drawing::PointF get()
			{
				return _max;
			}
			void set(System::Drawing::PointF value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_max = value;
				_center.X = (_min.X / 2.0f) + (_max.X / 2.0f);
				_center.Y = (_min.Y / 2.0f) + (_max.Y / 2.0f);
				_width.X = _max.X - _min.X;
				_width.Y = _max.Y - _min.Y;
				// TODO: recalculate scale
				OnCameraChanged();
			}
		}

		[DescriptionAttribute("The width of the graph (in X and Y coordinates)"),
		TypeConverter(PointFTypeConverter::typeid)]
		property System::Drawing::PointF Width
		{
			System::Drawing::PointF get()
			{
				return _width;
			}
			void set(System::Drawing::PointF value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_width = value;
				_max.X = _center.X + (_width.X / 2.0f);
				_max.Y = _center.Y + (_width.Y / 2.0f);
				_min.X = _center.X - (_width.X / 2.0f);
				_min.Y = _center.Y - (_width.Y / 2.0f);
				// TODO: recalculate scale
				OnCameraChanged();
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
}
