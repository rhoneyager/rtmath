#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;


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
			this->_width.X = 10;
			this->_width.Y = 10;
			this->_min.X = -5;
			this->_max.X = 5;
			this->_min.Y = -5;
			this->_max.Y = 5;
		}
		camera(System::ComponentModel::IContainer ^container)
		{
			/// <summary>
			/// Required for Windows.Forms Class Composition Designer support
			/// </summary>

			container->Add(this);
			InitializeComponent();
			this->_width.X = 10;
			this->_width.Y = 10;
			this->_min.X = -5;
			this->_max.X = 5;
			this->_min.Y = -5;
			this->_max.Y = 5;
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
		System::Drawing::Point _center;
		System::Drawing::Point _min;
		System::Drawing::Point _max;
		System::Drawing::Point _width;
		System::Drawing::Point _scale;
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
			TypeConverter(typeof(PointConverter))]
		public System::Drawing::PointF test {
			get { return _test; }
			set { _test = value; }
		}
		*/

		property System::Drawing::Point Center
		{
			System::Drawing::Point get()
			{
				return _center;
			}
			void set(System::Drawing::Point value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_min.X = value.X - (_width.X / 2);
				_max.X = value.X + (_width.X / 2);
				_min.Y = value.Y - (_width.Y / 2);
				_max.Y = value.Y + (_width.Y / 2);
				_center = value;
				OnCameraChanged();
			}
		}

		property System::Drawing::Point Min
		{
			System::Drawing::Point get()
			{
				return _min;
			}
			void set(System::Drawing::Point value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_min = value;
				_center.X = (_min.X / 2) + (_max.X / 2);
				_center.Y = (_min.Y / 2) + (_max.Y / 2);
				_width.X = _max.X - _min.X;
				_width.Y = _max.Y - _min.Y;
				// TODO: recalculate scale
				OnCameraChanged();
			}
		}

		property System::Drawing::Point Max
		{
			System::Drawing::Point get()
			{
				return _max;
			}
			void set(System::Drawing::Point value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_max = value;
				_center.X = (_min.X / 2) + (_max.X / 2);
				_center.Y = (_min.Y / 2) + (_max.Y / 2);
				_width.X = _max.X - _min.X;
				_width.Y = _max.Y - _min.Y;
				// TODO: recalculate scale
				OnCameraChanged();
			}
		}

		property System::Drawing::Point Width
		{
			System::Drawing::Point get()
			{
				return _width;
			}
			void set(System::Drawing::Point value)
			{
				// Recalculate the _min and _max
				// Given the current width and scale
				_width = value;
				_max.X = _center.X + (_width.X / 2);
				_max.Y = _center.Y + (_width.Y / 2);
				_min.X = _center.X - (_width.X / 2);
				_min.Y = _center.Y - (_width.Y / 2);
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
