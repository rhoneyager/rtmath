#pragma once
#include <Windows.h>
#include <gl/GL.h>

namespace glgraphwin {

	public ref class Plottable abstract : public System::ComponentModel::Component
	{
	public:
		Plottable(void);
		virtual ~Plottable(void);
		virtual void Plot() = 0;
		property bool Visible
		{
			bool get() { return _visible; }
			void set(bool newvis) {_visible = newvis; }
		}
		// Read-only property giving the min and max x and y coords. that an object 
		// covers for autofocusing purposes
		property System::Drawing::PointF^ min
		{
			System::Drawing::PointF^ get() { return _min;}
		}
		property System::Drawing::PointF^ max
		{
			System::Drawing::PointF^ get() { return _max;}
		}
		property HDC hDC
		{
			virtual void set(HDC hDC) { m_hDC = hDC; }
		}
		void setAspectRatio(unsigned int width, unsigned int height);
	protected:
		bool _visible;
		unsigned int _controlWidth;
		unsigned int _controlHeight;
		System::Drawing::PointF^ _min;
		System::Drawing::PointF^ _max;
		HDC m_hDC;
		HGLRC m_hglrc;
	};
	/*
	public ref class Color : public System::ComponentModel::Component
	{
	public:
		Color(double r, double g, double b, double a);
		Color();
		void Set(double r, double g, double b, double a);
		void Select();
	private:
		double _r, _g, _b, _a;
	};
	*/
	public ref class Shape abstract : public Plottable
	{
	public:
		Shape(double X, double Y, double Size, double Rotation);
		Shape();
		//virtual void Plot() = 0 override;
		// Colors are extracted as 32-bit integers
		System::Drawing::Color^ borderColor;
		System::Drawing::Color^ bodyColor;
	protected:
		double _x, _y, _size, _rotation;
	};

}; // end namespace
