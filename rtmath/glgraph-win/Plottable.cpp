#include "StdAfx.h"
#include "Plottable.h"
#include "glform.h"

namespace glgraphwin {

	Plottable::Plottable(void)
	{
	}


	Plottable::~Plottable(void)
	{
	}

	void Plottable::setAspectRatio(unsigned int width, unsigned int height)
	{
		// Important for drawing markers, which should be displayed in the correct ratio
		_controlWidth = width;
		_controlHeight = height;
	}
	/*
	Color::Color()
	{
		_r = 0;
		_g = 0;
		_b = 0;
		_a = 0;
	}

	Color::Color(double r, double g, double b, double a)
	{
		Set(r,g,b,a);
	}

	void Color::Set(double r, double g, double b, double a)
	{
		_r = r;
		_g = g;
		_b = b;
		_a = a;
	}

	void Color::Select()
	{
		// Assume correct opengl mode
		glColor4d(_r, _g, _b, _a);
	}
	*/
	Shape::Shape(double X, double Y, double Size, double Rotation)
	{
		_x = X;
		_y = Y;
		_size = Size;
		_rotation = Rotation;
		bodyColor = gcnew System::Drawing::Color();
		borderColor = gcnew System::Drawing::Color();
	}

	Shape::Shape()
	{
		_x = 0;
		_y = 0;
		_size = 0;
		_rotation = 0;
		bodyColor = gcnew System::Drawing::Color();
		borderColor = gcnew System::Drawing::Color();
	}


}; // end namespace

