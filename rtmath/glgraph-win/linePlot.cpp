#include "stdafx.h"
#include "linePlot.h"
#include "glform.h"

namespace glgraphwin {
	linePlot::linePlot(size_t num, double *x, double *y)
	{
		setData(num,x,y);
		lineColor = gcnew System::Drawing::Color();
	}

	void linePlot::setData(size_t num, double *x, double *y)
	{
		_numPoints = num;
		_x = x;
		_y = y;
		// Save the minima and maxima for x and y for autofocusing
		for (unsigned int i=0; i< (unsigned int) num;i++)
		{
			if (_x[i] < _min->X) _min->X = (float) _x[i];
			if (_x[i] > _max->X) _max->X = (float) _x[i];
			if (_y[i] < _min->Y) _min->Y = (float) _y[i];
			if (_y[i] > _max->Y) _max->Y = (float) _y[i];
		}
	}

	void linePlot::Plot()
	{
		// Check visibility
		if (!_visible) return;

		// This is similar to markerSquare's code
		glBegin(GL_LINE_LOOP);
		//lineColor.Select();
		glColor4d(lineColor->R / 255.0, lineColor->G / 255.0, lineColor->B / 255.0, lineColor->A / 255.0);
		// Plot each vertex
		// Follow order in the array
		// CCL direction does not matter, since no fill is done
		for (size_t i=0; i<_numPoints; i++)
		{
			glVertex2d(_x[i],_y[i]);
		}
		glEnd();
	}

}; // end namespace

