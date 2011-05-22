#include "stdafx.h"
#include "linePlot.h"
#include "glform.h"

namespace glgraphwin {
	linePlot::linePlot(size_t num, double *x, double *y)
	{
		setData(num,x,y);
	}

	void linePlot::setData(size_t num, double *x, double *y)
	{
		_numPoints = num;
		_x = x;
		_y = y;
	}

	void linePlot::Plot()
	{
		// Check visibility
		if (!_visible) return;

		// This is similar to markerSquare's code
		glBegin(GL_LINE_LOOP);
		lineColor.Select();
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

