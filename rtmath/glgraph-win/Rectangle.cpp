#include "StdAfx.h"
#include "Rectangle.h"
#include "../rtmath/matrixop.h"
#include <cmath>
#include "glform.h"

namespace glgraphwin {

	void Rectangle::Plot()
	{
		if (!_visible) return;
		// The openGL environment is ready
		// Take the x and y coordinate and add it to the rotated points

		// correct for the current aspect ratio, so that these display without distortion
		//double AR = (double) _controlHeight / (double) _controlWidth;
		// No aspect ratio correction
		double AR = 1.0;

		// Draw the inside of the shape first
		glBegin(GL_POLYGON);
		//bodyColor.Select();
		glColor4d(bodyColor->R / 255.0, bodyColor->G / 255.0, bodyColor->B / 255.0, bodyColor->A / 255.0);
		for (unsigned int i=0;i<4;i++)
		{
			glVertex2d(op[2*i]*AR+_x,op[2*i+1]+_y);
		}
		glEnd();

		// Draw the shape border
		glBegin(GL_LINE_LOOP);
		//borderColor.Select();
		glColor4d(borderColor->R / 255.0, borderColor->G / 255.0, borderColor->B / 255.0, borderColor->A / 255.0);
		for (unsigned int i=0;i<4;i++)
		{
			glVertex2d(op[2*i]*AR+_x,op[2*i+1]+_y);
		}
		glEnd();
	}


}; // end namespace

