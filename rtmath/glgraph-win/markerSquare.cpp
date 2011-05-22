#include "StdAfx.h"
#include "markerSquare.h"
#include "../rtmath-base/matrixop.h"
#include <cmath>
#include "glform.h"

namespace glgraphwin {
	/*
	markerSquare::markerSquare(double X, double Y, double Size, double Rotation)
	{
		// First, calculate via rotation where 
		// the points will be

		using namespace rtmath;
		// Going clockwise around the shape
		// Column vectors
		matrixop A(2,2,1), B(2,2,1), C(2,2,1), D(2,2,1);

		A.set(-1.0*Size/2.0,2,0,0);
		A.set(Size/2.0,2,0,1);
		B.set(Size/2.0,2,0,0);
		B.set(Size/2.0,2,0,1);
		C.set(Size/2.0,2,0,0);
		C.set(-1.0*Size/2.0,2,0,1);
		D.set(-1.0*Size/2.0,2,0,0);
		D.set(-1.0*Size/2.0,2,0,1);

		// Rotation is in radians
		matrixop rot(2,2,2);
		rot.set(cos(Rotation),2,0,0);
		rot.set(cos(Rotation),2,1,1);
		rot.set(sin(Rotation),2,1,0);
		rot.set(-1.0*sin(Rotation),2,0,1);

		matrixop Ar(2,2,1), Br(2,2,1), Cr(2,2,1), Dr(2,2,1);
		Ar = A * rot;
		Br = B * rot;
		Cr = C * rot;
		Dr = D * rot;

		// Good, the rotated points are now known
		// Must do these counterclockwise.....

		op[0] = A.get(2,0,0);
		op[1] = A.get(2,1,0);
		op[6] = B.get(2,0,0);
		op[7] = B.get(2,1,0);
		op[4] = C.get(2,0,0);
		op[5] = C.get(2,1,0);
		op[2] = D.get(2,0,0);
		op[3] = D.get(2,1,0);
	}
	*/
	void markerSquare::Plot()
	{
		// The openGL environment is ready
		// Take the x and y coordinate and add it to the rotated points

		// TODO: correct for the current aspect ratio, so that these display without distortion
		double AR = (double) _controlHeight / (double) _controlWidth;

		// Draw the inside of the shape first
		glBegin(GL_POLYGON);
		bodyColor.Select();
		for (unsigned int i=0;i<4;i++)
		{
			glVertex2d(op[2*i]*AR+_x,op[2*i+1]+_y);
		}
		glEnd();

		// Draw the shape border
		glBegin(GL_LINE_LOOP);
		borderColor.Select();
		for (unsigned int i=0;i<4;i++)
		{
			glVertex2d(op[2*i]*AR+_x,op[2*i+1]+_y);
		}
		glEnd();
	}


}; // end namespace

