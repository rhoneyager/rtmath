#pragma once
#include "Plottable.h"

#include "../rtmath-base/matrixop.h"
#include <cmath>

namespace glgraphwin {

	// Use this to plot a square marker at a point
	class markerSquare : public Shape
	{
	public:
		markerSquare(double X, double Y, double Size, double Rotation) 
			: Shape(X,Y,Size,Rotation)
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
			Ar = rot * A;
			Br = rot * B;
			Cr = rot * C;
			Dr = rot * D;

			// Good, the rotated points are now known
			// Must do these counterclockwise.....

			op[0] = A.get(2,0,0);
			op[1] = A.get(2,0,1);
			op[6] = B.get(2,0,0);
			op[7] = B.get(2,0,1);
			op[4] = C.get(2,0,0);
			op[5] = C.get(2,0,1);
			op[2] = D.get(2,0,0);
			op[3] = D.get(2,0,1);
		}
		virtual void Plot();
		//virtual ~markerSquare(void);
	private:
		// Ordered (x,y) points for plotting
		double op[8];
	};

}; // end namespace

