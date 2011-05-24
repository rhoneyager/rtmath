#pragma once
#include "Plottable.h"

#include "../rtmath-base/matrixop.h"
#include <cmath>

namespace glgraphwin {

	// Use this to plot a square marker at a point
	public ref class Rectangle : public Shape
	{
	public:
		Rectangle(double Xc, double Yc, double Xw, double Yw, double Rotation)
		{
			// First, calculate via rotation where 
			// the points will be

			_rotation = Rotation;
			_x = Xc;
			_y = Yc;

			using namespace rtmath;
			// Going clockwise around the shape in this part
			// Column vectors
			matrixop A(2,2,1), B(2,2,1), C(2,2,1), D(2,2,1);

			A.set(-1.0*Xw/2.0,2,0,0);
			A.set(Yw/2.0,2,1,0);
			B.set(Xw/2.0,2,0,0);
			B.set(Yw/2.0,2,1,0);
			C.set(Xw/2.0,2,0,0);
			C.set(-1.0*Yw/2.0,2,1,0);
			D.set(-1.0*Xw/2.0,2,0,0);
			D.set(-1.0*Yw/2.0,2,1,0);

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

			op = gcnew array<double>(8);
			op[0] = A.get(2,0,0);
			op[1] = A.get(2,1,0);
			op[6] = B.get(2,0,0);
			op[7] = B.get(2,1,0);
			op[4] = C.get(2,0,0);
			op[5] = C.get(2,1,0);
			op[2] = D.get(2,0,0);
			op[3] = D.get(2,1,0);

			// Save the minima and maxima for x and y for autofocusing
			for (unsigned int i=0;i<3;i++)
			{
				// xs
				if (op[2*i] < _min->X) _min->X = (float) op[2*i];
				if (op[2*i] > _max->X) _max->X = (float) op[2*i];
			}
			for (unsigned int i=0;i<3;i++)
			{
				// ys
				if (op[2*i+1] < _min->Y) _min->Y = (float) op[2*i+1];
				if (op[2*i+1] > _max->Y) _max->Y = (float) op[2*i+1];
			}
		}
		virtual void Plot() override;
		//virtual ~markerSquare(void);
	private:
		// Ordered (x,y) points for plotting
		//double op[8];
		array<double> ^op;
	};

}; // end namespace

