#include "StdAfx.h"
#include "histiogram.h"
#include <cmath>
#include "glform.h"
#include "Rectangle.h"

namespace glgraphwin {
	histiogram::histiogram(size_t num, double *x)
	{
		lineColor = gcnew System::Drawing::Color();
		barColor = gcnew System::Drawing::Color();
		_numBins = 10;
		_binWidth = 0;
		_min->X = 0;
		_min->Y = 0;
		_max->X = 0;
		_max->Y = 0;
		setData(num,x);
	}

	void histiogram::setData(size_t num, double *x)
	{
		// If numbins is zero, set numBins to 10
		_numPoints = num;
		_x = x;
		// Compute the min and max for x here
		_min->X = (float) _x[0];
		_max->X = (float) _x[0];
		for (unsigned int i=0;i< (unsigned int) num;i++)
		{
			if (_x[i]<_min->X) _min->X = (float) _x[i];
			if (_x[i]>_max->X) _max->X = (float) _x[i];
		}
		// Ugly. Tells numBins to recalculate bin width
		if (_numBins == 0) numBins = 10;
		else numBins = numBins;
	}

	void histiogram::_binning()
	{
		// Take the dataset and count the number of items in 
		// each bin. Plot accordingly
		array<double> ^bins = gcnew array<double>( (int) _numBins);
		double ymax = 0;
		// Take each datum, and fill appropriate bin
		for (size_t i=0;i<_numPoints;i++)
		{
			unsigned int bin = (unsigned int) ((_x[i] - _min->X) / _binWidth);
			bins[bin]++;
		}
		for (unsigned int i=0;i<_numBins;i++)
		{
			if (bins[i] > ymax) ymax = bins[i];
		}
		_max->Y = (float) ymax;
		_min->Y = 0;
	}

	void histiogram::Plot()
	{
		if (!bins) return;
		// Plot the bins
		// OpenGL fills counterclockwise
		for (size_t i=0; i<_numBins;i++)
		{
			// Plot bin i
			// Be lazy and define a Rectangle (similar to markerSquare)
			double Xc = _min->X + ( (double) i*_binWidth) + (0.5*_binWidth);
			double Yc = bins[(int) i] / 2.0;
			double Xw = _binWidth;
			Rectangle^ plotrect = gcnew Rectangle(Xc,Yc,Xw,Yc,0);
			plotrect->Plot();
		}
	}

}; // end glgraphwin

