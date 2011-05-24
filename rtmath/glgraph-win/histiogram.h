#pragma once
#include "Plottable.h"

/* The class necessary to produce histiograms of 
   data. I added this to take care of Ahlquist's 
   homework assignments. It reads the data pointers,
   and the data plotted reflects the user's choice of 
   either bin width or number of bins.
   Users can set the color of the whole histiogram or 
   can select the color for an individual column.

   The histiogram needs to override default axis plotting 
   to properly identify bins. This will come later.
   */

namespace glgraphwin {
	/*
	public ref class histiogram : public Plottable
	{
	public:
		histiogram(size_t num, double *x, double *y);
		virtual void Plot();
		Color lineColor;
		Color barColor;
		void setData(size_t num, double *x, double *y);
		void numBins(unsigned int bins);
		unsigned int numBins();
		void binWidth(double width);
		double binWidth();
	protected:
		size_t _numPoints;
		double *_x;
		double *_y;
	};
	*/

}; // end glgraphwin

