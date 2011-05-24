#pragma once
#include "Plottable.h"

namespace glgraphwin {

	// Use this to plot a series of connected lines at a point
	public ref class linePlot abstract : public Plottable
	{
	public:
		linePlot(size_t num, double *x, double *y);
		virtual void Plot() override;
		System::Drawing::Color^ lineColor;
		void setData(size_t num, double *x, double *y);
	protected:
		size_t _numPoints;
		double *_x;
		double *_y;
	};




}; // end glgraphwin

