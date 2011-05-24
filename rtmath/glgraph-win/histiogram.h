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
	
	public ref class histiogram abstract : public Plottable
	{
	public:
		histiogram(size_t num, double *x);
		virtual void Plot() override;
		System::Drawing::Color^ lineColor;
		System::Drawing::Color^ barColor;
		void setData(size_t num, double *x);
		property size_t numBins
		{
			size_t get() {return _numBins;}
			void set(size_t num) 
			{ 
				_numBins = num;
				// Find max and min x. Don't assume ordering.
				// TODO: put max/min in seperate function
				double min = _x[0], max = _x[0];
				for (size_t i=0;i<_numPoints;i++)
				{
					if (_x[i]<min) min = _x[i];
					if (_x[i]>max) max = _x[i];
				}
				_min = min;
				_max = max;
				_binWidth = (max - min) / (double) _numBins;
			}
		}
		property double binWidth
		{
			double get() { return _binWidth; }
			void set(double width)
			{
				_binWidth = width;
				// Find max / min
				double min = _x[0], max = _x[0];
				for (size_t i=0;i<_numPoints;i++)
				{
					if (_x[i]<min) min = _x[i];
					if (_x[i]>max) max = _x[i];
				}
				_min = min;
				_max = max;
				double nb = (max - min) / _binWidth;
				// numBins is nb+1, unless nb is an integer
				if (nb - (int) nb > 0) _numBins = (size_t) (nb + 1);
				else _numBins = (size_t) nb;
			}
		}
	protected:
		size_t _numPoints;
		size_t _numBins;
		double _binWidth;
		double *_x;
		double _min;
		double _max;
	};
	

}; // end glgraphwin

