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
				_binWidth = (_max->X - _min->X) / (double) _numBins;
				// Do binning
				_binning();
			}
		}
		property double binWidth
		{
			double get() { return _binWidth; }
			void set(double width)
			{
				_binWidth = width;
				double nb = (_max->X - _min->X) / _binWidth;
				// numBins is nb+1, unless nb is an integer
				if (nb - (int) nb > 0) _numBins = (size_t) (nb + 1);
				else _numBins = (size_t) nb;
				// Do binning
				_binning();
			}
		}
	protected:
		void _binning();
		size_t _numPoints;
		size_t _numBins;
		double _binWidth;
		double *_x;
		array<double> ^bins;
	};
	

}; // end glgraphwin

