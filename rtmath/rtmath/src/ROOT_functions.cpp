#include "../rtmath/Stdafx.h"
#include <cmath>
#include <memory>
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <TGraph.h>
#include <TF1.h>
#include <TCanvas.h>
#include <TAxis.h>
#include <TGaxis.h>
#include <TNamed.h>
#include <TGraph2D.h>
#include <TStyle.h>
#include <TH2.h>
#include "../rtmath/ROOT_functions.h"
#include "../rtmath/error/error.h"

namespace rtmath {

	namespace ROOT_funcs {

		// flip_axis inverts one of the axes for plotting. 
		// Based on http://root.cern.ch/root/roottalk/roottalk08/0214.html
		// Needs to reverse the axis and flip the graph
		void flip_axis(TGraph *g, AXIS a)
		{
			// Select the axis
			TAxis *axis;
			TGaxis *newaxis;
			switch (a)
			{
			case X:
				axis = g->GetXaxis();
				throw rtmath::debug::xBadInput("Bad graph (x) specified to be flipped");
				break;
			case Y:
				axis = g->GetYaxis();
				break;
			default: // Covers Z case, which is not in a 2d graph
				throw rtmath::debug::xBadInput("Bad axis specified to be flipped");
			}
			//axis->SetLabelOffset(999);
			

			// Redraw the new axis
			gPad->Update();
			//newaxis = new TGaxis(xmin,ymin,xmax,ymax,wmin,wmax,ndiv,chopt,gridlen);
			newaxis = new TGaxis();
			newaxis->ImportAxisAttributes(axis);
			newaxis->SetDrawOption(axis->GetDrawOption());
			newaxis->SetOption(axis->GetOption());
			newaxis->SetX1(gPad->GetUxmin());
			newaxis->SetX2(gPad->GetUxmin()-0.001);
			newaxis->SetY1(gPad->GetUymax());
			newaxis->SetY2(gPad->GetUymin());
			newaxis->SetWmin(axis->GetXmin());
			newaxis->SetWmax(axis->GetXmax());
			newaxis->SetNdivisions(axis->GetNdivisions());
			newaxis->SetLabelOffset(axis->GetLabelOffset());
			//newaxis->se
			axis->SetLabelOffset(999);
			axis->SetTitle("");
			axis->SetTickLength(0);

			newaxis->Draw();
			/*newaxis = new TGaxis(
				gPad->GetUxmin(), 
				gPad->GetUymax(), 
				gPad->GetUxmin()-0.001,
				gPad->GetUymin(), 
				axis->GetXmin(),
				axis->GetXmax(),
				axis->GetNdivisions(), 
				axis->GetDrawOption()
				);
			newaxis->SetLabelOffset( (Float_t) -0.04);
			newaxis->Draw();

			
			// Now, proceed to flip the graph
			Int_t n = g->GetN();
			Double_t *x = g->GetX();
			Double_t *y = g->GetY();
			Double_t *yr = new double[n];
			//Double_t yr[100];
			Double_t dy; 
			switch (a)
			{
			case X:
				throw rtmath::debug::xBadInput("Bad graph specified to be flipped");
				break;
			case Y:
				dy = g->GetYaxis()->GetXmin()+g->GetYaxis()->GetXmax();
				break;
			}
			for (Int_t i=0; i<n; i++) {
				yr[i] = -y[i]+dy;
			}
   
			TGraph* gr = new TGraph(n,x,yr);
			gr->SetMarkerStyle(g->GetMarkerStyle());
			gr->SetLineColor(g->GetLineColor());
			gr->SetMarkerColor(g->GetMarkerColor());
			gr->Draw(g->GetDrawOption());
			delete[] yr;
			*/
		}

	} // end ROOT namespace

} // end rtmath


