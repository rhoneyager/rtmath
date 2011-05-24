#include "StdAfx.h"
//#include <gl/GL.h>
//#include <gl/GLU.h>
#include "camera.h"

namespace glgraphwin {

		// void camera::Render(); // provided elsewhere
		void camera::panLeft()
		{
			// Move left by 5% of Width.X
			System::Drawing::PointF newcenter;
			newcenter.Y = Center.Y;
			newcenter.X = Center.X - (0.05f * Width.X);
			Center = newcenter;
		}

		void camera::panRight()
		{
			// Move right by 5% of Width.X
			System::Drawing::PointF newcenter;
			newcenter.Y = Center.Y;
			newcenter.X = Center.X + (0.05f * Width.X);
			Center = newcenter;
		}

		void camera::panUp()
		{
			// Move up by 5% of Width.Y
			System::Drawing::PointF newcenter;
			newcenter.X = Center.X;
			newcenter.Y = Center.Y + (0.05f * Width.Y);
			Center = newcenter;
		}

		void camera::panDown()
		{
			// Move down by 5% of Width.Y
			System::Drawing::PointF newcenter;
			newcenter.X = Center.X;
			newcenter.Y = Center.Y - (0.05f * Width.Y);
			Center = newcenter;
		}

		void camera::zoomOut()
		{
			System::Drawing::PointF newwidth;
			newwidth.X = Width.X / 1.1f;
			newwidth.Y = Width.Y / 1.1f;
			Width = newwidth;
		}

		void camera::zoomIn()
		{
			System::Drawing::PointF newwidth;
			newwidth.X = Width.X * 1.1f;
			newwidth.Y = Width.Y * 1.1f;
			Width = newwidth;
		}

		void camera::autoFocus(System::Collections::Generic::List<Plottable^>^ plotObjects)
		{
			// For this, take each plotObject in the list and find the minima and maxima for x and y
			// Then, set the camera so that these bounds are enclosed, with a good (~10%) margin

			// This typically takes the list of all visible entries in the glform plotobjects list
			System::Drawing::PointF newmin, newmax;
			bool init = false;
			for (int i=0;i<plotObjects->Count;i++)
			{
				if (plotObjects[i]->Visible)
				{
					if (!init)
					{
						newmin.X = plotObjects[i]->min->X;
						newmin.Y = plotObjects[i]->min->Y;
						newmax.X = plotObjects[i]->max->X;
						newmax.Y = plotObjects[i]->max->Y;
						init = true;
					} else {
						if (plotObjects[i]->min->X < newmin.X) newmin.X = plotObjects[i]->min->X;
						if (plotObjects[i]->min->Y < newmin.Y) newmin.Y = plotObjects[i]->min->Y;
						if (plotObjects[i]->max->X > newmax.X) newmax.X = plotObjects[i]->max->X;
						if (plotObjects[i]->max->Y > newmax.Y) newmax.Y = plotObjects[i]->max->Y;
					}
				}
			}
			// Next, give a margin to the graph by setting the new min and max, then tweaking widths
			Min = newmin;
			Max = newmax;
			Width.X = Width.X * 1.1f;
			Width.Y = Width.Y * 1.1f;
		}

}; // end glgraphwin
