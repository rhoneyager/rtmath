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

}; // end glgraphwin
