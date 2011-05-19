// glgraph-win.cpp : main project file.

#include "stdafx.h"
#include "glgraph-winControl.h"

namespace glgraphwin {

	glgraphwinControl::glgraphwinControl(void)
	{
		//openglform = 0;
		//_runtime = this->Ambient->UserMode;
		InitializeComponent();
		// Should always fail, as designmode is not yet set
		/*
		if (this->Visible && !(this->DesignMode) ) {
		openglform = gcnew glform(this);
		//openglform->Render();
		}
		*/
		//openglform->SwapOpenGLBuffers();
		_overrideui = false;
		mmode = NONE;
		activeCamera = gcnew camera();
		activeCamera->cameraChanged += gcnew CameraChangedEventHandler(this, &glgraphwinControl::activeCamera_Changed);

		_mouseinwindow = false;
	}

	System::Void glgraphwinControl::activeCamera_Changed(System::Object^  sender)
	{
		render();
	}

	void glgraphwinControl::redraw()
	{
		if (this->Visible && !(this->DesignMode) )
		{
			if (openglform)
			{
				delete openglform;
			}
			openglform = gcnew glform(this);
			openglform->activeCamera = activeCamera;
			if (openglform->Render()) 
			{
					timer1->Enabled = true;
			} else {
				timer1->Enabled = false;
			}
		}
		//openglform->SwapOpenGLBuffers();
	}

	void glgraphwinControl::render()
	{
		if (openglform && this->Visible && !(this->DesignMode) )
		{
			if (openglform->Render() ) redraw();
		}
		//openglform->SwapOpenGLBuffers();
	}

	glgraphwinControl::~glgraphwinControl()
	{
		if (components)
		{
			delete components;
		}
		if (openglform)
		{
			delete openglform;
		}
	}

	System::Void glgraphwinControl::glgraphwinControl_VisibleChanged(System::Object^  sender, System::EventArgs^  e) {
		//throw;
		// This doesn't affect invisibility in the standard sense, since the control loads 
		// an imaginary subwindow for the opengl rendering
		bool nowvis = this->Visible;
		if (this->Visible && !(this->DesignMode) )
		{
			redraw();
		} else {
			if (openglform)
			{
				delete openglform;
			}
		}
	}

	System::Void glgraphwinControl::glgraphwinControl_Resize(System::Object^  sender, System::EventArgs^  e)
	{
		redraw();
	}

	System::Void glgraphwinControl::glgraphwinControl_Move(System::Object^  sender, System::EventArgs^  e)
	{
	}

	System::Void glgraphwinControl::glgraphwinControl_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e)
	{
		UNREFERENCED_PARAMETER(sender);
		UNREFERENCED_PARAMETER(e);
		if (openglform)
		{
			openglform->Render();
		}
	}

	System::Void glgraphwinControl::glgraphwinControl_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		// Control is active. Process input.
		// This only gives the ascii character, so alt will not work since it's intercepted by the form
		// TODO: test this
		// For convenience, 
		System::Char key = e->KeyChar;
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseWheel(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		if (!_mouseinwindow) return;
		// Process the scroll event. See mouseclick for event documentation.
		// My delta seems to go in increments of 120
		// Negative delta (scroll up) is zoom in
		// Positive delta (scroll down) is zoom out
		// Each should change the width in 10 percent increments?
		// Delta is a signed Int32
		// System::Drawing::PointF activeCamera->_width;
		System::Drawing::PointF newwidth;
		if (e->Delta > 0)
		{
			newwidth.X = activeCamera->Width.X / ( 1.1f * ( (float) abs(e->Delta) / 120.0f) );
			newwidth.Y = activeCamera->Width.Y / ( 1.1f * ( (float) abs(e->Delta) / 120.0f) );
		} else {
			newwidth.X = activeCamera->Width.X * 1.1f * ( (float) abs(e->Delta) / 120.0f);
			newwidth.Y = activeCamera->Width.Y * 1.1f * ( (float) abs(e->Delta) / 120.0f);
		}
		activeCamera->Width = newwidth;

	}

	System::Void glgraphwinControl::glgraphwinControl_MouseEnter(System::Object^  sender, System::EventArgs^  e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		_mouseinwindow = true;
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseLeave(System::Object^  sender, System::EventArgs^  e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		_mouseinwindow = false;
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		// Control is active. Process input.
		/* mouseeventargs e contains:
		   Button (Left, Middle, Right, None, ...)  System.Windows.Forms.MouseButtons
		   Clicks (number of clicks) Int32
		   Delta (signed count of number of dents wheel is rotated)
		   Location (rel. to top left of form) System.Drawing.Point
		   X and Y
		   */
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		// Control active
		System::Windows::Forms::MouseButtons button = e->Button;
		if (e->Button == System::Windows::Forms::MouseButtons::Left)
		{
			if (mmode == NONE)
			{
				// Begin Panning
				mmode = PANNING;
				// Record mouse position on _control_
				mouserecpos = e->Location;
			}
		} else if (e->Button == System::Windows::Forms::MouseButtons::Middle)
		{
			if (mmode == NONE)
			{
				// Begin zoom
				mmode = ZOOM;
				// Record mouse position on _control_
				mouserecpos = e->Location;
			}
		} else {
			mmode = NONE;
		}
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
	{
		if (mmode != NONE)
		{
			// End panning
			mmode = NONE;
		}
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseCaptureChanged(System::Object^  sender, System::EventArgs^  e)
	{
		// End any panning
		if (mmode != NONE)
			mmode = NONE;
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
	{
		if (_overrideui) return;
		if (this->Enabled == false) return;
		if (!openglform) return;
		// Control active
		// The button mapping is designed for graph panning and zoom
		// Button 1 (Left), when held down, initiates dragging
		// Button 2 (Middle), when held down, changes zoom
		if (e->Button == System::Windows::Forms::MouseButtons::Left)
		{
			if (mmode == PANNING)
			{
				/* Redraw the camera so that the visible frame is
				   dragged along with the mouse
				   Get the direction and amount of motion based on the 
				   difference between the two Locations in pixels and 
				   then convert to appropriate units to move the camera.
				   */
				/* I know the size of the usercontrol in pixels, and I have the camera dimensions. */
				// Map the pixels into x-y coordinates
				//System::Drawing::PointF
				System::Drawing::Point deltaMouse;
				deltaMouse.X = mouserecpos.X - e->Location.X;
				deltaMouse.Y = mouserecpos.Y - e->Location.Y;
				
				System::Drawing::PointF deltaCamera, newCamera;
				// Size.Height and Size.Width help here
				deltaCamera.X = (float) deltaMouse.X * (float) activeCamera->Width.X / (float) Size.Width;
				deltaCamera.Y = (float) deltaMouse.Y * (float) activeCamera->Width.Y / (float) Size.Height;


				newCamera.X = activeCamera->Center.X + deltaCamera.X;
				newCamera.Y = activeCamera->Center.Y - deltaCamera.Y;
				// Change the camera center to compensate for the motion
				activeCamera->Center = newCamera;

				// Record the new mouse position
				mouserecpos = e->Location;
			}
		} else if (e->Button == System::Windows::Forms::MouseButtons::Middle)
		{
			if (mmode == ZOOM)
			{
				System::Drawing::Point deltaMouse;
				deltaMouse.X = mouserecpos.X - e->Location.X;
				deltaMouse.Y = mouserecpos.Y - e->Location.Y;

				// For zoom, I care only about delta Y
				// Y decreasing means zoom out
				// Y increasing means zoom in
				System::Drawing::PointF newwidth;
				if (deltaMouse.Y > 0)
				{
					newwidth.X = activeCamera->Width.X / (1.1 * ( (float) abs(deltaMouse.Y) / 5.0f ) );
					newwidth.Y = activeCamera->Width.Y / (1.1 * ( (float) abs(deltaMouse.Y) / 5.0f ) );
				} else if (deltaMouse.Y < 0) {
					newwidth.X = activeCamera->Width.X * (1.1 * ( (float) abs(deltaMouse.Y) / 5.0f ) );
					newwidth.Y = activeCamera->Width.Y * (1.1 * ( (float) abs(deltaMouse.Y) / 5.0f ) );
				}

				// Record the new mouse position
				mouserecpos = e->Location;
			}
		}
	}

}; // end namespace glgraphwin

/*
		System::Drawing::PointF newwidth;
		if (e->Delta > 0)
		{
			newwidth.X = activeCamera->Width.X / ( 1.1f * ( (float) abs(e->Delta) / 120.0f) );
			newwidth.Y = activeCamera->Width.Y / ( 1.1f * ( (float) abs(e->Delta) / 120.0f) );
		} else {
			newwidth.X = activeCamera->Width.X * 1.1f * ( (float) abs(e->Delta) / 120.0f);
			newwidth.Y = activeCamera->Width.Y * 1.1f * ( (float) abs(e->Delta) / 120.0f);
		}
		activeCamera->Width = newwidth;
		*/