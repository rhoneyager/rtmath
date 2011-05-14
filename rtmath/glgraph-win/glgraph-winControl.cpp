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
			openglform->Render();
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
		if (this->Enabled == false) return;
		if (!openglform) return;
		// Control is active. Process input.
	}

	System::Void glgraphwinControl::glgraphwinControl_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
	{
		if (this->Enabled == false) return;
		if (!openglform) return;
		// Control is active. Process input.
	}

}; // end namespace glgraphwin