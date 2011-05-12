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

}; // end namespace glgraphwin