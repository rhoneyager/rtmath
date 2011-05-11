// glgraph-win.cpp : main project file.

#include "stdafx.h"
#include "glgraph-winControl.h"

namespace glgraphwin {

	glgraphwinControl::glgraphwinControl(void)
	{
		InitializeComponent();
		openglform = gcnew glform(this);
		openglform->Render();
		//openglform->SwapOpenGLBuffers();
	}

	void glgraphwinControl::redraw()
	{
		if (openglform)
		{
			delete openglform;
		}
		openglform = gcnew glform(this);
		openglform->Render();
		//openglform->SwapOpenGLBuffers();
	}

	void glgraphwinControl::render()
	{
		if (openglform->Render() ) redraw();
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

}; // end namespace glgraphwin