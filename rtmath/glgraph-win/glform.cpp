#include "StdAfx.h"
#include "glform.h"
#include <gl/GL.h>
#include <gl/GLU.h>

namespace glgraphwin {

	glform::glform(System::Windows::Forms::UserControl ^ parent)
	{
		// TODO: Debug this
		initialized = false;
		m_hDC = 0;
		m_hglrc = 0;

		if (!parent) return;
		System::Windows::Forms::Form ^ parform = parent->ParentForm;
		if (!(parent->ParentForm) ) return;
		CreateParams^ cp = gcnew CreateParams;
		// TODO: find location on form of the parent object
		cp->X = 130;
		cp->Y = 80;
		cp->Height = parent->Size.Width;
		cp->Width = parent->Size.Height;

		cp->Parent = parent->ParentForm->Handle;
		cp->Style = WS_CHILD | WS_VISIBLE | 
			WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

		this->CreateHandle(cp);
		// Get the device context
		m_hDC = GetDC((HWND)this->Handle.ToPointer());

		// If a context is found
		if(m_hDC)
		{
			mySetPixelFormat(m_hDC);
			initialized = true;
		}
	}

	System::Void glform::SwapOpenGLBuffers(System::Void)
	{
		SwapBuffers(m_hDC);
	}

	glform::~glform()
	{
		wglMakeCurrent( NULL, NULL );
		wglDeleteContext(m_hglrc);
		ReleaseDC((HWND)this->Handle.ToPointer(), m_hDC);
		this->DestroyHandle();
	}

	GLint glform::mySetPixelFormat(HDC hdc)
	{
		//if (!initialized) return 0;
		PIXELFORMATDESCRIPTOR pfd = { 
			sizeof(PIXELFORMATDESCRIPTOR),    // size of this pfd 
			1,                                // version number 
			PFD_DRAW_TO_WINDOW |              // support window 
			PFD_SUPPORT_OPENGL |              // support OpenGL 
			PFD_DOUBLEBUFFER,                 // double buffered 
			PFD_TYPE_RGBA,                    // RGBA type 
			24,                               // 24-bit color depth 
			0, 0, 0, 0, 0, 0,                 // color bits ignored 
			0,                                // no alpha buffer 
			0,                                // shift bit ignored 
			0,                                // no accumulation buffer 
			0, 0, 0, 0,                       // accum bits ignored 
			32,                               // 32-bit z-buffer     
			0,                                // no stencil buffer 
			0,                                // no auxiliary buffer 
			PFD_MAIN_PLANE,                   // main layer 
			0,                                // reserved 
			0, 0, 0                           // layer masks ignored 
		};
		GLint  iPixelFormat; 

		// get the device context's best, available pixel format match 

		if((iPixelFormat = ChoosePixelFormat(hdc, &pfd)) == 0)
		{
			MessageBox::Show("ChoosePixelFormat Failed");
			return 0;
		}

		// make that match the device context's current pixel format 

		if(SetPixelFormat(hdc, iPixelFormat, &pfd) == FALSE)
		{
			MessageBox::Show("SetPixelFormat Failed");
			return 0;
		}
		SwapBuffers(m_hDC);
			
		unsigned long i = 0;
		if((m_hglrc = wglCreateContext(m_hDC)) == NULL)
		{
			//MessageBox::Show("wglCreateContext Failed");
			DWORD result;
                    result = GetLastError();
					//  error 126
					// module not found
					// occurs when it is too soon after creating a window and
					// getting the device context. this is crap.
			return 0;
			//i++;
			//if (i > 1000000) break;
		}

		wglMakeCurrent( NULL, NULL );

		return 1;
	}

	int glform::Render(System::Void)
	{
		if (!initialized) return 1;
		if (m_hglrc == NULL)
		{
			if((m_hglrc = wglCreateContext(m_hDC)) == NULL)
			{
 				//MessageBox::Show("wglCreateContext Failed");
				return 1;
			}
		}
		if((wglMakeCurrent(m_hDC, m_hglrc)) == NULL)
		{
			MessageBox::Show("wglMakeCurrent Failed");
			wglMakeCurrent( NULL, NULL );
			return 1;
		}

		glClearColor(0.0,0.0,0.0,0.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		SwapBuffers(m_hDC);

		wglMakeCurrent( NULL, NULL );
		return 0;
	}

};
