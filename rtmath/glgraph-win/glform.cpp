#include "StdAfx.h"
#include "glform.h"
#include <gl/GL.h>
#include <gl/GLU.h>
#include "markerSquare.h"
#include "fontlabel.h"

namespace glgraphwin {

	glform::glform(System::Windows::Forms::UserControl ^ parent)
	{
		initialized = false;
		glstarted = false;
		m_hDC = 0;
		m_hglrc = 0;

		if (!parent) return;
		System::Windows::Forms::Form ^ parform = parent->ParentForm;
		//plotObjects = gcnew System::Collections::Generic::List<Plottable^>; // Will be provided by parent
		if (!(parent->ParentForm) ) return;
		// Get the camera from the parent form
		cp = gcnew CreateParams;
		// TODO: find location on form of the parent object
		//System.Drawing::Point loc;
		//loc = parent->Location;
		// wpf only cp->X = parent->TranslatePoint(new Point(0,0), parent->ParentForm).X;
		//cp->X = parent->Location.X;
		//cp->Y = parent->Location.Y;
		// wpf onlycp->Y = parent->TranslatePoint(new Point(0,0), parent->ParentForm).Y;
		using namespace System::Drawing;
		Point screenpt;
		Point ^ wndpt;
		screenpt = parent->PointToScreen(Point(0,0));
		wndpt = parent->ParentForm->PointToClient(screenpt);
		cp->X = wndpt->X;
		cp->Y = wndpt->Y;
		cp->Height = parent->Size.Height;
		cp->Width = parent->Size.Width;

		cp->Parent = parent->ParentForm->Handle;
		/*
		cp->Style = WS_CHILD | WS_VISIBLE | 
			WS_CLIPSIBLINGS | WS_CLIPCHILDREN;
			*/
		// Allow sibling clipping - otherwise it ends up behind any controls,
		// such as system tabs or frames
		cp->Style = WS_CHILD | WS_VISIBLE | WS_CLIPCHILDREN;

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
			
		/*
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
		*/
		wglMakeCurrent( NULL, NULL );

		return 1;
	}

	int glform::startGL()
	{
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-.5, .5, .5, -.5, -1000, 1000);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

		/* Test plotting by adding a marker
		markerSquare *testm = new markerSquare(0,0,1,0);
		testm->bodyColor.Set(1.0,0,0,0.1);
		(plotObjects)->insert(testm);
		*/

		return 0;
	}

	void glform::draw()
	{
		// Plot all items in plotObjects
		/* C++ class implementation 
		std::set<Plottable*>::const_iterator it;
		for (it = (plotObjects)->begin(); it != (plotObjects)->end(); it++)
		{
			(*it)->setAspectRatio(cp->Width, cp->Height);
			(*it)->Plot();
		}
		*/
		// System::Collections::Generic::List<Plottable^>^ plotObjects;
		for each (Plottable^ obj in plotObjects)
		{
			obj->setAspectRatio(cp->Width, cp->Height);
			obj->Plot();
		}
		/* Example of some lines
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0,0.0,0.0);
		glVertex3f(-10.0,0.0,-30.0);
		glVertex3f(10.0,0.0,-30.0);
		glEnd();
		glBegin(GL_LINE_STRIP);
		glColor3f(0.0,0.0,1.0);
		glVertex3f(0.0,-10.0,-30.0);
		glVertex3f(0.0,10.0,-30.0);
		glEnd();
		glBegin(GL_LINE_STRIP);
		glColor3f(1.0,0.0,1.0);
		glVertex3f(1.0,-1.0,-30.0);
		glVertex3f(1.0,1.0,-30.0);
		glEnd();
		*/
		// Just a test - plot some text

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
			//MessageBox::Show("wglMakeCurrent Failed");
			wglMakeCurrent( NULL, NULL );
			return 1;
		}

		if (!glstarted) startGL();

		glClearColor(1.0,1.0,1.0,1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


		//glTranslatef(-0.25f, -0.10f, 0.0f);
		//glScalef(0.75f, 1.15f, 0.0f);
		// Have the camera calculate and set the correct translate and scale matrices
		if (activeCamera) activeCamera->Render();
		draw();
		glFlush();

		glPopMatrix();

		// Rendering done. Now, swap buffers and release window
		SwapBuffers(m_hDC);

		wglMakeCurrent( NULL, NULL );
		return 0;
	}

	void camera::Render(void)
	{
		// First, set glOrtho for 2d viewing
		
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-1.0*(_width.X)/2.0,
			(_width.X)/2.0,
			-1.0*(_width.Y)/2.0,
			1.0*(_width.Y)/2.0,
			-100,100);
		glMatrixMode(GL_MODELVIEW);
		glTranslatef((GLfloat) -1.0*_center.X, (GLfloat) -1.0*_center.Y,0.0);
		glPushMatrix();
		
	}
};
