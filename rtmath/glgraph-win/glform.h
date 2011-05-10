#pragma once
#include <Windows.h>
#include <gl/GL.h>

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;
using namespace System::Windows::Forms;

namespace glgraphwin {

	/// <summary>
	/// The wrapper for the form object to get a window handle for opengl
	/// </summary>
	public ref class glform :  public System::Windows::Forms::NativeWindow
	{
	public:
		glform(System::Windows::Forms::Form ^ parentForm,
			GLsizei iWidth, GLsizei iHeight)
		{
			if (!parentForm) return;
			CreateParams^ cp = gcnew CreateParams;
			cp->X = 100;
			cp->Y = 100;
			cp->Height = iWidth;
			cp->Width = iHeight;
			
			cp->Parent = parentForm->Handle;
			cp->Style = WS_CHILD | WS_VISIBLE | 
				WS_CLIPSIBLINGS | WS_CLIPCHILDREN;

			this->CreateHandle(cp);
			m_hDC = GetDC((HWND)this->Handle.ToPointer());

			if(m_hDC)
				MySetPixelFormat(m_hDC);

		}

		virtual System::Void Render(System::Void)
		{
			glClearColor(0.0,0.0,0.0,0.0);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		}

		System::Void SwapOpenGLBuffers(System::Void)
		{
			SwapBuffers(m_hDC);
		}

	protected:
		~glform()
		{
			this->DestroyHandle();
		}

		
		GLint MySetPixelFormat(HDC hdc)
		{
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
    
			if((m_hglrc = wglCreateContext(m_hDC)) == NULL)
			{
				MessageBox::Show("wglCreateContext Failed");
				return 0;
			}
        
			if((wglMakeCurrent(m_hDC, m_hglrc)) == NULL)
			{
				MessageBox::Show("wglMakeCurrent Failed");
				return 0;
			}
    
			return 1;
		}


	private:
		HDC m_hDC;
		HGLRC m_hglrc;
	};
}
