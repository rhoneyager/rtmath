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
		glform(System::Windows::Forms::UserControl ^ parent);
		virtual int Render(System::Void);
	protected:
		System::Void SwapOpenGLBuffers(System::Void);
	protected:
		~glform();
		GLint mySetPixelFormat(HDC hdc);
	private:
		HDC m_hDC;
		HGLRC m_hglrc;
		bool initialized;
	};
}
