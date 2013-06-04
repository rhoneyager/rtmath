#pragma once
#include <Windows.h>
#include <gl/GL.h>
#include <set>

#include "camera.h"
#include "Plottable.h"
//#include "embedded.h"

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
		camera ^ activeCamera;
		System::Collections::Generic::List<Plottable^>^ plotObjects;
		//Embedded< std::set<Plottable*> > plotObjects;
		property HDC hDC
		{
			HDC get() {return m_hDC;}
		}
	protected:
		System::Void SwapOpenGLBuffers(System::Void);
	protected:
		~glform();
		void draw();
		GLint mySetPixelFormat(HDC hdc);
	private:
		HDC m_hDC;
		HGLRC m_hglrc;
		CreateParams^ cp;
		int startGL();
		bool initialized;
		bool glstarted;
	};
}
