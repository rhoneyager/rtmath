#pragma once
#include <Windows.h>
#include <gl/GL.h>
#include "camera.h"
#include "glform.h"
using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;


namespace glgraphwin {

	enum graphmousemodes
	{
		NONE,
		PANNING
	};

	/// <summary>
	/// Summary for glgraphwinControl
	/*    This is a control for adding an openGL graph (2D/3D)
	into a managed window.
	Note:
	I need the functionality of System::Windows::Forms::NativeWindow,
	so I must encapsulate this within the usercontrol.
	It's unfortunate that I can't have two base classes.
	*/
	/// </summary>
	public ref class glgraphwinControl : 
		public System::Windows::Forms::UserControl
			//public System::Windows::Forms::NativeWindow
		{
		public:
			glgraphwinControl(void);
			void redraw();
			void render();
			glform^ openglform;
			camera^ activeCamera;
		private:
			bool _runtime;
			bool _mouseinwindow;
			System::Drawing::Point mouserecpos;
		private: System::Windows::Forms::Timer^  timer1;
		protected:
			~glgraphwinControl();
		protected: property System::Windows::Forms::CreateParams^ CreateParams 
				   {

					   virtual System::Windows::Forms::CreateParams^ get () override
					   {
						   //System::Windows::Forms::CreateParams^ cp=__super::CreateParams;
						   System::Windows::Forms::CreateParams^ cp=UserControl::CreateParams;
						   //cp->ExStyle|=0x00000020; //WS_EX_TRANSPARENT
						   cp->ExStyle|=WS_EX_TRANSPARENT;
						   return cp;
					   }

				   }

		private: System::ComponentModel::IContainer^  components;
		// Add the override mouse/keyboard handler flag
		private:
			bool _overrideui;
			graphmousemodes mmode;
		public:
			property bool UIoverride
			{
				bool get()
				{
					return _overrideui;
				}
				void set(bool value)
				{
					_overrideui = value;
				}
			}

		private:
			/// <summary>
			/// Required designer variable.
			/// </summary>


#pragma region Windows Form Designer generated code
			/// <summary>
			/// Required method for Designer support - do not modify
			/// the contents of this method with the code editor.
			/// </summary>
			void InitializeComponent(void)
			{
				this->components = (gcnew System::ComponentModel::Container());
				this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
				this->SuspendLayout();
				// 
				// timer1
				// 
				this->timer1->Enabled = true;
				this->timer1->Interval = 5;
				this->timer1->Tick += gcnew System::EventHandler(this, &glgraphwinControl::timer1_Tick);
				// 
				// glgraphwinControl
				// 
				this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
				this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
				this->BackColor = System::Drawing::SystemColors::ControlDark;
				this->Name = L"glgraphwinControl";
				this->Size = System::Drawing::Size(300, 300);
				this->VisibleChanged += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_VisibleChanged);
				this->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &glgraphwinControl::glgraphwinControl_Paint);
				this->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &glgraphwinControl::glgraphwinControl_KeyPress);
				this->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &glgraphwinControl::glgraphwinControl_MouseClick);
				this->MouseCaptureChanged += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_MouseCaptureChanged);
				this->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &glgraphwinControl::glgraphwinControl_MouseDown);
				this->MouseEnter += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_MouseEnter);
				this->MouseLeave += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_MouseLeave);
				this->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &glgraphwinControl::glgraphwinControl_MouseMove);
				this->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &glgraphwinControl::glgraphwinControl_MouseUp);
				this->MouseWheel += gcnew System::Windows::Forms::MouseEventHandler(this, &glgraphwinControl::glgraphwinControl_MouseWheel);
				this->Move += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_Move);
				this->Resize += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_Resize);
				this->ResumeLayout(false);

			}
#pragma endregion
		private: System::Void glgraphwinControl_Resize(System::Object^  sender, System::EventArgs^  e);
		private: System::Void glgraphwinControl_Move(System::Object^  sender, System::EventArgs^  e);
		private: System::Void glgraphwinControl_VisibleChanged(System::Object^  sender, System::EventArgs^  e);
		private: System::Void glgraphwinControl_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e);
		private: System::Void glgraphwinControl_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
		private: System::Void glgraphwinControl_MouseWheel(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e);
		private: System::Void glgraphwinControl_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e);
		private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
					 UNREFERENCED_PARAMETER(sender);
					 UNREFERENCED_PARAMETER(e);
					 if (openglform)
					 {
						 redraw();
					 }
					 //openglform->SwapOpenGLBuffers();
				 }

		private: System::Void glgraphwinControl_MouseEnter(System::Object^  sender, System::EventArgs^  e);
		private: System::Void glgraphwinControl_MouseLeave(System::Object^  sender, System::EventArgs^  e);
		private: System::Void activeCamera_Changed(System::Object^  sender);
private: System::Void glgraphwinControl_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
private: System::Void glgraphwinControl_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
private: System::Void glgraphwinControl_MouseUp(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e);
private: System::Void glgraphwinControl_MouseCaptureChanged(System::Object^  sender, System::EventArgs^  e);
};
}
