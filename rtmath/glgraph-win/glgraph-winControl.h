#pragma once
#include <Windows.h>
#include <gl/GL.h>
#include "glform.h"
using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Windows::Forms;
using namespace System::Data;
using namespace System::Drawing;


namespace glgraphwin {

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
		glgraphwinControl(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			openglform = gcnew glform(this->ParentForm, 
				this->Size.Width, this->Size.Height);
		}
		void redraw(System::Windows::Forms::Form ^ par)
		{
			if (openglform)
			{
				delete openglform;
			}
			openglform = gcnew glform(par, 
				this->Size.Width, this->Size.Height);
		}
	private: System::Windows::Forms::Timer^  timer1;
	public: 
		glform^ openglform;
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~glgraphwinControl()
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
	private: System::ComponentModel::IContainer^  components;
	protected: 

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
			this->timer1->Tick += gcnew System::EventHandler(this, &glgraphwinControl::timer1_Tick);
			// 
			// glgraphwinControl
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->BackColor = System::Drawing::SystemColors::ControlDark;
			this->Name = L"glgraphwinControl";
			this->Size = System::Drawing::Size(300, 300);
			this->BackColorChanged += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_BackColorChanged);
			this->EnabledChanged += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_EnabledChanged);
			this->VisibleChanged += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_VisibleChanged);
			this->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &glgraphwinControl::glgraphwinControl_Paint);
			this->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &glgraphwinControl::glgraphwinControl_KeyPress);
			this->MouseClick += gcnew System::Windows::Forms::MouseEventHandler(this, &glgraphwinControl::glgraphwinControl_MouseClick);
			this->Move += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_Move);
			this->Resize += gcnew System::EventHandler(this, &glgraphwinControl::glgraphwinControl_Resize);
			this->ResumeLayout(false);

		}
#pragma endregion
	private: System::Void glgraphwinControl_Resize(System::Object^  sender, System::EventArgs^  e) {
				 //throw;
			 }
	private: System::Void glgraphwinControl_Move(System::Object^  sender, System::EventArgs^  e) {
				 //throw;
			 }
	private: System::Void glgraphwinControl_VisibleChanged(System::Object^  sender, System::EventArgs^  e) {
				 //throw;
			 }
	private: System::Void glgraphwinControl_EnabledChanged(System::Object^  sender, System::EventArgs^  e) {
				 //throw;
			 }
	private: System::Void glgraphwinControl_KeyPress(System::Object^  sender, System::Windows::Forms::KeyPressEventArgs^  e) {
			 }
	private: System::Void glgraphwinControl_MouseClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e) {
			 }
	private: System::Void glgraphwinControl_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
    UNREFERENCED_PARAMETER(sender);
    UNREFERENCED_PARAMETER(e);
    openglform->Render();
    openglform->SwapOpenGLBuffers();
			 }
	private: System::Void timer1_Tick(System::Object^  sender, System::EventArgs^  e) {
    UNREFERENCED_PARAMETER(sender);
    UNREFERENCED_PARAMETER(e);
    openglform->Render();
    openglform->SwapOpenGLBuffers();
			 }
	private: System::Void glgraphwinControl_BackColorChanged(System::Object^  sender, System::EventArgs^  e) {
				 //this->glgraphwinControl1->BackColor = System::Drawing::Color::Black;
				 
			 }
	};
}
