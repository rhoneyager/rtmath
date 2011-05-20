#pragma once

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;


namespace rtmathwinui {

	/// <summary>
	/// Summary for rtBaseAtmos
	/// </summary>
	public ref class rtBaseAtmos :  public System::ComponentModel::Component
	{
	public:
		rtBaseAtmos(void);
		rtBaseAtmos(System::ComponentModel::IContainer ^container);
	protected:
		~rtBaseAtmos()
	private:
	private:
		System::ComponentModel::Container ^components;
#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			components = gcnew System::ComponentModel::Container();
		}
#pragma endregion
	};
}
