#pragma once
#include "rtModel.h"
using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;
using namespace System::Collections::Generic;


namespace rtmathwinui {

	/// <summary>
	/// rtRun is the class that holds the necessary run data for the current run
	/// This includes the atmosphere and all its layers and overlays
	/// Also contains the surface, lighting, filters and models
	/* A run is really just a collection of models that are performed together
	   The run holds them together for efficient batch processing and parallelization.
	   Each model contains a defined atmosphere, with layers, overlays, filters, lights 
	   and surfaces.
	   The effective atmosphere is computed based on angle, since the p-p approximation is 
	   assumed concuttently with the independent column approximation.

	   So, rtRun really just holds an array of models, an array of atmospheres, 
	   the set of layers, ..., independently, and each model links the necessary 
	   components together.
	   */
	/// </summary>
	public ref class rtRun :  public System::ComponentModel::Component
	{
	public:
		rtRun(void)
		{
			InitializeComponent();
			//
			//TODO: Add the constructor code here
			//
			models = gcnew List<rtModel^>();
		}
		rtRun(System::ComponentModel::IContainer ^container)
		{
			/// <summary>
			/// Required for Windows.Forms Class Composition Designer support
			/// </summary>

			container->Add(this);
			InitializeComponent();
		}
		List<rtModel^>^ models;
		//List<rtAtmos^>^ atmos;
		//List<rtLight^>^ lights;
		//List<rtLayer^>^ layers;
		//List<rtSurface^>^ surfaces;
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~rtRun()
		{
			if (components)
			{
				delete components;
			}
		}

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
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
