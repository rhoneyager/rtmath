#pragma once
#include "rtRun.h"

namespace rtmathwinui {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for frmMain
	/// </summary>
	public ref class frmMain : public System::Windows::Forms::Form
	{
	public:
		frmMain(void);
	private: glgraphwin::glgraphwinControl^  glgraphwinControl1;
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::CheckBox^  checkBox1;
	public: 
		// Add in the rtRun class
		rtRun^ currRun;
	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~frmMain()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::MenuStrip^  menuStrip1;
	protected: 
	private: System::Windows::Forms::ToolStripMenuItem^  fileToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  newRunToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  openRunToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  saveResultsToolStripMenuItem;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator1;
	private: System::Windows::Forms::ToolStripMenuItem^  exitToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  modelToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  newModelToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  openModelToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  saveModelToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  atmosphereToolStripMenuItem;







	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator3;

	private: System::Windows::Forms::ToolStripMenuItem^  surfaceToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  modelSelectionToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  lightingToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  filtersToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  defineFIlterToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  bandpassToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  highPassToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  lowPassToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  notchToolStripMenuItem;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator5;
	private: System::Windows::Forms::ToolStripMenuItem^  customToolStripMenuItem1;
	private: System::Windows::Forms::ToolStripMenuItem^  selectFilterToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  noneToolStripMenuItem;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator4;
	private: System::Windows::Forms::ToolStripMenuItem^  customToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  setLightingSourcesToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  exportImagesToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  exportResultsToolStripMenuItem;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator6;
	private: System::Windows::Forms::ToolStripMenuItem^  propertiesToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  baseAtmosphereToolStripMenuItem;

	private: System::Windows::Forms::ToolStripMenuItem^  toolStripMenuItem1;
	private: System::Windows::Forms::ToolStripMenuItem^  newToolStripMenuItem2;
	private: System::Windows::Forms::ToolStripMenuItem^  openToolStripMenuItem2;
	private: System::Windows::Forms::ToolStripMenuItem^  saveToolStripMenuItem2;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator7;
	private: System::Windows::Forms::ToolStripMenuItem^  layersToolStripMenuItem1;
	private: System::Windows::Forms::ToolStripMenuItem^  newToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  openToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  saveToolStripMenuItem;
	private: System::Windows::Forms::ToolStripSeparator^  toolStripSeparator2;
	private: System::Windows::Forms::ToolStripMenuItem^  aerosolsToolStripMenuItem1;
	private: System::Windows::Forms::ToolStripMenuItem^  cloudsToolStripMenuItem1;
	private: System::Windows::Forms::ToolStripMenuItem^  overridesToolStripMenuItem1;




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
			this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
			this->fileToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->newRunToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openRunToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveResultsToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator1 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->exportImagesToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->exportResultsToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator6 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->exitToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->modelToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->newModelToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openModelToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveModelToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->propertiesToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->atmosphereToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->baseAtmosphereToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->newToolStripMenuItem2 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openToolStripMenuItem2 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveToolStripMenuItem2 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator7 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->layersToolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator3 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->toolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->newToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator2 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->aerosolsToolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->cloudsToolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->overridesToolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->surfaceToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->modelSelectionToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->lightingToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->setLightingSourcesToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->filtersToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->defineFIlterToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->bandpassToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->highPassToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->lowPassToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->notchToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator5 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->customToolStripMenuItem1 = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->selectFilterToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->noneToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->toolStripSeparator4 = (gcnew System::Windows::Forms::ToolStripSeparator());
			this->customToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->glgraphwinControl1 = (gcnew glgraphwin::glgraphwinControl());
			this->checkBox1 = (gcnew System::Windows::Forms::CheckBox());
			this->menuStrip1->SuspendLayout();
			this->SuspendLayout();
			// 
			// menuStrip1
			// 
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {this->fileToolStripMenuItem, 
				this->modelToolStripMenuItem, this->atmosphereToolStripMenuItem, this->surfaceToolStripMenuItem, this->lightingToolStripMenuItem, 
				this->filtersToolStripMenuItem});
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Size = System::Drawing::Size(548, 24);
			this->menuStrip1->TabIndex = 0;
			this->menuStrip1->Text = L"menuStrip1";
			this->menuStrip1->ItemClicked += gcnew System::Windows::Forms::ToolStripItemClickedEventHandler(this, &frmMain::menuStrip1_ItemClicked);
			// 
			// fileToolStripMenuItem
			// 
			this->fileToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(8) {this->newRunToolStripMenuItem, 
				this->openRunToolStripMenuItem, this->saveResultsToolStripMenuItem, this->toolStripSeparator1, this->exportImagesToolStripMenuItem, 
				this->exportResultsToolStripMenuItem, this->toolStripSeparator6, this->exitToolStripMenuItem});
			this->fileToolStripMenuItem->Name = L"fileToolStripMenuItem";
			this->fileToolStripMenuItem->Size = System::Drawing::Size(37, 20);
			this->fileToolStripMenuItem->Text = L"&File";
			// 
			// newRunToolStripMenuItem
			// 
			this->newRunToolStripMenuItem->Name = L"newRunToolStripMenuItem";
			this->newRunToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->newRunToolStripMenuItem->Text = L"&New Run";
			// 
			// openRunToolStripMenuItem
			// 
			this->openRunToolStripMenuItem->Name = L"openRunToolStripMenuItem";
			this->openRunToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->openRunToolStripMenuItem->Text = L"&Open Run";
			// 
			// saveResultsToolStripMenuItem
			// 
			this->saveResultsToolStripMenuItem->Name = L"saveResultsToolStripMenuItem";
			this->saveResultsToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->saveResultsToolStripMenuItem->Text = L"&Save Results";
			// 
			// toolStripSeparator1
			// 
			this->toolStripSeparator1->Name = L"toolStripSeparator1";
			this->toolStripSeparator1->Size = System::Drawing::Size(145, 6);
			// 
			// exportImagesToolStripMenuItem
			// 
			this->exportImagesToolStripMenuItem->Name = L"exportImagesToolStripMenuItem";
			this->exportImagesToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->exportImagesToolStripMenuItem->Text = L"Export &Images";
			// 
			// exportResultsToolStripMenuItem
			// 
			this->exportResultsToolStripMenuItem->Name = L"exportResultsToolStripMenuItem";
			this->exportResultsToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->exportResultsToolStripMenuItem->Text = L"&Export Results";
			// 
			// toolStripSeparator6
			// 
			this->toolStripSeparator6->Name = L"toolStripSeparator6";
			this->toolStripSeparator6->Size = System::Drawing::Size(145, 6);
			// 
			// exitToolStripMenuItem
			// 
			this->exitToolStripMenuItem->Name = L"exitToolStripMenuItem";
			this->exitToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->exitToolStripMenuItem->Text = L"E&xit";
			this->exitToolStripMenuItem->Click += gcnew System::EventHandler(this, &frmMain::exitToolStripMenuItem_Click);
			// 
			// modelToolStripMenuItem
			// 
			this->modelToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(4) {this->newModelToolStripMenuItem, 
				this->openModelToolStripMenuItem, this->saveModelToolStripMenuItem, this->propertiesToolStripMenuItem});
			this->modelToolStripMenuItem->Name = L"modelToolStripMenuItem";
			this->modelToolStripMenuItem->Size = System::Drawing::Size(53, 20);
			this->modelToolStripMenuItem->Text = L"&Model";
			// 
			// newModelToolStripMenuItem
			// 
			this->newModelToolStripMenuItem->Name = L"newModelToolStripMenuItem";
			this->newModelToolStripMenuItem->Size = System::Drawing::Size(140, 22);
			this->newModelToolStripMenuItem->Text = L"&New Model";
			// 
			// openModelToolStripMenuItem
			// 
			this->openModelToolStripMenuItem->Name = L"openModelToolStripMenuItem";
			this->openModelToolStripMenuItem->Size = System::Drawing::Size(140, 22);
			this->openModelToolStripMenuItem->Text = L"&Open Model";
			// 
			// saveModelToolStripMenuItem
			// 
			this->saveModelToolStripMenuItem->Name = L"saveModelToolStripMenuItem";
			this->saveModelToolStripMenuItem->Size = System::Drawing::Size(140, 22);
			this->saveModelToolStripMenuItem->Text = L"&Save Model";
			// 
			// propertiesToolStripMenuItem
			// 
			this->propertiesToolStripMenuItem->Name = L"propertiesToolStripMenuItem";
			this->propertiesToolStripMenuItem->Size = System::Drawing::Size(140, 22);
			this->propertiesToolStripMenuItem->Text = L"&Properties...";
			// 
			// atmosphereToolStripMenuItem
			// 
			this->atmosphereToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->baseAtmosphereToolStripMenuItem, 
				this->toolStripSeparator3, this->toolStripMenuItem1});
			this->atmosphereToolStripMenuItem->Name = L"atmosphereToolStripMenuItem";
			this->atmosphereToolStripMenuItem->Size = System::Drawing::Size(84, 20);
			this->atmosphereToolStripMenuItem->Text = L"&Atmosphere";
			// 
			// baseAtmosphereToolStripMenuItem
			// 
			this->baseAtmosphereToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(5) {this->newToolStripMenuItem2, 
				this->openToolStripMenuItem2, this->saveToolStripMenuItem2, this->toolStripSeparator7, this->layersToolStripMenuItem1});
			this->baseAtmosphereToolStripMenuItem->Name = L"baseAtmosphereToolStripMenuItem";
			this->baseAtmosphereToolStripMenuItem->Size = System::Drawing::Size(166, 22);
			this->baseAtmosphereToolStripMenuItem->Text = L"&Base Atmosphere";
			this->baseAtmosphereToolStripMenuItem->Click += gcnew System::EventHandler(this, &frmMain::baseAtmosphereToolStripMenuItem_Click);
			// 
			// newToolStripMenuItem2
			// 
			this->newToolStripMenuItem2->Name = L"newToolStripMenuItem2";
			this->newToolStripMenuItem2->Size = System::Drawing::Size(116, 22);
			this->newToolStripMenuItem2->Text = L"&New";
			// 
			// openToolStripMenuItem2
			// 
			this->openToolStripMenuItem2->Name = L"openToolStripMenuItem2";
			this->openToolStripMenuItem2->Size = System::Drawing::Size(116, 22);
			this->openToolStripMenuItem2->Text = L"&Open";
			// 
			// saveToolStripMenuItem2
			// 
			this->saveToolStripMenuItem2->Name = L"saveToolStripMenuItem2";
			this->saveToolStripMenuItem2->Size = System::Drawing::Size(116, 22);
			this->saveToolStripMenuItem2->Text = L"&Save";
			// 
			// toolStripSeparator7
			// 
			this->toolStripSeparator7->Name = L"toolStripSeparator7";
			this->toolStripSeparator7->Size = System::Drawing::Size(113, 6);
			// 
			// layersToolStripMenuItem1
			// 
			this->layersToolStripMenuItem1->Name = L"layersToolStripMenuItem1";
			this->layersToolStripMenuItem1->Size = System::Drawing::Size(116, 22);
			this->layersToolStripMenuItem1->Text = L"&Layers...";
			this->layersToolStripMenuItem1->Click += gcnew System::EventHandler(this, &frmMain::layersToolStripMenuItem1_Click);
			// 
			// toolStripSeparator3
			// 
			this->toolStripSeparator3->Name = L"toolStripSeparator3";
			this->toolStripSeparator3->Size = System::Drawing::Size(163, 6);
			// 
			// toolStripMenuItem1
			// 
			this->toolStripMenuItem1->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(7) {this->newToolStripMenuItem, 
				this->openToolStripMenuItem, this->saveToolStripMenuItem, this->toolStripSeparator2, this->aerosolsToolStripMenuItem1, this->cloudsToolStripMenuItem1, 
				this->overridesToolStripMenuItem1});
			this->toolStripMenuItem1->Name = L"toolStripMenuItem1";
			this->toolStripMenuItem1->Size = System::Drawing::Size(166, 22);
			this->toolStripMenuItem1->Text = L"&Overlays";
			// 
			// newToolStripMenuItem
			// 
			this->newToolStripMenuItem->Name = L"newToolStripMenuItem";
			this->newToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->newToolStripMenuItem->Text = L"&New";
			// 
			// openToolStripMenuItem
			// 
			this->openToolStripMenuItem->Name = L"openToolStripMenuItem";
			this->openToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->openToolStripMenuItem->Text = L"&Open";
			// 
			// saveToolStripMenuItem
			// 
			this->saveToolStripMenuItem->Name = L"saveToolStripMenuItem";
			this->saveToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->saveToolStripMenuItem->Text = L"&Save";
			// 
			// toolStripSeparator2
			// 
			this->toolStripSeparator2->Name = L"toolStripSeparator2";
			this->toolStripSeparator2->Size = System::Drawing::Size(130, 6);
			// 
			// aerosolsToolStripMenuItem1
			// 
			this->aerosolsToolStripMenuItem1->Name = L"aerosolsToolStripMenuItem1";
			this->aerosolsToolStripMenuItem1->Size = System::Drawing::Size(133, 22);
			this->aerosolsToolStripMenuItem1->Text = L"&Aerosols...";
			// 
			// cloudsToolStripMenuItem1
			// 
			this->cloudsToolStripMenuItem1->Name = L"cloudsToolStripMenuItem1";
			this->cloudsToolStripMenuItem1->Size = System::Drawing::Size(133, 22);
			this->cloudsToolStripMenuItem1->Text = L"&Clouds...";
			// 
			// overridesToolStripMenuItem1
			// 
			this->overridesToolStripMenuItem1->Name = L"overridesToolStripMenuItem1";
			this->overridesToolStripMenuItem1->Size = System::Drawing::Size(133, 22);
			this->overridesToolStripMenuItem1->Text = L"&Overrides...";
			// 
			// surfaceToolStripMenuItem
			// 
			this->surfaceToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->modelSelectionToolStripMenuItem});
			this->surfaceToolStripMenuItem->Name = L"surfaceToolStripMenuItem";
			this->surfaceToolStripMenuItem->Size = System::Drawing::Size(58, 20);
			this->surfaceToolStripMenuItem->Text = L"&Surface";
			// 
			// modelSelectionToolStripMenuItem
			// 
			this->modelSelectionToolStripMenuItem->Name = L"modelSelectionToolStripMenuItem";
			this->modelSelectionToolStripMenuItem->Size = System::Drawing::Size(159, 22);
			this->modelSelectionToolStripMenuItem->Text = L"&Model Selection";
			// 
			// lightingToolStripMenuItem
			// 
			this->lightingToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->setLightingSourcesToolStripMenuItem});
			this->lightingToolStripMenuItem->Name = L"lightingToolStripMenuItem";
			this->lightingToolStripMenuItem->Size = System::Drawing::Size(63, 20);
			this->lightingToolStripMenuItem->Text = L"&Lighting";
			// 
			// setLightingSourcesToolStripMenuItem
			// 
			this->setLightingSourcesToolStripMenuItem->Name = L"setLightingSourcesToolStripMenuItem";
			this->setLightingSourcesToolStripMenuItem->Size = System::Drawing::Size(181, 22);
			this->setLightingSourcesToolStripMenuItem->Text = L"&Set Lighting Sources";
			// 
			// filtersToolStripMenuItem
			// 
			this->filtersToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {this->defineFIlterToolStripMenuItem, 
				this->selectFilterToolStripMenuItem});
			this->filtersToolStripMenuItem->Name = L"filtersToolStripMenuItem";
			this->filtersToolStripMenuItem->Size = System::Drawing::Size(50, 20);
			this->filtersToolStripMenuItem->Text = L"&Filters";
			// 
			// defineFIlterToolStripMenuItem
			// 
			this->defineFIlterToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {this->bandpassToolStripMenuItem, 
				this->highPassToolStripMenuItem, this->lowPassToolStripMenuItem, this->notchToolStripMenuItem, this->toolStripSeparator5, this->customToolStripMenuItem1});
			this->defineFIlterToolStripMenuItem->Name = L"defineFIlterToolStripMenuItem";
			this->defineFIlterToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->defineFIlterToolStripMenuItem->Text = L"&Define Filter";
			// 
			// bandpassToolStripMenuItem
			// 
			this->bandpassToolStripMenuItem->Name = L"bandpassToolStripMenuItem";
			this->bandpassToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->bandpassToolStripMenuItem->Text = L"&Bandpass...";
			// 
			// highPassToolStripMenuItem
			// 
			this->highPassToolStripMenuItem->Name = L"highPassToolStripMenuItem";
			this->highPassToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->highPassToolStripMenuItem->Text = L"&High-Pass...";
			// 
			// lowPassToolStripMenuItem
			// 
			this->lowPassToolStripMenuItem->Name = L"lowPassToolStripMenuItem";
			this->lowPassToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->lowPassToolStripMenuItem->Text = L"&Low-Pass...";
			// 
			// notchToolStripMenuItem
			// 
			this->notchToolStripMenuItem->Name = L"notchToolStripMenuItem";
			this->notchToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->notchToolStripMenuItem->Text = L"&Notch...";
			// 
			// toolStripSeparator5
			// 
			this->toolStripSeparator5->Name = L"toolStripSeparator5";
			this->toolStripSeparator5->Size = System::Drawing::Size(134, 6);
			// 
			// customToolStripMenuItem1
			// 
			this->customToolStripMenuItem1->Name = L"customToolStripMenuItem1";
			this->customToolStripMenuItem1->Size = System::Drawing::Size(137, 22);
			this->customToolStripMenuItem1->Text = L"&Custom...";
			// 
			// selectFilterToolStripMenuItem
			// 
			this->selectFilterToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->noneToolStripMenuItem, 
				this->toolStripSeparator4, this->customToolStripMenuItem});
			this->selectFilterToolStripMenuItem->Name = L"selectFilterToolStripMenuItem";
			this->selectFilterToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->selectFilterToolStripMenuItem->Text = L"&Select Filter";
			// 
			// noneToolStripMenuItem
			// 
			this->noneToolStripMenuItem->Name = L"noneToolStripMenuItem";
			this->noneToolStripMenuItem->Size = System::Drawing::Size(122, 22);
			this->noneToolStripMenuItem->Text = L"&None";
			// 
			// toolStripSeparator4
			// 
			this->toolStripSeparator4->Name = L"toolStripSeparator4";
			this->toolStripSeparator4->Size = System::Drawing::Size(119, 6);
			// 
			// customToolStripMenuItem
			// 
			this->customToolStripMenuItem->Name = L"customToolStripMenuItem";
			this->customToolStripMenuItem->Size = System::Drawing::Size(122, 22);
			this->customToolStripMenuItem->Text = L"(custom)";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(437, 199);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(75, 23);
			this->button1->TabIndex = 2;
			this->button1->Text = L"Reinit";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &frmMain::button1_Click);
			// 
			// glgraphwinControl1
			// 
			this->glgraphwinControl1->BackColor = System::Drawing::SystemColors::ControlDark;
			this->glgraphwinControl1->Location = System::Drawing::Point(34, 50);
			this->glgraphwinControl1->Name = L"glgraphwinControl1";
			this->glgraphwinControl1->Size = System::Drawing::Size(300, 300);
			this->glgraphwinControl1->TabIndex = 1;
			this->glgraphwinControl1->TabStop = false;
			// 
			// checkBox1
			// 
			this->checkBox1->AutoSize = true;
			this->checkBox1->Checked = true;
			this->checkBox1->CheckState = System::Windows::Forms::CheckState::Checked;
			this->checkBox1->Location = System::Drawing::Point(438, 148);
			this->checkBox1->Name = L"checkBox1";
			this->checkBox1->Size = System::Drawing::Size(56, 17);
			this->checkBox1->TabIndex = 3;
			this->checkBox1->Text = L"Visible";
			this->checkBox1->UseVisualStyleBackColor = true;
			this->checkBox1->CheckedChanged += gcnew System::EventHandler(this, &frmMain::checkBox1_CheckedChanged);
			// 
			// frmMain
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(548, 407);
			this->Controls->Add(this->checkBox1);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->glgraphwinControl1);
			this->Controls->Add(this->menuStrip1);
			this->MainMenuStrip = this->menuStrip1;
			this->Name = L"frmMain";
			this->Text = L"rtmath";
			this->Load += gcnew System::EventHandler(this, &frmMain::frmMain_Load);
			this->Move += gcnew System::EventHandler(this, &frmMain::frmMain_Move);
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void exitToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void baseAtmosphereToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e) {
		 }
private: System::Void menuStrip1_ItemClicked(System::Object^  sender, System::Windows::Forms::ToolStripItemClickedEventArgs^  e) {
		 }
private: System::Void layersToolStripMenuItem1_Click(System::Object^  sender, System::EventArgs^  e);
private: System::Void frmMain_Load(System::Object^  sender, System::EventArgs^  e) {
			 //this->glgraphwinControl1;
			 // Needed since the form doesn't fully exist at initialization
			 this->glgraphwinControl1->redraw();
			 this->glgraphwinControl1->render();
		 }
private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->glgraphwinControl1->redraw();
			 this->glgraphwinControl1->render();
		 }
private: System::Void frmMain_Move(System::Object^  sender, System::EventArgs^  e) {
			 this->glgraphwinControl1->render();
		 }
private: System::Void checkBox1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
			 this->glgraphwinControl1->Visible = checkBox1->Checked;
		 }
};
}
