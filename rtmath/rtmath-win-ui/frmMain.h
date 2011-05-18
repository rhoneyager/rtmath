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



	private: System::Windows::Forms::TabControl^  tabs;
	private: System::Windows::Forms::TabPage^  tabSummary;

	private: System::Windows::Forms::TabPage^  tabAtmos;





	private: System::Windows::Forms::TabPage^  tabResults;
	private: System::Windows::Forms::TableLayoutPanel^  panelAtmosLayout;
	private: System::Windows::Forms::Label^  label1;

	private: System::Windows::Forms::FlowLayoutPanel^  flowLayoutPanel1;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::Button^  button4;
	private: System::Windows::Forms::Button^  button5;
	private: System::Windows::Forms::Button^  button6;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::PropertyGrid^  layerProps;
	private: System::Windows::Forms::Label^  label3;

	private: System::Windows::Forms::FlowLayoutPanel^  flowLayoutPanel2;
	private: System::Windows::Forms::Button^  button3;
	private: System::Windows::Forms::Button^  button7;
	private: System::Windows::Forms::Button^  button8;
	private: System::Windows::Forms::NumericUpDown^  numericUpDown1;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::StatusStrip^  statusbar;

	private: System::Windows::Forms::ToolStripProgressBar^  toolStripProgressBar1;
	private: System::Windows::Forms::TableLayoutPanel^  tableLayoutPanel1;

	private: glgraphwin::glgraphwinControl^  glgraphwinControl1;
	private: System::Windows::Forms::SplitContainer^  splitContainer1;









	private: System::Windows::Forms::CheckBox^  chkAdvAtmos;

	private: System::Windows::Forms::TableLayoutPanel^  tableLayoutPanel3;
	private: System::Windows::Forms::FlowLayoutPanel^  flowLayoutPanel3;
	private: System::Windows::Forms::Button^  button9;
	private: System::Windows::Forms::Button^  button10;
	private: System::Windows::Forms::Button^  button11;
	private: System::Windows::Forms::ToolStripStatusLabel^  toolStripStatusLabel1;
	private: System::Windows::Forms::ToolStripStatusLabel^  toolStripStatusLabel2;
	private: System::Windows::Forms::Label^  label8;
	private: System::ComponentModel::BackgroundWorker^  threadCalc;
	private: System::Windows::Forms::PictureBox^  pictureBox1;
	private: System::Windows::Forms::SaveFileDialog^  dlgSave;
	private: System::Windows::Forms::OpenFileDialog^  dlgOpen;
	private: System::Windows::Forms::ColorDialog^  dlgColor;
	private: System::Windows::Forms::TreeView^  listAtmosLayers;
	private: System::Windows::Forms::SplitContainer^  splitContainer2;
	private: System::Windows::Forms::TreeView^  treePlots;
	private: System::Windows::Forms::PropertyGrid^  propPlots;


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
			System::ComponentModel::ComponentResourceManager^  resources = (gcnew System::ComponentModel::ComponentResourceManager(frmMain::typeid));
			System::Windows::Forms::TreeNode^  treeNode1 = (gcnew System::Windows::Forms::TreeNode(L"Base Atmosphere Selection"));
			System::Windows::Forms::TreeNode^  treeNode2 = (gcnew System::Windows::Forms::TreeNode(L"Example Cloud Layer 1"));
			System::Windows::Forms::TreeNode^  treeNode3 = (gcnew System::Windows::Forms::TreeNode(L"Example Cloud Layer 2"));
			System::Windows::Forms::TreeNode^  treeNode4 = (gcnew System::Windows::Forms::TreeNode(L"Clouds", gcnew cli::array< System::Windows::Forms::TreeNode^  >(2) {treeNode2, 
				treeNode3}));
			System::Windows::Forms::TreeNode^  treeNode5 = (gcnew System::Windows::Forms::TreeNode(L"CO2"));
			System::Windows::Forms::TreeNode^  treeNode6 = (gcnew System::Windows::Forms::TreeNode(L"H2O"));
			System::Windows::Forms::TreeNode^  treeNode7 = (gcnew System::Windows::Forms::TreeNode(L"Gases", gcnew cli::array< System::Windows::Forms::TreeNode^  >(2) {treeNode5, 
				treeNode6}));
			System::Windows::Forms::TreeNode^  treeNode8 = (gcnew System::Windows::Forms::TreeNode(L"Aerosols"));
			System::Windows::Forms::TreeNode^  treeNode9 = (gcnew System::Windows::Forms::TreeNode(L"Sun"));
			System::Windows::Forms::TreeNode^  treeNode10 = (gcnew System::Windows::Forms::TreeNode(L"Lighting", gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) {treeNode9}));
			System::Windows::Forms::TreeNode^  treeNode11 = (gcnew System::Windows::Forms::TreeNode(L"Surface Model"));
			System::Windows::Forms::TreeNode^  treeNode12 = (gcnew System::Windows::Forms::TreeNode(L"Filters"));
			System::Windows::Forms::TreeNode^  treeNode13 = (gcnew System::Windows::Forms::TreeNode(L"Spectral Region"));
			System::Windows::Forms::TreeNode^  treeNode14 = (gcnew System::Windows::Forms::TreeNode(L"Sample Axis Line 1"));
			System::Windows::Forms::TreeNode^  treeNode15 = (gcnew System::Windows::Forms::TreeNode(L"Sample Axis Line 2"));
			System::Windows::Forms::TreeNode^  treeNode16 = (gcnew System::Windows::Forms::TreeNode(L"Model Run 1", gcnew cli::array< System::Windows::Forms::TreeNode^  >(2) {treeNode14, 
				treeNode15}));
			System::Windows::Forms::TreeNode^  treeNode17 = (gcnew System::Windows::Forms::TreeNode(L"Upwelling Radiance at TOA"));
			System::Windows::Forms::TreeNode^  treeNode18 = (gcnew System::Windows::Forms::TreeNode(L"Model Run 2", gcnew cli::array< System::Windows::Forms::TreeNode^  >(1) {treeNode17}));
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
			this->tabs = (gcnew System::Windows::Forms::TabControl());
			this->tabSummary = (gcnew System::Windows::Forms::TabPage());
			this->tableLayoutPanel3 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->flowLayoutPanel3 = (gcnew System::Windows::Forms::FlowLayoutPanel());
			this->button9 = (gcnew System::Windows::Forms::Button());
			this->button10 = (gcnew System::Windows::Forms::Button());
			this->button11 = (gcnew System::Windows::Forms::Button());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->pictureBox1 = (gcnew System::Windows::Forms::PictureBox());
			this->tabAtmos = (gcnew System::Windows::Forms::TabPage());
			this->panelAtmosLayout = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->flowLayoutPanel1 = (gcnew System::Windows::Forms::FlowLayoutPanel());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->button4 = (gcnew System::Windows::Forms::Button());
			this->button5 = (gcnew System::Windows::Forms::Button());
			this->button6 = (gcnew System::Windows::Forms::Button());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->layerProps = (gcnew System::Windows::Forms::PropertyGrid());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->flowLayoutPanel2 = (gcnew System::Windows::Forms::FlowLayoutPanel());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->button7 = (gcnew System::Windows::Forms::Button());
			this->button8 = (gcnew System::Windows::Forms::Button());
			this->numericUpDown1 = (gcnew System::Windows::Forms::NumericUpDown());
			this->chkAdvAtmos = (gcnew System::Windows::Forms::CheckBox());
			this->listAtmosLayers = (gcnew System::Windows::Forms::TreeView());
			this->tabResults = (gcnew System::Windows::Forms::TabPage());
			this->splitContainer1 = (gcnew System::Windows::Forms::SplitContainer());
			this->glgraphwinControl1 = (gcnew glgraphwin::glgraphwinControl());
			this->splitContainer2 = (gcnew System::Windows::Forms::SplitContainer());
			this->treePlots = (gcnew System::Windows::Forms::TreeView());
			this->propPlots = (gcnew System::Windows::Forms::PropertyGrid());
			this->statusbar = (gcnew System::Windows::Forms::StatusStrip());
			this->toolStripStatusLabel1 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->toolStripProgressBar1 = (gcnew System::Windows::Forms::ToolStripProgressBar());
			this->toolStripStatusLabel2 = (gcnew System::Windows::Forms::ToolStripStatusLabel());
			this->tableLayoutPanel1 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->threadCalc = (gcnew System::ComponentModel::BackgroundWorker());
			this->dlgSave = (gcnew System::Windows::Forms::SaveFileDialog());
			this->dlgOpen = (gcnew System::Windows::Forms::OpenFileDialog());
			this->dlgColor = (gcnew System::Windows::Forms::ColorDialog());
			this->menuStrip1->SuspendLayout();
			this->tabs->SuspendLayout();
			this->tabSummary->SuspendLayout();
			this->tableLayoutPanel3->SuspendLayout();
			this->flowLayoutPanel3->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->BeginInit();
			this->tabAtmos->SuspendLayout();
			this->panelAtmosLayout->SuspendLayout();
			this->flowLayoutPanel1->SuspendLayout();
			this->flowLayoutPanel2->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numericUpDown1))->BeginInit();
			this->tabResults->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->splitContainer1))->BeginInit();
			this->splitContainer1->Panel1->SuspendLayout();
			this->splitContainer1->Panel2->SuspendLayout();
			this->splitContainer1->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->splitContainer2))->BeginInit();
			this->splitContainer2->Panel1->SuspendLayout();
			this->splitContainer2->Panel2->SuspendLayout();
			this->splitContainer2->SuspendLayout();
			this->statusbar->SuspendLayout();
			this->tableLayoutPanel1->SuspendLayout();
			this->SuspendLayout();
			// 
			// menuStrip1
			// 
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {this->fileToolStripMenuItem, 
				this->modelToolStripMenuItem, this->atmosphereToolStripMenuItem, this->surfaceToolStripMenuItem, this->lightingToolStripMenuItem, 
				this->filtersToolStripMenuItem});
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Size = System::Drawing::Size(800, 24);
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
			this->fileToolStripMenuItem->Size = System::Drawing::Size(35, 20);
			this->fileToolStripMenuItem->Text = L"&File";
			// 
			// newRunToolStripMenuItem
			// 
			this->newRunToolStripMenuItem->Name = L"newRunToolStripMenuItem";
			this->newRunToolStripMenuItem->Size = System::Drawing::Size(144, 22);
			this->newRunToolStripMenuItem->Text = L"&New Run";
			// 
			// openRunToolStripMenuItem
			// 
			this->openRunToolStripMenuItem->Name = L"openRunToolStripMenuItem";
			this->openRunToolStripMenuItem->Size = System::Drawing::Size(144, 22);
			this->openRunToolStripMenuItem->Text = L"&Open Run";
			// 
			// saveResultsToolStripMenuItem
			// 
			this->saveResultsToolStripMenuItem->Name = L"saveResultsToolStripMenuItem";
			this->saveResultsToolStripMenuItem->Size = System::Drawing::Size(144, 22);
			this->saveResultsToolStripMenuItem->Text = L"&Save Results";
			// 
			// toolStripSeparator1
			// 
			this->toolStripSeparator1->Name = L"toolStripSeparator1";
			this->toolStripSeparator1->Size = System::Drawing::Size(141, 6);
			// 
			// exportImagesToolStripMenuItem
			// 
			this->exportImagesToolStripMenuItem->Name = L"exportImagesToolStripMenuItem";
			this->exportImagesToolStripMenuItem->Size = System::Drawing::Size(144, 22);
			this->exportImagesToolStripMenuItem->Text = L"Export &Images";
			// 
			// exportResultsToolStripMenuItem
			// 
			this->exportResultsToolStripMenuItem->Name = L"exportResultsToolStripMenuItem";
			this->exportResultsToolStripMenuItem->Size = System::Drawing::Size(144, 22);
			this->exportResultsToolStripMenuItem->Text = L"&Export Results";
			// 
			// toolStripSeparator6
			// 
			this->toolStripSeparator6->Name = L"toolStripSeparator6";
			this->toolStripSeparator6->Size = System::Drawing::Size(141, 6);
			// 
			// exitToolStripMenuItem
			// 
			this->exitToolStripMenuItem->Name = L"exitToolStripMenuItem";
			this->exitToolStripMenuItem->Size = System::Drawing::Size(144, 22);
			this->exitToolStripMenuItem->Text = L"E&xit";
			this->exitToolStripMenuItem->Click += gcnew System::EventHandler(this, &frmMain::exitToolStripMenuItem_Click);
			// 
			// modelToolStripMenuItem
			// 
			this->modelToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(4) {this->newModelToolStripMenuItem, 
				this->openModelToolStripMenuItem, this->saveModelToolStripMenuItem, this->propertiesToolStripMenuItem});
			this->modelToolStripMenuItem->Name = L"modelToolStripMenuItem";
			this->modelToolStripMenuItem->Size = System::Drawing::Size(47, 20);
			this->modelToolStripMenuItem->Text = L"&Model";
			// 
			// newModelToolStripMenuItem
			// 
			this->newModelToolStripMenuItem->Name = L"newModelToolStripMenuItem";
			this->newModelToolStripMenuItem->Size = System::Drawing::Size(135, 22);
			this->newModelToolStripMenuItem->Text = L"&New Model";
			// 
			// openModelToolStripMenuItem
			// 
			this->openModelToolStripMenuItem->Name = L"openModelToolStripMenuItem";
			this->openModelToolStripMenuItem->Size = System::Drawing::Size(135, 22);
			this->openModelToolStripMenuItem->Text = L"&Open Model";
			// 
			// saveModelToolStripMenuItem
			// 
			this->saveModelToolStripMenuItem->Name = L"saveModelToolStripMenuItem";
			this->saveModelToolStripMenuItem->Size = System::Drawing::Size(135, 22);
			this->saveModelToolStripMenuItem->Text = L"&Save Model";
			// 
			// propertiesToolStripMenuItem
			// 
			this->propertiesToolStripMenuItem->Name = L"propertiesToolStripMenuItem";
			this->propertiesToolStripMenuItem->Size = System::Drawing::Size(135, 22);
			this->propertiesToolStripMenuItem->Text = L"&Properties...";
			// 
			// atmosphereToolStripMenuItem
			// 
			this->atmosphereToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->baseAtmosphereToolStripMenuItem, 
				this->toolStripSeparator3, this->toolStripMenuItem1});
			this->atmosphereToolStripMenuItem->Name = L"atmosphereToolStripMenuItem";
			this->atmosphereToolStripMenuItem->Size = System::Drawing::Size(77, 20);
			this->atmosphereToolStripMenuItem->Text = L"&Atmosphere";
			// 
			// baseAtmosphereToolStripMenuItem
			// 
			this->baseAtmosphereToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(5) {this->newToolStripMenuItem2, 
				this->openToolStripMenuItem2, this->saveToolStripMenuItem2, this->toolStripSeparator7, this->layersToolStripMenuItem1});
			this->baseAtmosphereToolStripMenuItem->Name = L"baseAtmosphereToolStripMenuItem";
			this->baseAtmosphereToolStripMenuItem->Size = System::Drawing::Size(158, 22);
			this->baseAtmosphereToolStripMenuItem->Text = L"&Base Atmosphere";
			this->baseAtmosphereToolStripMenuItem->Click += gcnew System::EventHandler(this, &frmMain::baseAtmosphereToolStripMenuItem_Click);
			// 
			// newToolStripMenuItem2
			// 
			this->newToolStripMenuItem2->Name = L"newToolStripMenuItem2";
			this->newToolStripMenuItem2->Size = System::Drawing::Size(118, 22);
			this->newToolStripMenuItem2->Text = L"&New";
			// 
			// openToolStripMenuItem2
			// 
			this->openToolStripMenuItem2->Name = L"openToolStripMenuItem2";
			this->openToolStripMenuItem2->Size = System::Drawing::Size(118, 22);
			this->openToolStripMenuItem2->Text = L"&Open";
			// 
			// saveToolStripMenuItem2
			// 
			this->saveToolStripMenuItem2->Name = L"saveToolStripMenuItem2";
			this->saveToolStripMenuItem2->Size = System::Drawing::Size(118, 22);
			this->saveToolStripMenuItem2->Text = L"&Save";
			// 
			// toolStripSeparator7
			// 
			this->toolStripSeparator7->Name = L"toolStripSeparator7";
			this->toolStripSeparator7->Size = System::Drawing::Size(115, 6);
			// 
			// layersToolStripMenuItem1
			// 
			this->layersToolStripMenuItem1->Name = L"layersToolStripMenuItem1";
			this->layersToolStripMenuItem1->Size = System::Drawing::Size(118, 22);
			this->layersToolStripMenuItem1->Text = L"&Layers...";
			this->layersToolStripMenuItem1->Click += gcnew System::EventHandler(this, &frmMain::layersToolStripMenuItem1_Click);
			// 
			// toolStripSeparator3
			// 
			this->toolStripSeparator3->Name = L"toolStripSeparator3";
			this->toolStripSeparator3->Size = System::Drawing::Size(155, 6);
			// 
			// toolStripMenuItem1
			// 
			this->toolStripMenuItem1->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(7) {this->newToolStripMenuItem, 
				this->openToolStripMenuItem, this->saveToolStripMenuItem, this->toolStripSeparator2, this->aerosolsToolStripMenuItem1, this->cloudsToolStripMenuItem1, 
				this->overridesToolStripMenuItem1});
			this->toolStripMenuItem1->Name = L"toolStripMenuItem1";
			this->toolStripMenuItem1->Size = System::Drawing::Size(158, 22);
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
			this->surfaceToolStripMenuItem->Size = System::Drawing::Size(56, 20);
			this->surfaceToolStripMenuItem->Text = L"&Surface";
			// 
			// modelSelectionToolStripMenuItem
			// 
			this->modelSelectionToolStripMenuItem->Name = L"modelSelectionToolStripMenuItem";
			this->modelSelectionToolStripMenuItem->Size = System::Drawing::Size(148, 22);
			this->modelSelectionToolStripMenuItem->Text = L"&Model Selection";
			// 
			// lightingToolStripMenuItem
			// 
			this->lightingToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(1) {this->setLightingSourcesToolStripMenuItem});
			this->lightingToolStripMenuItem->Name = L"lightingToolStripMenuItem";
			this->lightingToolStripMenuItem->Size = System::Drawing::Size(56, 20);
			this->lightingToolStripMenuItem->Text = L"&Lighting";
			// 
			// setLightingSourcesToolStripMenuItem
			// 
			this->setLightingSourcesToolStripMenuItem->Name = L"setLightingSourcesToolStripMenuItem";
			this->setLightingSourcesToolStripMenuItem->Size = System::Drawing::Size(171, 22);
			this->setLightingSourcesToolStripMenuItem->Text = L"&Set Lighting Sources";
			// 
			// filtersToolStripMenuItem
			// 
			this->filtersToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {this->defineFIlterToolStripMenuItem, 
				this->selectFilterToolStripMenuItem});
			this->filtersToolStripMenuItem->Name = L"filtersToolStripMenuItem";
			this->filtersToolStripMenuItem->Size = System::Drawing::Size(48, 20);
			this->filtersToolStripMenuItem->Text = L"&Filters";
			// 
			// defineFIlterToolStripMenuItem
			// 
			this->defineFIlterToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(6) {this->bandpassToolStripMenuItem, 
				this->highPassToolStripMenuItem, this->lowPassToolStripMenuItem, this->notchToolStripMenuItem, this->toolStripSeparator5, this->customToolStripMenuItem1});
			this->defineFIlterToolStripMenuItem->Name = L"defineFIlterToolStripMenuItem";
			this->defineFIlterToolStripMenuItem->Size = System::Drawing::Size(132, 22);
			this->defineFIlterToolStripMenuItem->Text = L"&Define Filter";
			// 
			// bandpassToolStripMenuItem
			// 
			this->bandpassToolStripMenuItem->Name = L"bandpassToolStripMenuItem";
			this->bandpassToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->bandpassToolStripMenuItem->Text = L"&Bandpass...";
			// 
			// highPassToolStripMenuItem
			// 
			this->highPassToolStripMenuItem->Name = L"highPassToolStripMenuItem";
			this->highPassToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->highPassToolStripMenuItem->Text = L"&High-Pass...";
			// 
			// lowPassToolStripMenuItem
			// 
			this->lowPassToolStripMenuItem->Name = L"lowPassToolStripMenuItem";
			this->lowPassToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->lowPassToolStripMenuItem->Text = L"&Low-Pass...";
			// 
			// notchToolStripMenuItem
			// 
			this->notchToolStripMenuItem->Name = L"notchToolStripMenuItem";
			this->notchToolStripMenuItem->Size = System::Drawing::Size(133, 22);
			this->notchToolStripMenuItem->Text = L"&Notch...";
			// 
			// toolStripSeparator5
			// 
			this->toolStripSeparator5->Name = L"toolStripSeparator5";
			this->toolStripSeparator5->Size = System::Drawing::Size(130, 6);
			// 
			// customToolStripMenuItem1
			// 
			this->customToolStripMenuItem1->Name = L"customToolStripMenuItem1";
			this->customToolStripMenuItem1->Size = System::Drawing::Size(133, 22);
			this->customToolStripMenuItem1->Text = L"&Custom...";
			// 
			// selectFilterToolStripMenuItem
			// 
			this->selectFilterToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->noneToolStripMenuItem, 
				this->toolStripSeparator4, this->customToolStripMenuItem});
			this->selectFilterToolStripMenuItem->Name = L"selectFilterToolStripMenuItem";
			this->selectFilterToolStripMenuItem->Size = System::Drawing::Size(132, 22);
			this->selectFilterToolStripMenuItem->Text = L"&Select Filter";
			// 
			// noneToolStripMenuItem
			// 
			this->noneToolStripMenuItem->Name = L"noneToolStripMenuItem";
			this->noneToolStripMenuItem->Size = System::Drawing::Size(116, 22);
			this->noneToolStripMenuItem->Text = L"&None";
			// 
			// toolStripSeparator4
			// 
			this->toolStripSeparator4->Name = L"toolStripSeparator4";
			this->toolStripSeparator4->Size = System::Drawing::Size(113, 6);
			// 
			// customToolStripMenuItem
			// 
			this->customToolStripMenuItem->Name = L"customToolStripMenuItem";
			this->customToolStripMenuItem->Size = System::Drawing::Size(116, 22);
			this->customToolStripMenuItem->Text = L"(custom)";
			// 
			// tabs
			// 
			this->tabs->Controls->Add(this->tabSummary);
			this->tabs->Controls->Add(this->tabAtmos);
			this->tabs->Controls->Add(this->tabResults);
			this->tabs->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tabs->Location = System::Drawing::Point(3, 3);
			this->tabs->Name = L"tabs";
			this->tabs->SelectedIndex = 0;
			this->tabs->Size = System::Drawing::Size(794, 535);
			this->tabs->TabIndex = 4;
			// 
			// tabSummary
			// 
			this->tabSummary->Controls->Add(this->tableLayoutPanel3);
			this->tabSummary->Location = System::Drawing::Point(4, 22);
			this->tabSummary->Name = L"tabSummary";
			this->tabSummary->Padding = System::Windows::Forms::Padding(3);
			this->tabSummary->Size = System::Drawing::Size(786, 509);
			this->tabSummary->TabIndex = 0;
			this->tabSummary->Text = L"Run Summary";
			this->tabSummary->UseVisualStyleBackColor = true;
			// 
			// tableLayoutPanel3
			// 
			this->tableLayoutPanel3->ColumnCount = 2;
			this->tableLayoutPanel3->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent, 
				50)));
			this->tableLayoutPanel3->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent, 
				50)));
			this->tableLayoutPanel3->Controls->Add(this->label4, 0, 0);
			this->tableLayoutPanel3->Controls->Add(this->flowLayoutPanel3, 1, 0);
			this->tableLayoutPanel3->Controls->Add(this->label8, 0, 2);
			this->tableLayoutPanel3->Controls->Add(this->pictureBox1, 1, 3);
			this->tableLayoutPanel3->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel3->Location = System::Drawing::Point(3, 3);
			this->tableLayoutPanel3->Name = L"tableLayoutPanel3";
			this->tableLayoutPanel3->RowCount = 6;
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 35)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 20)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 20)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 100)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 20)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 20)));
			this->tableLayoutPanel3->Size = System::Drawing::Size(780, 503);
			this->tableLayoutPanel3->TabIndex = 1;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label4->Location = System::Drawing::Point(3, 0);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(384, 35);
			this->label4->TabIndex = 0;
			this->label4->Text = L"Welcome to rtmath";
			this->label4->TextAlign = System::Drawing::ContentAlignment::MiddleLeft;
			// 
			// flowLayoutPanel3
			// 
			this->flowLayoutPanel3->Controls->Add(this->button9);
			this->flowLayoutPanel3->Controls->Add(this->button10);
			this->flowLayoutPanel3->Controls->Add(this->button11);
			this->flowLayoutPanel3->Dock = System::Windows::Forms::DockStyle::Fill;
			this->flowLayoutPanel3->Location = System::Drawing::Point(393, 3);
			this->flowLayoutPanel3->Name = L"flowLayoutPanel3";
			this->flowLayoutPanel3->Size = System::Drawing::Size(384, 29);
			this->flowLayoutPanel3->TabIndex = 1;
			// 
			// button9
			// 
			this->button9->Location = System::Drawing::Point(3, 3);
			this->button9->Name = L"button9";
			this->button9->Size = System::Drawing::Size(75, 23);
			this->button9->TabIndex = 0;
			this->button9->Text = L"&New Run";
			this->button9->UseVisualStyleBackColor = true;
			// 
			// button10
			// 
			this->button10->Location = System::Drawing::Point(84, 3);
			this->button10->Name = L"button10";
			this->button10->Size = System::Drawing::Size(75, 23);
			this->button10->TabIndex = 1;
			this->button10->Text = L"&Open Run";
			this->button10->UseVisualStyleBackColor = true;
			// 
			// button11
			// 
			this->button11->Location = System::Drawing::Point(165, 3);
			this->button11->Name = L"button11";
			this->button11->Size = System::Drawing::Size(75, 23);
			this->button11->TabIndex = 2;
			this->button11->Text = L"&Save Run";
			this->button11->UseVisualStyleBackColor = true;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label8->Location = System::Drawing::Point(3, 55);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(384, 20);
			this->label8->TabIndex = 2;
			this->label8->Text = L"Run Parameters:";
			// 
			// pictureBox1
			// 
			this->pictureBox1->Image = (cli::safe_cast<System::Drawing::Image^  >(resources->GetObject(L"pictureBox1.Image")));
			this->pictureBox1->Location = System::Drawing::Point(393, 78);
			this->pictureBox1->Name = L"pictureBox1";
			this->pictureBox1->Size = System::Drawing::Size(188, 212);
			this->pictureBox1->TabIndex = 3;
			this->pictureBox1->TabStop = false;
			// 
			// tabAtmos
			// 
			this->tabAtmos->Controls->Add(this->panelAtmosLayout);
			this->tabAtmos->Location = System::Drawing::Point(4, 22);
			this->tabAtmos->Name = L"tabAtmos";
			this->tabAtmos->Size = System::Drawing::Size(786, 509);
			this->tabAtmos->TabIndex = 2;
			this->tabAtmos->Text = L"Model";
			this->tabAtmos->UseVisualStyleBackColor = true;
			// 
			// panelAtmosLayout
			// 
			this->panelAtmosLayout->ColumnCount = 2;
			this->panelAtmosLayout->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent, 
				50)));
			this->panelAtmosLayout->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent, 
				50)));
			this->panelAtmosLayout->Controls->Add(this->label1, 0, 1);
			this->panelAtmosLayout->Controls->Add(this->flowLayoutPanel1, 0, 3);
			this->panelAtmosLayout->Controls->Add(this->label2, 1, 1);
			this->panelAtmosLayout->Controls->Add(this->layerProps, 1, 2);
			this->panelAtmosLayout->Controls->Add(this->label3, 0, 4);
			this->panelAtmosLayout->Controls->Add(this->flowLayoutPanel2, 0, 0);
			this->panelAtmosLayout->Controls->Add(this->numericUpDown1, 1, 4);
			this->panelAtmosLayout->Controls->Add(this->chkAdvAtmos, 1, 0);
			this->panelAtmosLayout->Controls->Add(this->listAtmosLayers, 0, 2);
			this->panelAtmosLayout->Dock = System::Windows::Forms::DockStyle::Fill;
			this->panelAtmosLayout->Location = System::Drawing::Point(0, 0);
			this->panelAtmosLayout->Name = L"panelAtmosLayout";
			this->panelAtmosLayout->RowCount = 5;
			this->panelAtmosLayout->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 35)));
			this->panelAtmosLayout->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 15)));
			this->panelAtmosLayout->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 100)));
			this->panelAtmosLayout->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 35)));
			this->panelAtmosLayout->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 24)));
			this->panelAtmosLayout->Size = System::Drawing::Size(786, 509);
			this->panelAtmosLayout->TabIndex = 0;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label1->Location = System::Drawing::Point(3, 35);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(387, 15);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Specify the model parameters:";
			this->label1->TextAlign = System::Drawing::ContentAlignment::BottomLeft;
			// 
			// flowLayoutPanel1
			// 
			this->flowLayoutPanel1->Controls->Add(this->button2);
			this->flowLayoutPanel1->Controls->Add(this->button4);
			this->flowLayoutPanel1->Controls->Add(this->button5);
			this->flowLayoutPanel1->Controls->Add(this->button6);
			this->flowLayoutPanel1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->flowLayoutPanel1->Location = System::Drawing::Point(3, 453);
			this->flowLayoutPanel1->Name = L"flowLayoutPanel1";
			this->flowLayoutPanel1->Size = System::Drawing::Size(387, 29);
			this->flowLayoutPanel1->TabIndex = 2;
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(3, 3);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(75, 23);
			this->button2->TabIndex = 0;
			this->button2->Text = L"&Add";
			this->button2->UseVisualStyleBackColor = true;
			// 
			// button4
			// 
			this->button4->Location = System::Drawing::Point(84, 3);
			this->button4->Name = L"button4";
			this->button4->Size = System::Drawing::Size(75, 23);
			this->button4->TabIndex = 2;
			this->button4->Text = L"&Remove";
			this->button4->UseVisualStyleBackColor = true;
			// 
			// button5
			// 
			this->button5->Location = System::Drawing::Point(165, 3);
			this->button5->Name = L"button5";
			this->button5->Size = System::Drawing::Size(75, 23);
			this->button5->TabIndex = 3;
			this->button5->Text = L"Move &Up";
			this->button5->UseVisualStyleBackColor = true;
			// 
			// button6
			// 
			this->button6->Location = System::Drawing::Point(246, 3);
			this->button6->Name = L"button6";
			this->button6->Size = System::Drawing::Size(75, 23);
			this->button6->TabIndex = 4;
			this->button6->Text = L"Move &Down";
			this->button6->UseVisualStyleBackColor = true;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label2->Location = System::Drawing::Point(396, 35);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(387, 15);
			this->label2->TabIndex = 3;
			this->label2->Text = L"Selected Layer Properties:";
			this->label2->TextAlign = System::Drawing::ContentAlignment::BottomLeft;
			// 
			// layerProps
			// 
			this->layerProps->Dock = System::Windows::Forms::DockStyle::Fill;
			this->layerProps->Location = System::Drawing::Point(396, 53);
			this->layerProps->Name = L"layerProps";
			this->layerProps->Size = System::Drawing::Size(387, 394);
			this->layerProps->TabIndex = 4;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label3->Location = System::Drawing::Point(3, 485);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(387, 24);
			this->label3->TabIndex = 5;
			this->label3->Text = L"Number of Homogeneous Doubling-Adding &Layers:";
			this->label3->TextAlign = System::Drawing::ContentAlignment::MiddleRight;
			// 
			// flowLayoutPanel2
			// 
			this->flowLayoutPanel2->Controls->Add(this->button3);
			this->flowLayoutPanel2->Controls->Add(this->button7);
			this->flowLayoutPanel2->Controls->Add(this->button8);
			this->flowLayoutPanel2->Dock = System::Windows::Forms::DockStyle::Fill;
			this->flowLayoutPanel2->Location = System::Drawing::Point(3, 3);
			this->flowLayoutPanel2->Name = L"flowLayoutPanel2";
			this->flowLayoutPanel2->Size = System::Drawing::Size(387, 29);
			this->flowLayoutPanel2->TabIndex = 7;
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(3, 3);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(75, 23);
			this->button3->TabIndex = 0;
			this->button3->Text = L"&New";
			this->button3->UseVisualStyleBackColor = true;
			// 
			// button7
			// 
			this->button7->Location = System::Drawing::Point(84, 3);
			this->button7->Name = L"button7";
			this->button7->Size = System::Drawing::Size(75, 23);
			this->button7->TabIndex = 1;
			this->button7->Text = L"&Open";
			this->button7->UseVisualStyleBackColor = true;
			// 
			// button8
			// 
			this->button8->Location = System::Drawing::Point(165, 3);
			this->button8->Name = L"button8";
			this->button8->Size = System::Drawing::Size(75, 23);
			this->button8->TabIndex = 2;
			this->button8->Text = L"&Save";
			this->button8->UseVisualStyleBackColor = true;
			// 
			// numericUpDown1
			// 
			this->numericUpDown1->Location = System::Drawing::Point(396, 488);
			this->numericUpDown1->Maximum = System::Decimal(gcnew cli::array< System::Int32 >(4) {256, 0, 0, 0});
			this->numericUpDown1->Minimum = System::Decimal(gcnew cli::array< System::Int32 >(4) {1, 0, 0, 0});
			this->numericUpDown1->Name = L"numericUpDown1";
			this->numericUpDown1->Size = System::Drawing::Size(120, 20);
			this->numericUpDown1->TabIndex = 8;
			this->numericUpDown1->Value = System::Decimal(gcnew cli::array< System::Int32 >(4) {64, 0, 0, 0});
			// 
			// chkAdvAtmos
			// 
			this->chkAdvAtmos->AutoSize = true;
			this->chkAdvAtmos->Dock = System::Windows::Forms::DockStyle::Fill;
			this->chkAdvAtmos->Location = System::Drawing::Point(396, 3);
			this->chkAdvAtmos->Name = L"chkAdvAtmos";
			this->chkAdvAtmos->Size = System::Drawing::Size(387, 29);
			this->chkAdvAtmos->TabIndex = 9;
			this->chkAdvAtmos->Text = L"Enable Advanced Atmospheric Profiles";
			this->chkAdvAtmos->UseVisualStyleBackColor = true;
			// 
			// listAtmosLayers
			// 
			this->listAtmosLayers->Dock = System::Windows::Forms::DockStyle::Fill;
			this->listAtmosLayers->Location = System::Drawing::Point(3, 53);
			this->listAtmosLayers->Name = L"listAtmosLayers";
			treeNode1->Name = L"nodeBaseAtmos";
			treeNode1->Text = L"Base Atmosphere Selection";
			treeNode1->ToolTipText = resources->GetString(L"treeNode1.ToolTipText");
			treeNode2->Name = L"nodeExCld1";
			treeNode2->Text = L"Example Cloud Layer 1";
			treeNode2->ToolTipText = L"A node for an example cloud layer has been created.";
			treeNode3->Name = L"Node5";
			treeNode3->Text = L"Example Cloud Layer 2";
			treeNode3->ToolTipText = L"A node for an example cloud layer has been created.";
			treeNode4->Name = L"nodeClouds";
			treeNode4->Text = L"Clouds";
			treeNode5->Name = L"Node6";
			treeNode5->Text = L"CO2";
			treeNode6->Name = L"Node7";
			treeNode6->Text = L"H2O";
			treeNode7->Name = L"nodeGases";
			treeNode7->Text = L"Gases";
			treeNode7->ToolTipText = L"Gases to use in calculations. This acts as a master switch to use or ignore the g" 
				L"as profile in the base atmosphere. Base profiles may also be overridden here.";
			treeNode8->Name = L"nodeAerosols";
			treeNode8->Text = L"Aerosols";
			treeNode8->ToolTipText = L"Enter any aerosols here.";
			treeNode9->Name = L"nodeSun";
			treeNode9->Text = L"Sun";
			treeNode10->Name = L"nodeLighting";
			treeNode10->Text = L"Lighting";
			treeNode11->Name = L"nodeSurface";
			treeNode11->Text = L"Surface Model";
			treeNode12->Name = L"nodeFilters";
			treeNode12->Text = L"Filters";
			treeNode13->Name = L"nodeRegion";
			treeNode13->Text = L"Spectral Region";
			this->listAtmosLayers->Nodes->AddRange(gcnew cli::array< System::Windows::Forms::TreeNode^  >(8) {treeNode1, treeNode4, treeNode7, 
				treeNode8, treeNode10, treeNode11, treeNode12, treeNode13});
			this->listAtmosLayers->ShowNodeToolTips = true;
			this->listAtmosLayers->Size = System::Drawing::Size(387, 394);
			this->listAtmosLayers->TabIndex = 10;
			// 
			// tabResults
			// 
			this->tabResults->Controls->Add(this->splitContainer1);
			this->tabResults->Location = System::Drawing::Point(4, 22);
			this->tabResults->Name = L"tabResults";
			this->tabResults->Size = System::Drawing::Size(786, 509);
			this->tabResults->TabIndex = 6;
			this->tabResults->Text = L"Results";
			this->tabResults->UseVisualStyleBackColor = true;
			this->tabResults->Click += gcnew System::EventHandler(this, &frmMain::tabResults_Click);
			// 
			// splitContainer1
			// 
			this->splitContainer1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->splitContainer1->Location = System::Drawing::Point(0, 0);
			this->splitContainer1->Name = L"splitContainer1";
			// 
			// splitContainer1.Panel1
			// 
			this->splitContainer1->Panel1->Controls->Add(this->glgraphwinControl1);
			// 
			// splitContainer1.Panel2
			// 
			this->splitContainer1->Panel2->Controls->Add(this->splitContainer2);
			this->splitContainer1->Size = System::Drawing::Size(786, 509);
			this->splitContainer1->SplitterDistance = 568;
			this->splitContainer1->TabIndex = 4;
			// 
			// glgraphwinControl1
			// 
			this->glgraphwinControl1->AutoSize = true;
			this->glgraphwinControl1->BackColor = System::Drawing::SystemColors::ControlDark;
			this->glgraphwinControl1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->glgraphwinControl1->Location = System::Drawing::Point(0, 0);
			this->glgraphwinControl1->Name = L"glgraphwinControl1";
			this->glgraphwinControl1->Size = System::Drawing::Size(568, 509);
			this->glgraphwinControl1->TabIndex = 1;
			this->glgraphwinControl1->TabStop = false;
			this->glgraphwinControl1->UIoverride = false;
			this->glgraphwinControl1->Visible = true;
			// 
			// splitContainer2
			// 
			this->splitContainer2->Dock = System::Windows::Forms::DockStyle::Fill;
			this->splitContainer2->Location = System::Drawing::Point(0, 0);
			this->splitContainer2->Name = L"splitContainer2";
			this->splitContainer2->Orientation = System::Windows::Forms::Orientation::Horizontal;
			// 
			// splitContainer2.Panel1
			// 
			this->splitContainer2->Panel1->Controls->Add(this->treePlots);
			// 
			// splitContainer2.Panel2
			// 
			this->splitContainer2->Panel2->Controls->Add(this->propPlots);
			this->splitContainer2->Size = System::Drawing::Size(214, 509);
			this->splitContainer2->SplitterDistance = 254;
			this->splitContainer2->TabIndex = 4;
			// 
			// treePlots
			// 
			this->treePlots->CheckBoxes = true;
			this->treePlots->Dock = System::Windows::Forms::DockStyle::Fill;
			this->treePlots->Location = System::Drawing::Point(0, 0);
			this->treePlots->Name = L"treePlots";
			treeNode14->Checked = true;
			treeNode14->Name = L"Node1";
			treeNode14->Text = L"Sample Axis Line 1";
			treeNode15->Name = L"Node2";
			treeNode15->Text = L"Sample Axis Line 2";
			treeNode16->Name = L"Node0";
			treeNode16->Text = L"Model Run 1";
			treeNode17->Name = L"Node4";
			treeNode17->Text = L"Upwelling Radiance at TOA";
			treeNode18->Name = L"Node3";
			treeNode18->Text = L"Model Run 2";
			this->treePlots->Nodes->AddRange(gcnew cli::array< System::Windows::Forms::TreeNode^  >(2) {treeNode16, treeNode18});
			this->treePlots->Size = System::Drawing::Size(214, 254);
			this->treePlots->TabIndex = 0;
			// 
			// propPlots
			// 
			this->propPlots->Dock = System::Windows::Forms::DockStyle::Fill;
			this->propPlots->Location = System::Drawing::Point(0, 0);
			this->propPlots->Name = L"propPlots";
			this->propPlots->PropertySort = System::Windows::Forms::PropertySort::Alphabetical;
			this->propPlots->Size = System::Drawing::Size(214, 251);
			this->propPlots->TabIndex = 4;
			this->propPlots->ToolbarVisible = false;
			// 
			// statusbar
			// 
			this->statusbar->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(3) {this->toolStripStatusLabel1, 
				this->toolStripProgressBar1, this->toolStripStatusLabel2});
			this->statusbar->Location = System::Drawing::Point(0, 573);
			this->statusbar->Name = L"statusbar";
			this->statusbar->Size = System::Drawing::Size(800, 22);
			this->statusbar->TabIndex = 5;
			this->statusbar->Text = L"statusStrip1";
			// 
			// toolStripStatusLabel1
			// 
			this->toolStripStatusLabel1->Name = L"toolStripStatusLabel1";
			this->toolStripStatusLabel1->Size = System::Drawing::Size(75, 17);
			this->toolStripStatusLabel1->Text = L"Run Progress:";
			// 
			// toolStripProgressBar1
			// 
			this->toolStripProgressBar1->Name = L"toolStripProgressBar1";
			this->toolStripProgressBar1->Size = System::Drawing::Size(300, 16);
			// 
			// toolStripStatusLabel2
			// 
			this->toolStripStatusLabel2->Name = L"toolStripStatusLabel2";
			this->toolStripStatusLabel2->Size = System::Drawing::Size(64, 17);
			this->toolStripStatusLabel2->Text = L"(run status)";
			// 
			// tableLayoutPanel1
			// 
			this->tableLayoutPanel1->ColumnCount = 1;
			this->tableLayoutPanel1->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent, 
				50)));
			this->tableLayoutPanel1->Controls->Add(this->tabs, 0, 0);
			this->tableLayoutPanel1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel1->Location = System::Drawing::Point(0, 24);
			this->tableLayoutPanel1->Name = L"tableLayoutPanel1";
			this->tableLayoutPanel1->RowCount = 2;
			this->tableLayoutPanel1->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 98.54281F)));
			this->tableLayoutPanel1->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 1.457195F)));
			this->tableLayoutPanel1->Size = System::Drawing::Size(800, 549);
			this->tableLayoutPanel1->TabIndex = 6;
			// 
			// dlgOpen
			// 
			this->dlgOpen->FileName = L"openFileDialog1";
			// 
			// frmMain
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(800, 595);
			this->Controls->Add(this->tableLayoutPanel1);
			this->Controls->Add(this->statusbar);
			this->Controls->Add(this->menuStrip1);
			this->Icon = (cli::safe_cast<System::Drawing::Icon^  >(resources->GetObject(L"$this.Icon")));
			this->MainMenuStrip = this->menuStrip1;
			this->Name = L"frmMain";
			this->Text = L"rtmath";
			this->Load += gcnew System::EventHandler(this, &frmMain::frmMain_Load);
			this->Move += gcnew System::EventHandler(this, &frmMain::frmMain_Move);
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			this->tabs->ResumeLayout(false);
			this->tabSummary->ResumeLayout(false);
			this->tableLayoutPanel3->ResumeLayout(false);
			this->tableLayoutPanel3->PerformLayout();
			this->flowLayoutPanel3->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->pictureBox1))->EndInit();
			this->tabAtmos->ResumeLayout(false);
			this->panelAtmosLayout->ResumeLayout(false);
			this->panelAtmosLayout->PerformLayout();
			this->flowLayoutPanel1->ResumeLayout(false);
			this->flowLayoutPanel2->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->numericUpDown1))->EndInit();
			this->tabResults->ResumeLayout(false);
			this->splitContainer1->Panel1->ResumeLayout(false);
			this->splitContainer1->Panel1->PerformLayout();
			this->splitContainer1->Panel2->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->splitContainer1))->EndInit();
			this->splitContainer1->ResumeLayout(false);
			this->splitContainer2->Panel1->ResumeLayout(false);
			this->splitContainer2->Panel2->ResumeLayout(false);
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^  >(this->splitContainer2))->EndInit();
			this->splitContainer2->ResumeLayout(false);
			this->statusbar->ResumeLayout(false);
			this->statusbar->PerformLayout();
			this->tableLayoutPanel1->ResumeLayout(false);
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
private: System::Void frmMain_Move(System::Object^  sender, System::EventArgs^  e) {
			 this->glgraphwinControl1->render();
		 }
private: System::Void tabResults_Click(System::Object^  sender, System::EventArgs^  e) {
			 this->glgraphwinControl1->redraw();
			 this->glgraphwinControl1->render();
		 }
};
}
