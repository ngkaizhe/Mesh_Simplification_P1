#pragma once

#include "Common.h"
#include "DotNetUtilities.h"
#include "Shader.h"
#include "ViewManager.h"
#include "MeshObject.h"

using namespace std;
using namespace glm;

float			aspect;
ViewManager		m_camera;

MeshObject* meshObjectPtr;
Shader shader;

bool meshInited = false;

Mode currentMode = Mode::QEM;

int currentSSMRate = 0;

namespace OpenMesh_EX {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// MyForm 的摘要
	/// </summary>
	public ref class MyForm : public System::Windows::Forms::Form
	{
	public:
		MyForm(void)
		{
			InitializeComponent();
			//
			//TODO:  在此加入建構函式程式碼
			//
		}

	protected:
		/// <summary>
		/// 清除任何使用中的資源。
		/// </summary>
		~MyForm()
		{
			if (components)
			{
				delete components;
			}
		}

	private: System::Windows::Forms::MenuStrip^  menuStrip1;
	private: System::Windows::Forms::ToolStripMenuItem^  fileToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^  loadModelToolStripMenuItem;
	private: System::Windows::Forms::OpenFileDialog^  openModelDialog;
	private: System::Windows::Forms::SaveFileDialog^  saveModelDialog;
	private: System::Windows::Forms::ToolStripMenuItem^  saveModelToolStripMenuItem;
	private: HKOGLPanel::HKOGLPanelControl^  hkoglPanelControl1;
	private: System::Windows::Forms::Timer^ timer1;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel1;

	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel2;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel3;
	private: System::Windows::Forms::Label^ label1;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel6;
	private: System::Windows::Forms::Label^ label6;
	private: System::Windows::Forms::Label^ label7;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel5;
	private: System::Windows::Forms::Label^ label4;
	private: System::Windows::Forms::Label^ label5;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel4;
	private: System::Windows::Forms::Label^ label3;
	private: System::Windows::Forms::Label^ label2;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel7;
	private: System::Windows::Forms::Label^ label8;
	private: System::Windows::Forms::Label^ label9;
	private: System::Windows::Forms::Label^ label12;
	private: System::Windows::Forms::Label^ label10;
	private: System::Windows::Forms::TrackBar^ trackBar1;
	private: System::Windows::Forms::Label^ label11;
	private: System::Windows::Forms::Label^ label13;
	private: System::Windows::Forms::Button^ button1;
	private: System::Windows::Forms::Label^ label14;
	private: System::Windows::Forms::Label^ label15;
	private: System::Windows::Forms::TableLayoutPanel^ tableLayoutPanel8;
	private: System::Windows::Forms::Label^ label16;
	private: System::Windows::Forms::Label^ label17;
	private: System::Windows::Forms::TrackBar^ trackBar2;
	private: System::Windows::Forms::ToolStripMenuItem^ modeToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^ qEMToolStripMenuItem;
	private: System::Windows::Forms::ToolStripMenuItem^ sSMToolStripMenuItem;









	private: System::ComponentModel::IContainer^ components;
	protected:

	private:
		/// <summary>
		/// 設計工具所需的變數。
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// 此為設計工具支援所需的方法 - 請勿使用程式碼編輯器修改
		/// 這個方法的內容。
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			HKOGLPanel::HKCOGLPanelCameraSetting^ hkcoglPanelCameraSetting1 = (gcnew HKOGLPanel::HKCOGLPanelCameraSetting());
			HKOGLPanel::HKCOGLPanelPixelFormat^ hkcoglPanelPixelFormat1 = (gcnew HKOGLPanel::HKCOGLPanelPixelFormat());
			this->menuStrip1 = (gcnew System::Windows::Forms::MenuStrip());
			this->fileToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->loadModelToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->saveModelToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->modeToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->qEMToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->sSMToolStripMenuItem = (gcnew System::Windows::Forms::ToolStripMenuItem());
			this->openModelDialog = (gcnew System::Windows::Forms::OpenFileDialog());
			this->saveModelDialog = (gcnew System::Windows::Forms::SaveFileDialog());
			this->hkoglPanelControl1 = (gcnew HKOGLPanel::HKOGLPanelControl());
			this->timer1 = (gcnew System::Windows::Forms::Timer(this->components));
			this->tableLayoutPanel1 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->tableLayoutPanel3 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->tableLayoutPanel6 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->tableLayoutPanel5 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->tableLayoutPanel4 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->tableLayoutPanel7 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->trackBar1 = (gcnew System::Windows::Forms::TrackBar());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->tableLayoutPanel8 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->trackBar2 = (gcnew System::Windows::Forms::TrackBar());
			this->tableLayoutPanel2 = (gcnew System::Windows::Forms::TableLayoutPanel());
			this->menuStrip1->SuspendLayout();
			this->tableLayoutPanel1->SuspendLayout();
			this->tableLayoutPanel3->SuspendLayout();
			this->tableLayoutPanel6->SuspendLayout();
			this->tableLayoutPanel5->SuspendLayout();
			this->tableLayoutPanel4->SuspendLayout();
			this->tableLayoutPanel7->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->BeginInit();
			this->tableLayoutPanel8->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar2))->BeginInit();
			this->tableLayoutPanel2->SuspendLayout();
			this->SuspendLayout();
			// 
			// menuStrip1
			// 
			this->menuStrip1->Items->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {
				this->fileToolStripMenuItem,
					this->modeToolStripMenuItem
			});
			this->menuStrip1->Location = System::Drawing::Point(0, 0);
			this->menuStrip1->Name = L"menuStrip1";
			this->menuStrip1->Size = System::Drawing::Size(1239, 24);
			this->menuStrip1->TabIndex = 1;
			this->menuStrip1->Text = L"menuStrip1";
			// 
			// fileToolStripMenuItem
			// 
			this->fileToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {
				this->loadModelToolStripMenuItem,
					this->saveModelToolStripMenuItem
			});
			this->fileToolStripMenuItem->Name = L"fileToolStripMenuItem";
			this->fileToolStripMenuItem->Size = System::Drawing::Size(37, 20);
			this->fileToolStripMenuItem->Text = L"File";
			// 
			// loadModelToolStripMenuItem
			// 
			this->loadModelToolStripMenuItem->Name = L"loadModelToolStripMenuItem";
			this->loadModelToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->loadModelToolStripMenuItem->Text = L"Load Model";
			this->loadModelToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::loadModelToolStripMenuItem_Click);
			// 
			// saveModelToolStripMenuItem
			// 
			this->saveModelToolStripMenuItem->Name = L"saveModelToolStripMenuItem";
			this->saveModelToolStripMenuItem->Size = System::Drawing::Size(137, 22);
			this->saveModelToolStripMenuItem->Text = L"Save Model";
			this->saveModelToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::saveModelToolStripMenuItem_Click);
			// 
			// modeToolStripMenuItem
			// 
			this->modeToolStripMenuItem->DropDownItems->AddRange(gcnew cli::array< System::Windows::Forms::ToolStripItem^  >(2) {
				this->qEMToolStripMenuItem,
					this->sSMToolStripMenuItem
			});
			this->modeToolStripMenuItem->Name = L"modeToolStripMenuItem";
			this->modeToolStripMenuItem->Size = System::Drawing::Size(50, 20);
			this->modeToolStripMenuItem->Text = L"Mode";
			// 
			// qEMToolStripMenuItem
			// 
			this->qEMToolStripMenuItem->Name = L"qEMToolStripMenuItem";
			this->qEMToolStripMenuItem->Size = System::Drawing::Size(100, 22);
			this->qEMToolStripMenuItem->Text = L"QEM";
			this->qEMToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::qEMToolStripMenuItem_Click);
			// 
			// sSMToolStripMenuItem
			// 
			this->sSMToolStripMenuItem->Name = L"sSMToolStripMenuItem";
			this->sSMToolStripMenuItem->Size = System::Drawing::Size(100, 22);
			this->sSMToolStripMenuItem->Text = L"SSM";
			this->sSMToolStripMenuItem->Click += gcnew System::EventHandler(this, &MyForm::sSMToolStripMenuItem_Click);
			// 
			// openModelDialog
			// 
			this->openModelDialog->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &MyForm::openModelDialog_FileOk);
			// 
			// saveModelDialog
			// 
			this->saveModelDialog->DefaultExt = L"obj";
			this->saveModelDialog->FileOk += gcnew System::ComponentModel::CancelEventHandler(this, &MyForm::saveModelDialog_FileOk);
			// 
			// hkoglPanelControl1
			// 
			hkcoglPanelCameraSetting1->Far = 1000;
			hkcoglPanelCameraSetting1->Fov = 45;
			hkcoglPanelCameraSetting1->Near = -1000;
			hkcoglPanelCameraSetting1->Type = HKOGLPanel::HKCOGLPanelCameraSetting::CAMERATYPE::ORTHOGRAPHIC;
			this->hkoglPanelControl1->Camera_Setting = hkcoglPanelCameraSetting1;
			this->hkoglPanelControl1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->hkoglPanelControl1->Location = System::Drawing::Point(3, 3);
			this->hkoglPanelControl1->Name = L"hkoglPanelControl1";
			hkcoglPanelPixelFormat1->Accumu_Buffer_Bits = HKOGLPanel::HKCOGLPanelPixelFormat::PIXELBITS::BITS_0;
			hkcoglPanelPixelFormat1->Alpha_Buffer_Bits = HKOGLPanel::HKCOGLPanelPixelFormat::PIXELBITS::BITS_0;
			hkcoglPanelPixelFormat1->Stencil_Buffer_Bits = HKOGLPanel::HKCOGLPanelPixelFormat::PIXELBITS::BITS_0;
			this->hkoglPanelControl1->Pixel_Format = hkcoglPanelPixelFormat1;
			this->hkoglPanelControl1->Size = System::Drawing::Size(610, 668);
			this->hkoglPanelControl1->TabIndex = 2;
			this->hkoglPanelControl1->Load += gcnew System::EventHandler(this, &MyForm::hkoglPanelControl1_Load);
			this->hkoglPanelControl1->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &MyForm::hkoglPanelControl1_Paint);
			this->hkoglPanelControl1->KeyPress += gcnew System::Windows::Forms::KeyPressEventHandler(this, &MyForm::hkoglPanelControl1_KeyPress);
			this->hkoglPanelControl1->MouseDown += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseDown);
			this->hkoglPanelControl1->MouseMove += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseMove);
			this->hkoglPanelControl1->MouseUp += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseUp);
			this->hkoglPanelControl1->MouseWheel += gcnew System::Windows::Forms::MouseEventHandler(this, &MyForm::hkoglPanelControl1_MouseWheel);
			this->hkoglPanelControl1->Resize += gcnew System::EventHandler(this, &MyForm::hkoglPanelControl1_Resize);
			// 
			// timer1
			// 
			this->timer1->Enabled = true;
			this->timer1->Interval = 10;
			this->timer1->Tick += gcnew System::EventHandler(this, &MyForm::timer1_Tick);
			// 
			// tableLayoutPanel1
			// 
			this->tableLayoutPanel1->ColumnCount = 2;
			this->tableLayoutPanel1->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel1->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel1->Controls->Add(this->hkoglPanelControl1, 0, 0);
			this->tableLayoutPanel1->Controls->Add(this->tableLayoutPanel3, 1, 0);
			this->tableLayoutPanel1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel1->Location = System::Drawing::Point(3, 3);
			this->tableLayoutPanel1->Name = L"tableLayoutPanel1";
			this->tableLayoutPanel1->RowCount = 1;
			this->tableLayoutPanel1->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 49)));
			this->tableLayoutPanel1->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 674)));
			this->tableLayoutPanel1->Size = System::Drawing::Size(1233, 674);
			this->tableLayoutPanel1->TabIndex = 3;
			// 
			// tableLayoutPanel3
			// 
			this->tableLayoutPanel3->ColumnCount = 1;
			this->tableLayoutPanel3->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				100)));
			this->tableLayoutPanel3->Controls->Add(this->label12, 0, 4);
			this->tableLayoutPanel3->Controls->Add(this->tableLayoutPanel6, 0, 3);
			this->tableLayoutPanel3->Controls->Add(this->tableLayoutPanel5, 0, 2);
			this->tableLayoutPanel3->Controls->Add(this->label1, 0, 0);
			this->tableLayoutPanel3->Controls->Add(this->tableLayoutPanel4, 0, 1);
			this->tableLayoutPanel3->Controls->Add(this->label13, 0, 8);
			this->tableLayoutPanel3->Controls->Add(this->label11, 0, 5);
			this->tableLayoutPanel3->Controls->Add(this->label10, 0, 9);
			this->tableLayoutPanel3->Controls->Add(this->tableLayoutPanel7, 0, 6);
			this->tableLayoutPanel3->Controls->Add(this->button1, 0, 10);
			this->tableLayoutPanel3->Controls->Add(this->trackBar1, 0, 7);
			this->tableLayoutPanel3->Controls->Add(this->label14, 0, 11);
			this->tableLayoutPanel3->Controls->Add(this->label15, 0, 12);
			this->tableLayoutPanel3->Controls->Add(this->tableLayoutPanel8, 0, 13);
			this->tableLayoutPanel3->Controls->Add(this->trackBar2, 0, 14);
			this->tableLayoutPanel3->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel3->Location = System::Drawing::Point(619, 3);
			this->tableLayoutPanel3->Name = L"tableLayoutPanel3";
			this->tableLayoutPanel3->RowCount = 16;
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 36)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 44)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 40)));
			this->tableLayoutPanel3->RowStyles->Add((gcnew System::Windows::Forms::RowStyle()));
			this->tableLayoutPanel3->Size = System::Drawing::Size(611, 668);
			this->tableLayoutPanel3->TabIndex = 3;
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label12->Location = System::Drawing::Point(3, 160);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(605, 40);
			this->label12->TabIndex = 7;
			this->label12->Text = L"================================================================================="
				L"==================";
			this->label12->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// tableLayoutPanel6
			// 
			this->tableLayoutPanel6->ColumnCount = 2;
			this->tableLayoutPanel6->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel6->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel6->Controls->Add(this->label6, 1, 0);
			this->tableLayoutPanel6->Controls->Add(this->label7, 0, 0);
			this->tableLayoutPanel6->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel6->Location = System::Drawing::Point(3, 123);
			this->tableLayoutPanel6->Name = L"tableLayoutPanel6";
			this->tableLayoutPanel6->RowCount = 1;
			this->tableLayoutPanel6->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 50)));
			this->tableLayoutPanel6->Size = System::Drawing::Size(605, 34);
			this->tableLayoutPanel6->TabIndex = 3;
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label6->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label6->Location = System::Drawing::Point(305, 0);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(297, 34);
			this->label6->TabIndex = 2;
			this->label6->Text = L"30";
			this->label6->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label7->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label7->Location = System::Drawing::Point(3, 0);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(296, 34);
			this->label7->TabIndex = 1;
			this->label7->Text = L"Face Number:";
			this->label7->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// tableLayoutPanel5
			// 
			this->tableLayoutPanel5->ColumnCount = 2;
			this->tableLayoutPanel5->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel5->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel5->Controls->Add(this->label4, 1, 0);
			this->tableLayoutPanel5->Controls->Add(this->label5, 0, 0);
			this->tableLayoutPanel5->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel5->Location = System::Drawing::Point(3, 83);
			this->tableLayoutPanel5->Name = L"tableLayoutPanel5";
			this->tableLayoutPanel5->RowCount = 1;
			this->tableLayoutPanel5->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 50)));
			this->tableLayoutPanel5->Size = System::Drawing::Size(605, 34);
			this->tableLayoutPanel5->TabIndex = 2;
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label4->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label4->Location = System::Drawing::Point(305, 0);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(297, 34);
			this->label4->TabIndex = 2;
			this->label4->Text = L"50";
			this->label4->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label5->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label5->Location = System::Drawing::Point(3, 0);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(296, 34);
			this->label5->TabIndex = 1;
			this->label5->Text = L"Edge Number:";
			this->label5->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label1->Location = System::Drawing::Point(3, 0);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(605, 40);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Mesh Info";
			this->label1->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// tableLayoutPanel4
			// 
			this->tableLayoutPanel4->ColumnCount = 2;
			this->tableLayoutPanel4->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel4->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel4->Controls->Add(this->label3, 1, 0);
			this->tableLayoutPanel4->Controls->Add(this->label2, 0, 0);
			this->tableLayoutPanel4->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel4->Location = System::Drawing::Point(3, 43);
			this->tableLayoutPanel4->Name = L"tableLayoutPanel4";
			this->tableLayoutPanel4->RowCount = 1;
			this->tableLayoutPanel4->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 50)));
			this->tableLayoutPanel4->Size = System::Drawing::Size(605, 34);
			this->tableLayoutPanel4->TabIndex = 1;
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label3->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label3->Location = System::Drawing::Point(305, 0);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(297, 34);
			this->label3->TabIndex = 2;
			this->label3->Text = L"100";
			this->label3->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label2->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label2->Location = System::Drawing::Point(3, 0);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(296, 34);
			this->label2->TabIndex = 1;
			this->label2->Text = L"Vertex Number: ";
			this->label2->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label13->Location = System::Drawing::Point(3, 316);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(605, 44);
			this->label13->TabIndex = 8;
			this->label13->Text = L"================================================================================="
				L"==================";
			this->label13->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label11->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label11->Location = System::Drawing::Point(3, 200);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(605, 40);
			this->label11->TabIndex = 6;
			this->label11->Text = L"QEM Mesh Simplification";
			this->label11->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label10->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label10->Location = System::Drawing::Point(3, 360);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(605, 40);
			this->label10->TabIndex = 4;
			this->label10->Text = L"Skeleton Extraction";
			this->label10->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// tableLayoutPanel7
			// 
			this->tableLayoutPanel7->ColumnCount = 2;
			this->tableLayoutPanel7->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel7->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel7->Controls->Add(this->label8, 1, 0);
			this->tableLayoutPanel7->Controls->Add(this->label9, 0, 0);
			this->tableLayoutPanel7->Location = System::Drawing::Point(3, 243);
			this->tableLayoutPanel7->Name = L"tableLayoutPanel7";
			this->tableLayoutPanel7->RowCount = 1;
			this->tableLayoutPanel7->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 50)));
			this->tableLayoutPanel7->Size = System::Drawing::Size(605, 34);
			this->tableLayoutPanel7->TabIndex = 4;
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label8->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label8->Location = System::Drawing::Point(305, 0);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(297, 34);
			this->label8->TabIndex = 2;
			this->label8->Text = L"0";
			this->label8->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label9->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label9->Location = System::Drawing::Point(3, 0);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(296, 34);
			this->label9->TabIndex = 1;
			this->label9->Text = L"Current QEM Simplified Rate:";
			this->label9->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(3, 403);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(605, 34);
			this->button1->TabIndex = 5;
			this->button1->Text = L"Extract Skeleton";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MyForm::button1_Click);
			// 
			// trackBar1
			// 
			this->trackBar1->Dock = System::Windows::Forms::DockStyle::Fill;
			this->trackBar1->Location = System::Drawing::Point(3, 283);
			this->trackBar1->Maximum = 100;
			this->trackBar1->Name = L"trackBar1";
			this->trackBar1->Size = System::Drawing::Size(605, 30);
			this->trackBar1->TabIndex = 4;
			this->trackBar1->ValueChanged += gcnew System::EventHandler(this, &MyForm::trackBar1_ValueChanged);
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label14->Location = System::Drawing::Point(3, 440);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(605, 40);
			this->label14->TabIndex = 9;
			this->label14->Text = L"================================================================================="
				L"==================";
			this->label14->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label15->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label15->Location = System::Drawing::Point(3, 480);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(605, 40);
			this->label15->TabIndex = 10;
			this->label15->Text = L"SSM Mesh Simplification";
			this->label15->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// tableLayoutPanel8
			// 
			this->tableLayoutPanel8->ColumnCount = 2;
			this->tableLayoutPanel8->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel8->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				50)));
			this->tableLayoutPanel8->Controls->Add(this->label16, 1, 0);
			this->tableLayoutPanel8->Controls->Add(this->label17, 0, 0);
			this->tableLayoutPanel8->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel8->Location = System::Drawing::Point(3, 523);
			this->tableLayoutPanel8->Name = L"tableLayoutPanel8";
			this->tableLayoutPanel8->RowCount = 1;
			this->tableLayoutPanel8->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Percent, 50)));
			this->tableLayoutPanel8->Size = System::Drawing::Size(605, 34);
			this->tableLayoutPanel8->TabIndex = 11;
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label16->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label16->Location = System::Drawing::Point(305, 0);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(297, 34);
			this->label16->TabIndex = 2;
			this->label16->Text = L"0";
			this->label16->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->BorderStyle = System::Windows::Forms::BorderStyle::FixedSingle;
			this->label17->Dock = System::Windows::Forms::DockStyle::Fill;
			this->label17->Location = System::Drawing::Point(3, 0);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(296, 34);
			this->label17->TabIndex = 1;
			this->label17->Text = L"Current SSM Simplified Rate:";
			this->label17->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// trackBar2
			// 
			this->trackBar2->Dock = System::Windows::Forms::DockStyle::Fill;
			this->trackBar2->Enabled = false;
			this->trackBar2->Location = System::Drawing::Point(3, 563);
			this->trackBar2->Maximum = 100;
			this->trackBar2->Name = L"trackBar2";
			this->trackBar2->Size = System::Drawing::Size(605, 34);
			this->trackBar2->TabIndex = 12;
			this->trackBar2->ValueChanged += gcnew System::EventHandler(this, &MyForm::trackBar2_ValueChanged);
			// 
			// tableLayoutPanel2
			// 
			this->tableLayoutPanel2->ColumnCount = 1;
			this->tableLayoutPanel2->ColumnStyles->Add((gcnew System::Windows::Forms::ColumnStyle(System::Windows::Forms::SizeType::Percent,
				100)));
			this->tableLayoutPanel2->Controls->Add(this->tableLayoutPanel1, 0, 0);
			this->tableLayoutPanel2->Dock = System::Windows::Forms::DockStyle::Fill;
			this->tableLayoutPanel2->Location = System::Drawing::Point(0, 24);
			this->tableLayoutPanel2->Name = L"tableLayoutPanel2";
			this->tableLayoutPanel2->RowCount = 1;
			this->tableLayoutPanel2->RowStyles->Add((gcnew System::Windows::Forms::RowStyle()));
			this->tableLayoutPanel2->RowStyles->Add((gcnew System::Windows::Forms::RowStyle(System::Windows::Forms::SizeType::Absolute, 20)));
			this->tableLayoutPanel2->Size = System::Drawing::Size(1239, 680);
			this->tableLayoutPanel2->TabIndex = 4;
			// 
			// MyForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 12);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1239, 704);
			this->Controls->Add(this->tableLayoutPanel2);
			this->Controls->Add(this->menuStrip1);
			this->MainMenuStrip = this->menuStrip1;
			this->Name = L"MyForm";
			this->Text = L"OpenMesh_EX";
			this->menuStrip1->ResumeLayout(false);
			this->menuStrip1->PerformLayout();
			this->tableLayoutPanel1->ResumeLayout(false);
			this->tableLayoutPanel3->ResumeLayout(false);
			this->tableLayoutPanel3->PerformLayout();
			this->tableLayoutPanel6->ResumeLayout(false);
			this->tableLayoutPanel6->PerformLayout();
			this->tableLayoutPanel5->ResumeLayout(false);
			this->tableLayoutPanel5->PerformLayout();
			this->tableLayoutPanel4->ResumeLayout(false);
			this->tableLayoutPanel4->PerformLayout();
			this->tableLayoutPanel7->ResumeLayout(false);
			this->tableLayoutPanel7->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar1))->EndInit();
			this->tableLayoutPanel8->ResumeLayout(false);
			this->tableLayoutPanel8->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->trackBar2))->EndInit();
			this->tableLayoutPanel2->ResumeLayout(false);
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
private: System::Void hkoglPanelControl1_Load(System::Object^  sender, System::EventArgs^  e)
{
	// glew init
	GLenum res = glewInit();
	if (res != GLEW_OK) {
		std::cout << "Error: " << glewGetErrorString(res) << "\n";
	}
	// some default enable and disable settings
	glClearColor(0, 0, 0, 1);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDepthFunc(GL_LEQUAL);
	glEnable(GL_PROGRAM_POINT_SIZE);
	glEnable(GL_COLOR_MATERIAL);
	glPointSize(10);

	// init shader and model
	shader = Shader("../Assets/shaders/vertex.vs.glsl", "../Assets/shaders/fragment.fs.glsl");
}
private: System::Void hkoglPanelControl1_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e)
{
	glClearColor(0.5f, 0.5f, 1.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT);

	shader.use();
	// set view matrix
	shader.setUniformMatrix4fv("view", m_camera.GetViewMatrix() * m_camera.GetModelMatrix());
	// set projection matrix
	shader.setUniformMatrix4fv("projection", m_camera.GetProjectionMatrix(aspect));
	// do something here
	// set model matrix
	float scale = 0.1f;
	shader.setUniformMatrix4fv("model", glm::scale(vec3(scale, scale, scale)));

	// render 
	if (meshInited) {
		meshObjectPtr->Render(shader, currentMode);
	}
}

private: System::Void hkoglPanelControl1_Resize(System::Object^ sender, System::EventArgs^ e) {
	float width = (float)hkoglPanelControl1->Size.Width;
	float height = (float)hkoglPanelControl1->Size.Height;
	aspect = width * 1.0f / height;
	m_camera.SetWindowSize(width, height);
}
private: System::Void hkoglPanelControl1_KeyPress(System::Object^ sender, System::Windows::Forms::KeyPressEventArgs^ e) {
	m_camera.keyEvents((unsigned char)e->KeyChar);
	std::cout << "Key " << e->KeyChar << " is pressed\n";

	if (e->KeyChar == 'o') {
		if (meshInited) {
			meshObjectPtr->PrintSkeletonMeshInfo();
			
			std::cout << "The Current SSM Rate is " << currentSSMRate << '\n';
		}
	}

	else if (e->KeyChar == '1') {
		if (meshInited) {
			if (currentSSMRate > 0) currentSSMRate--;
			meshObjectPtr->SetSSMRate(currentSSMRate);
		}
	}

	else if (e->KeyChar == '2') {
		if (meshInited) {
			if (currentSSMRate < meshObjectPtr->GetSkeletonMeshsSize() - 1) currentSSMRate++;
			meshObjectPtr->SetSSMRate(currentSSMRate);
		}
	}
	else if (e->KeyChar == 'a') {
		meshObjectPtr->ResetModel();
		currentMode = Mode::QEM;
	}
	else if (e->KeyChar == 'b') {
		currentMode = Mode::SE;
		//meshObjectPtr->ResetModel(50);
		for (int i = 0; i < 3; i++) {
			meshObjectPtr->InitSE(50);
			//meshObjectPtr->Parameterization();
		}
	}
}
private: System::Void hkoglPanelControl1_MouseDown(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	if (e->Button == System::Windows::Forms::MouseButtons::Left) {
		m_camera.mousePressEvent(GLUT_LEFT_BUTTON, e->Location.X, e->Location.Y);
		printf("Mouse %d is pressed at (%d, %d)\n", GLUT_LEFT_BUTTON, e->Location.X, e->Location.Y);
	}
	else if (e->Button == System::Windows::Forms::MouseButtons::Middle) {
		m_camera.mousePressEvent(GLUT_MIDDLE_BUTTON, e->Location.X, e->Location.Y);
		printf("Mouse %d is pressed at (%d, %d)\n", GLUT_MIDDLE_BUTTON, e->Location.X, e->Location.Y);
	}
}
private: System::Void hkoglPanelControl1_MouseUp(System::Object^ sender, System::Windows::Forms::MouseEventArgs^ e) {
	if (e->Button == System::Windows::Forms::MouseButtons::Left) {
		m_camera.mouseReleaseEvent(GLUT_LEFT_BUTTON, e->Location.X, e->Location.Y);
		printf("Mouse %d is released at (%d, %d)\n", GLUT_LEFT_BUTTON, e->Location.X, e->Location.Y);
	}
	else if (e->Button == System::Windows::Forms::MouseButtons::Middle) {
		m_camera.mouseReleaseEvent(GLUT_MIDDLE_BUTTON, e->Location.X, e->Location.Y);
		printf("Mouse %d is released at (%d, %d)\n", GLUT_MIDDLE_BUTTON, e->Location.X, e->Location.Y);
	}
}
private: System::Void hkoglPanelControl1_MouseMove(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	m_camera.mouseMoveEvent(e->Location.X, e->Location.Y);
}
private: System::Void hkoglPanelControl1_MouseWheel(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	m_camera.wheelEvent(-e->Delta);
}
private: System::Void loadModelToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{
	openModelDialog->Filter = "Model(*.obj)|*obj";
	openModelDialog->Multiselect = false;
	openModelDialog->ShowDialog();
}
private: System::Void openModelDialog_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e)
{
	std::string filename;
	MarshalString(openModelDialog->FileName, filename);

	// do load model action
	meshObjectPtr = new MeshObject();
	meshObjectPtr->Init(filename);
	meshInited = true;

	this->trackBar1->Minimum = 0;
	this->trackBar1->Maximum = meshObjectPtr->GetModelsSize() - 1;

	this->trackBar2->Minimum = 0;
	this->trackBar2->Maximum = meshObjectPtr->GetSkeletonMeshsSize() - 1;
}
private: System::Void saveModelToolStripMenuItem_Click(System::Object^  sender, System::EventArgs^  e)
{
	saveModelDialog->Filter = "Model(*.obj)|*obj";
	saveModelDialog->ShowDialog();
}
private: System::Void saveModelDialog_FileOk(System::Object^  sender, System::ComponentModel::CancelEventArgs^  e)
{
	std::string filename;
	MarshalString(saveModelDialog->FileName, filename);
	// do save file action
	if (!OpenMesh::IO::write_mesh(meshObjectPtr->modelToRender->mesh, filename))
	{
		std::cerr << "write error\n";
		exit(1);
	}
}

private: System::Void timer1_Tick(System::Object^ sender, System::EventArgs^ e) {
	// update the opengl's render
	hkoglPanelControl1->Invalidate();

	// update the mesh information part
	if (meshObjectPtr) {
		if (currentMode == Mode::QEM) {
			this->label3->Text = System::Convert::ToString(meshObjectPtr->GetVerticesNumber());
			this->label4->Text = System::Convert::ToString(meshObjectPtr->GetEdgesNumber());
			this->label6->Text = System::Convert::ToString(meshObjectPtr->GetFacesNumber());
		}
		else if (currentMode == Mode::SE) {
			this->label3->Text = System::Convert::ToString(meshObjectPtr->SEModel.mesh.n_vertices());
			this->label4->Text = System::Convert::ToString(meshObjectPtr->SEModel.mesh.n_edges());
			this->label6->Text = System::Convert::ToString(meshObjectPtr->SEModel.mesh.n_faces());
		}
		else if (currentMode == Mode::SSM) {
			this->label3->Text = System::Convert::ToString(meshObjectPtr->skeletonMeshToRender->GetUndeletedVerticesNumber());
			this->label4->Text = System::Convert::ToString(meshObjectPtr->skeletonMeshToRender->GetUndeletedEdgesNumber());
			this->label6->Text = "(N/A)";
		}
	}
	else {
		this->label3->Text = "(N/A)";
		this->label4->Text = "(N/A)";
		this->label6->Text = "(N/A)";
	}
	
}
private: System::Void trackBar1_ValueChanged(System::Object^ sender, System::EventArgs^ e) {
	meshObjectPtr->SetQEMRate(this->trackBar1->Value);

	// update the label
	this->label8->Text = System::Convert::ToString(this->trackBar1->Value);
}
private: System::Void button1_Click(System::Object^ sender, System::EventArgs^ e) {
	// skeleton extraction
	currentMode = Mode::SE;
	//meshObjectPtr->InitSE(80);
}
private: System::Void trackBar2_ValueChanged(System::Object^ sender, System::EventArgs^ e) {
	meshObjectPtr->SetSSMRate(this->trackBar2->Value);

	// update the label
	this->label16->Text = System::Convert::ToString(this->trackBar2->Value);
}
private: System::Void qEMToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e) {
	currentMode = Mode::QEM;

	// lock the trackBar2
	this->trackBar2->Enabled = false;
	this->trackBar1->Enabled = true;
}
private: System::Void sSMToolStripMenuItem_Click(System::Object^ sender, System::EventArgs^ e) {
	currentMode = Mode::SSM;

	// lock the trackBar1
	this->trackBar1->Enabled = false;
	this->trackBar2->Enabled = true;
}
};
}
