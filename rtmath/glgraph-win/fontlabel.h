/* fontlabel - my solution to fonts on windows
   All text will have this as a base class. Derived classes will determine 
   if an outline or bitmapped font is used.
   When drawing time approaches, the device context and window handle are 
   already set up, so the windows routines just need to be called.
   */
#pragma once
#include <Windows.h>

using namespace System;
using namespace System::ComponentModel;
using namespace System::Collections;
using namespace System::Diagnostics;

#include "pointfconverter.h"
#include "Plottable.h"

namespace glgraphwin {


	public delegate void labelChangedEventHandler(System::Object^ sender);
	public delegate void fontChangedEventHandler(System::Object^ sender);

	public ref class fontlabel abstract : public Plottable
	{
	public:
		fontlabel() 
		{
			// Set the default font settings
			_font = gcnew System::Drawing::Font("Arial",12);
			_text = gcnew System::String("");
			_textColor = gcnew System::Drawing::Color();
			_orientation = 0;
			_initialized = false;
		}

		// TODO: wrap these up nicely so that OnLabelChanged gets called
		[DescriptionAttribute("The font used in rendering text")]
		property System::Drawing::Font^ Font
		{
			System::Drawing::Font^ get()
			{
				return _font;
			}

			void set(System::Drawing::Font^ value)
			{
				_font = value;
				// Redo the font context
				_initFont();
				// Update label
				OnFontChanged();
			}
		}

		[DescriptionAttribute("The text string to display")]
		property System::String^ Text
		{
			System::String^ get()
			{
				return _text;
			}

			void set(System::String^ value)
			{
				_text = value;
				OnLabelChanged();
			}
		}

		[DescriptionAttribute("The text color")]
		property System::Drawing::Color^ TextColor
		{
			System::Drawing::Color^ get()
			{
				return _textColor;
			}

			void set(System::Drawing::Color^ value)
			{
				_textColor = value;
				OnLabelChanged();
			}
		}

		[DescriptionAttribute("The location of the text"),
		TypeConverter(PointFTypeConverter::typeid)]
		property System::Drawing::PointF Location;

		[DescriptionAttribute("The orientation of the text (in radians, ccl)")]
		property float Orientation
		{
			float get() { return _orientation;}
			void set(float value)
			{
				_orientation = value;
				OnLabelChanged();
			}
		}

		event labelChangedEventHandler^ labelChanged;
		event fontChangedEventHandler^ fontChanged;
		
		virtual void Plot() override {};

		// When the hDC changes, we need to know about it.
		property HDC hDC
		{
			virtual void set(HDC hDC) override 
			{ 
				if (hDC == m_hDC) return;
				_deleteFont();
				m_hDC = hDC;
			}
		}

	protected: 
		
		float _orientation;
		System::String^ _text;
		System::Drawing::Font^ _font;
		System::Drawing::Color^ _textColor;
		bool _initialized;

		virtual void _initFont() = 0;
		virtual void _deleteFont() = 0;

		virtual System::Void OnLabelChanged()
			   {
				   labelChanged(this);
			   }

		virtual System::Void OnFontChanged()
		{
			fontChanged(this);
		}

		virtual ~fontlabel()
			   {
			   }
	};

	// Outline font
	public ref class outlineFont : public fontlabel
	{
	public:
		outlineFont();
		virtual ~outlineFont();
		virtual void Plot() override;
		
	protected:
		virtual void _initFont() override;
		virtual void _deleteFont() override;
		virtual System::Void OnFontChanged() override;
	private:
		int m_listbase;
		HFONT font;
	};

}; // end glgraphwin

