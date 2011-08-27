#include "stdafx.h"
#include <Windows.h>
#include "fontlabel.h"
#include <gl/GL.h>
#include <gl/GLU.h>
#include <string>
#include "stringConverter.h"
//#include "hfontConverter.h"

namespace glgraphwin {

	outlineFont::outlineFont()
	{
		m_listbase = 0;
		font = 0;
	}

	outlineFont::~outlineFont()
	{
		glDeleteLists(m_listbase, 256);
		m_listbase = 0;
		m_hDC = 0;
	}

	void outlineFont::Plot()
	{
		if (!Visible) return;
		if (!_initialized) _initFont();
		// Print the text
		GLsizei len = GLsizei(_text->Length);
		if (len > 0)
		{
			glBegin(GL_POLYGON);
			glColor4d(_textColor->R / 255.0, _textColor->G / 255.0, _textColor->B / 255.0, _textColor->A / 255.0);
			glEnd();
			std::string txt;
			MarshalString(_text,txt);
			glPushAttrib(GL_LIST_BIT);
			{
				glListBase(m_listbase);
				glCallLists(len, GL_UNSIGNED_BYTE, 
					(const GLvoid*)txt.c_str());
					//(const GLvoid*) "this is a test");
			}
			glPopAttrib();
		}
	}

	System::Void outlineFont::OnFontChanged()
	{
		_deleteFont();
		fontChanged(this);
	}

	void outlineFont::_deleteFont()
	{
		glDeleteLists(m_listbase,256);
		m_listbase = 0;
		_initialized = false;
	}

	void outlineFont::_initFont()
	{
		//HFONT font; // now a class variable
		GLYPHMETRICSFLOAT gmf[256];
		m_listbase = glGenLists(256);
		//font = (HFONT) _font->ToHfont(); // check cast
		font = (HFONT) _font->ToHfont().ToPointer();
		//font = context.marshal_as<HFONT>(_font);
		/*
		font = CreateFont( -height, // height of font
			0, // width of font
			0, // angle of escapement
			0, // orientation angle
			FW_BOLD, // font weight
			FALSE, // italic
			FALSE, // underline
			FALSE, // strikeout
			ANSI_CHARSET, // character set identifier
			OUT_TT_PRECIS, // Output Precision
			CLIP_DEFAULT_PRECIS, // Clipping Precision
			ANTIALIASED_QUALITY, // Output Quality
			FF_DONTCARE|DEFAULT_PITCH, // Family And Pitch
			(LPCWSTR) "Comic Sans MS"); // Font Name
		*/
		SelectObject(m_hDC, font); // Selects The Font We Created
		BOOL success;
		success = wglUseFontOutlines(
			m_hDC, // from the existing glform
			0,     // Starting character
			256,   // ending character
			m_listbase,	// number of display lists
			0.0,		// deviation from the true outlines
			0.0,		// font z thickness
			//WGL_FONT_LINES, // using lines, not polygons
			WGL_FONT_POLYGONS,	// use polygons, not lines
			gmf);		// address of data-receiving buffer
		if (!success)
		{
			_deleteFont();
			return;
		}
		_initialized = true;
	}

}; // end glgraphwin
