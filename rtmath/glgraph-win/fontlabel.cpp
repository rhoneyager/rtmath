#include "stdafx.h"
#include <Windows.h>
#include "fontlabel.h"
#include <gl/GL.h>
#include <gl/GLU.h>

namespace glgraphwin {

	outlineFont::outlineFont(HDC m_hdc)
	{
		m_listbase = 0;
		m_hDC = m_hdc;
	}

	void outlineFont::Plot()
	{
		if (!Visible) return;
		// Print the text
		GLsizei len = GLsizei(_text->Length);
		if (len > 0)
		{
			glPushAttrib(GL_LIST_BIT);
			{
				glListBase(m_listbase);
				glCallLists(len, GL_UNSIGNED_BYTE, 
					//(const GLvoid*)_text->ToCharArray());
					(const GLvoid*) "this is a test");
			}
			glPopAttrib();
		}
	}

	void outlineFont::_initFont()
	{
		GLYPHMETRICSFLOAT gmf[256];
		m_listbase = glGenLists(256);
		BOOL success;
		success = wglUseFontOutlines(
			m_hDC, // from the existing glform
			0,
			256,
			m_listbase,
			0.0,
			0.0,
			WGL_FONT_POLYGONS,
			gmf);
		if (!success)
		{
			glDeleteLists(m_listbase,256);
			m_listbase = 0;
		}
	}

}; // end glgraphwin
