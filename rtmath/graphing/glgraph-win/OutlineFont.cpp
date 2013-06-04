#include "StdAfx.h"
/*
#include "OutlineFont.h"

COutlineFont::COutlineFont(
    CDC* dc, 
    char* fontname)
{
    // Class constructor.
    // Stores each character in its own display list
    // for later drawing via the wglUseFontOutlines() call.

    if (dc && fontname && strlen(fontname) > 0) {

        m_pDC = dc;
        m_listbase = glGenLists(256);

        // Setup the Font characteristics
        LOGFONT logfont;
        GLYPHMETRICSFLOAT gmf[256];

        logfont.lfHeight        = -12;
        logfont.lfWidth         = 0;
        logfont.lfEscapement    = 0;
        logfont.lfOrientation   = logfont.lfEscapement;
        logfont.lfWeight        = FW_NORMAL;
        logfont.lfItalic        = FALSE;
        logfont.lfUnderline     = FALSE;
        logfont.lfStrikeOut     = FALSE;
        logfont.lfCharSet       = ANSI_CHARSET;
        logfont.lfOutPrecision  = OUT_DEFAULT_PRECIS;
        logfont.lfClipPrecision = CLIP_DEFAULT_PRECIS;
        logfont.lfQuality       = DEFAULT_QUALITY;
        logfont.lfPitchAndFamily = FF_DONTCARE|DEFAULT_PITCH;
        strcpy(logfont.lfFaceName, fontname);

        CFont font;
        CFont* oldfont;
        BOOL success = font.CreateFontIndirect(&logfont);
        oldfont = m_pDC->SelectObject(&font);
        if (!success || 
            FALSE == wglUseFontOutlines(
                m_pDC->m_hDC, 
                0, 
                256, 
                m_listbase,
                0.0, 
                0.0, 
                WGL_FONT_POLYGONS,
                gmf)) {
            glDeleteLists(m_listbase, 256);
            m_listbase = 0;
        }
        else {
            m_pDC->SelectObject(oldfont);
        }
    }
}

COutlineFont::~COutlineFont()
{
    // Class destructor.

    glDeleteLists(m_listbase, 256);
    m_listbase = 0;
}


void
COutlineFont::DrawString(char* s)
{
    // Draws the given text string.

    GLsizei len = GLsizei(strlen(s));
    if (s && len > 0) {
		// Must save/restore the list base.
		glPushAttrib(GL_LIST_BIT);{
			glListBase(m_listbase);
			glCallLists(len, GL_UNSIGNED_BYTE, (const GLvoid*)s);
		} glPopAttrib();
    }
}


*/